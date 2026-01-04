#include <Arduino.h>

/* 
  On ESP32 (Arduino framework), FreeRTOS is already included and running.
  We still include the headers to access FreeRTOS APIs like creating tasks delays and tick timing.
*/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* ========================= CONFIGURATION =========================== */

/* This is how often the control loop should run. 200 Hz means: 1/200 seconds = 5ms. This is pretty common. */
static const uint32_t CONTROL_HZ = 200;

/* FreeRTOS uses "ticks" for timing. A tick is like the heartbeat of the OS scheduler. pdMS_TO_TICKS converst ms into ticks. */
static const TickType_t CONTROL_PERIOD_TICKS = pdMS_TO_TICKS(1000 / CONTROL_HZ); 

/**
 * Task Priorities : Higher number = Higher priority 
 * 
 * We will set CONTROL_TASK_PRIO to the highest since it is our time critical task and it must run every 5ms no matter what (deterministic).
 * If this misses deadlines then our robot would be misbehaving. 
 * Then we set TELEMETRY_TASK_PRIO to midium priority since this is user interface / debugging output and it can tolerate delays. Shouldn't
 * Interfere with the control loop/
 * Lastly we set LOAD_TASK_PRIO to lowest since this just represents our intential CPU stress for testing should never block high priority work 
 * and will run in the left over cpu time. 
 * So if u recall cpen 331 this is preemption whenever a control task needs CPU it will interrupt lower priority tasks. 
 * 
 */
static const int CONTROL_TASK_PRIO = 8;
static const int TELEMETRY_TASK_PRIO = 2; 
static const int LOAD_TASK_PRIO = 1; 

/**
 * In robotics if timing fails, we must fail safely and that means disabling motors and entering a known safe state. This is a robotic
 * Concept where its better to fail safely rather than to continue unpredictably. So we will use a boolean flag 
 */ 
static volatile bool safe_mode = false; 

/**
 * Real robots have plenty of other tasks so we will need to create a load to test if our control timing stays stable. 
 * We will do this by stimulate system stress ourselves which will be represented by the CPU load level (0 = no load, 1.0 = heavy load)
 */
static volatile float cpu_load_lvl = 0.30f; 


/* ========================== TIMING STATISTICS STRUCT ========================*/

/* We will need a structure to keep track of how well we're meeting our timing goals */
typedef struct {
  uint32_t samples; /* how many control loops have run */
  uint32_t missed_deadlines; /* how many times we were late */

  /* Period measurements (time between control loop iterations) */
  int32_t min_period;
  int32_t max_period; 
  int64_t total_time;
  int32_t last_period_time;
} timing_stats_t; 

static timing_stats_t stats;

/* INitialize the timing stats (the first real measurments will replace these) */
static void stats_init(timing_stats_t *s){
  s->samples = 0; 
  s->missed_deadlines = 0; 
  s->min_period = INT32_MAX;
  s->max_period = INT32_MIN;
  s->total_time = 0; 
  s->last_period_time = 0; 
}


/* ======================== CONTROL TASK (HEART OF THE SYSTEM) ===================== */

/**
 * This task runs at a fixed rate (CONTROL_HZ). In a robot this is wehre we would read sensor data (from a queue),
 * compute control output, or send actuator command. 
 */
void control_task(void *arg){
  (void)arg;

  /* Remember this task runs at a fixed rate and should be deterministic */
  /* In order to caclualte the time since last iteration we need to get the current time. esp_timer_get_time() returns microseconds since boot. */
  int64_t last_us = esp_timer_get_time();

  /* Since we will be using vTaskDelayUNtil (see why below) we need that last wake time */
  /* so our last wake time will be the reference point for periodic scheudling */
  TickType_t last_wake = xTaskGetTickCount();

  /* main control loop this will run forever */
  while (true) {
    /**
     * We must use vTaskDelayUntil(&last_wake, CONTROL_PERIOD_TICKS). This is because vTaskDelay(5) this will sleep for 5 ticks,
     * starting NOW. This is problamatic since if your code takes 2 ticks to run you actually get 7 tick periods and this will just
     * cause drift (period will get longer and longer over time)
     * On the other hand vTaskDelayUntil(&last_wake, 5) this will wake up at last_wake + 5 ticks, so regardless of how long the code takes
     * this will be 5 consistent tick periods which will compensate for the execution time and result in deterministic scheduling. 
     */
    vTaskDelayUntil(&last_wake, CONTROL_PERIOD_TICKS); 

    /* Check if we need to enter safe mode */
    if (safe_mode){
      continue;
    }


    /* Measure the actual time between iterations (we do this so we can make sure actual = expected since it can still vary though to interrupts and cpu load) */
    int64_t current_us = esp_timer_get_time();
    int32_t dt_time = (int32_t)(current_us - last_us); /* time since last iteration */
    last_us = current_us; /* update for next measuremnet */

    /* Update our statistics */
    stats.samples++;
    stats.last_period_time = dt_time;
    stats.total_time += dt_time; 

    if (dt_time < stats.min_period){
      stats.min_period = dt_time; 
    }

    if (dt_time >= stats.max_period){
      stats.max_period = dt_time;
    }

    /* ========= DEADLINE MISS DETECTION ======= */

    /**
     * We need to detect if we're running too slow our expected period is 5000 micro sec = 5 ms 
     * and if the actual period is > 2 * expected then something is seriously wrong and we count it as a deadline miss. 
     * We do 2 * since small variations are normal but large variations indicate system overload so its quite practical. 
     * We can vary this based on test as well. 
     * 
     * For our safety policy for now we will say if we miss 5 deadlines then we enter safe mode.
     */
    const int32_t expected_us = (int32_t)(1000000 / CONTROL_HZ);

    if (dt_time > 2 * expected_us){
      stats.missed_deadlines++;

      if (stats.missed_deadlines >= 5){
        safe_mode = true;
      }
    }

  }
}

/* ================================= TELEMETRY TASK (OBSERVABILITY) ====================== */

/**
 * This task prints status and is used to observe the statistics. This is important for debugging
 */
void telemetry_task(void *arg){
  (void)arg; 

  while (true) {
    /* run once per second this isn't timing critical so we can just use taskdelay */
    vTaskDelay(pdMS_TO_TICKS(1000));

    /* if control hasn't run yet then there isn't anythign to report */
    uint32_t n = stats.samples;
    if (n == 0){
      Serial.println("[telemetry] waiting for control samples...");
      continue;
    }

    /* Snapshot current stats */
    int32_t min_dt = stats.min_period;
    int32_t max_dt = stats.max_period;
    int32_t last_dt = stats.last_period_time;
    uint32_t misses = stats.missed_deadlines;
    
    /* Calculate average */
    float avg_dt = (float)stats.total_time / (float)n;

    /**
     * EXPECTED PERIOD
     * For 200 Hz: 1,000,000 µs / 200 = 5,000 µs
     */
    const float expected_us = 1000000.0f / (float)CONTROL_HZ;
    
    /**
     * JITTER CALCULATION
     * 
     * Jitter = max - min
     * 
     * Why jitter matters:
     * In a PID controller:
     * - P term: Not affected
     * - I term: Slightly affected
     * - D term: VERY affected (derivative = change/time)
     * 
     * If time varies, derivative calculation is noisy.
     * Result: Jerky, unstable control.
     */
    const float jitter_us = (float)(max_dt - min_dt);

    /**
     * Professional telemetry format:
     * - Clear section headers
     * - All relevant metrics
     * - Units clearly labeled
     */
    Serial.println("========== RTOS TIMING STATISTICS ==========");
    Serial.printf("System Mode:     %s\n", safe_mode ? "SAFE" : "RUN");
    Serial.printf("Control Rate:    %lu Hz (%.1f µs expected)\n", (unsigned long)CONTROL_HZ, expected_us);
    Serial.println("--------------------------------------------");
    Serial.printf("Period (µs):     last=%-6ld  min=%-6ld  max=%-6ld  avg=%.1f\n", (long)last_dt, (long)min_dt, (long)max_dt, avg_dt);
    Serial.printf("Jitter:          %.1f µs\n", jitter_us);
    Serial.printf("Samples:         %lu\n", (unsigned long)n);
    Serial.printf("Missed Deadlines: %lu\n", (unsigned long)misses);
    Serial.printf("CPU Load:        %.0f%%\n", cpu_load_lvl * 100.0f);
    Serial.println("============================================");

    /**
     * SAFE MODE ALERT
     * 
     * If system entered SAFE mode:
     * - Alert the user
     * - Explain how to recover
    
     */
    if (safe_mode) {
      Serial.println("[telemetry] SAFE mode entered due to missed deadlines.");
    }
  }
}


/* ===================== LOAD TASK (STRESS TEST) ==================== */
/**
 * This task will add cpu stress (intentialloy burn CPU TImE to stimulate real system laod) to see if our CPU 
 * will still meet the deterministic control loop timing. 
 */

 void load_task (void *arg){
  (void)arg; 

  while (true) {
    /**
     * We need to calculate the busy/idle time. 
     * Total cycle: 50 ms so busy time will be : cpu_load_level * 50ms and the idle time is the remainder
     */

     int busy_ms = (int)(cpu_load_lvl * 50.0f); 
     int idle_ms = 50 - busy_ms; 

     /* So we will spin in a loop doing nothing to burn cpu time */
     int64_t start = esp_timer_get_time();
     while ((esp_timer_get_time() - start) < (int64_t)busy_ms * 1000LL){
      /* no op does nothing but keeps the CPU busy */
      asm volatile("nop");
     }

     /* then we sleep for idle_ms to yeild the CPU */
     vTaskDelay(pdMS_TO_TICKS(idle_ms));
  }
 }
 

/* ===================== ARDUINO ENTRYPOINTS =================== */
void setup() {
  Serial.begin(115200);

  /*
    Small delay so that when you open Serial Monitor, you don't
    miss the first few print messages.
  */
  delay(800);

  Serial.println("\nESP32 RTOS Week 1: starting...");

  // Initialize stats before tasks start using them
  stats_init(&stats);

  /*
    Task creation details:

    xTaskCreatePinnedToCore(
      task_function,
      "task_name",
      stack_size_bytes,
      task_parameter,
      priority,
      task_handle_out,
      core_id
    )

    On ESP32 there are 2 cores (0 and 1). Pinning is optional.
    Pinning helps reduce interference: we keep control on core 1,
    load on core 0.
  */
  xTaskCreatePinnedToCore(control_task, "control_task", 4096, NULL, CONTROL_TASK_PRIO, NULL, 1);

  xTaskCreatePinnedToCore(telemetry_task, "telemetry_task", 4096, NULL, TELEMETRY_TASK_PRIO, NULL, 1);

  xTaskCreatePinnedToCore(load_task, "load_task", 2048, NULL, LOAD_TASK_PRIO, NULL, 0);

  Serial.println("Tasks created. Open Serial Monitor.");
}

void loop() {
  /*
    We intentionally DO NOT use Arduino's loop() for application logic.

    Why:
      In RTOS-based firmware, you avoid putting major logic in loop().
      Instead, each responsibility becomes a task.

    We leave loop doing nothing important.
  */
  vTaskDelay(pdMS_TO_TICKS(1000));
}
