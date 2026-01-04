
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app_config.h"
#include "shared_state.h"
#include "tasks_control.h"

/* ======================== CONTROL TASK (HEART OF THE SYSTEM) ===================== */

/**
 * This task runs at a fixed rate (CONTROL_HZ). In a robot this is wehre we would read sensor data (from a queue),
 * compute control output, or send actuator command. 
 */
static void control_task(void *arg){
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

void start_control_task() {
  /*
    Pinning control task to core 1 helps reduce interference from load task on core 0.
    This is a practical ESP32 trick.
  */
  xTaskCreatePinnedToCore(control_task, "control_task", CONTROL_TASK_STACK, NULL, CONTROL_TASK_PRIO, NULL, CONTROL_TASK_CORE);
}