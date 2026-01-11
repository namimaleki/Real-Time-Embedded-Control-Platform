
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app_config.h"
#include "shared_state.h"
#include "tasks_control.h"
#include "ipc.h"
#include "imu_driver.h"
#include <math.h>

#include "pwm_driver.h"


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

  /* We keep the latest known sample so control can run even if the queue doesn't have a new sample every single cycle */
  imu_sample_t last_sample = {};
  bool have_sample = false; 

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
      /* Need to ensure actuators are OFF */
      pwm_emergency_stop_all();
      continue;
    }

    /* ============= SENSOR INPUT (IMU QUEUE) ========== */
   /**
     * sensor_task pushes imu_sample_t into imu_queue.
     * control_task pulls the LATEST sample out.
     *
     * We use NON-BLOCKING receive (timeout = 0) because:
     * - control_task is timing-critical
     * - it must never stall waiting for sensor data
     *
     * If there is no new sample right now, we just keep last_sample.
     */

     imu_sample_t s; 
     if (xQueueReceive(imu_queue, &s, 0) == pdTRUE) {
      last_sample = s; 
      have_sample = true; 
     }
     if (!have_sample){
      /* if we don't have one sample yet don't run the control logic to avoid using zeros or producing fake actuator outputs */
      continue; 
     }

   /**
     * PID needs a dt (time step).
     * We run the loop at CONTROL_HZ, so dt is deterministic:
     * dt = 1 / CONTROL_HZ seconds.
     *
     * We'll use this dt for PID math (more stable than using jittery measured dt_time).
     */
    const float dt = 1.0f / (float)CONTROL_HZ;
     

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
        pwm_emergency_stop_all();
        continue;
      }
    }

    /* ===================== ACTUATOR OUTPUT (PWM) ===================== */
    /**
     *  we need a physical output that proves:
     * - our control task can command an actuator
     * - PWM driver works
     * - scheduling stays stable while controlling hardware
     *
     * We'll generate a simple triangle wave duty cycle:
     * 0% -> 100% -> 0% -> repeat
     *
     * This makes an LED "breathe" (brightness goes up and down).
     *
     * Later:
     * - This block becomes: compute PID output and set motor PWM.
     */
    static float duty = 0.0f;    // duty in percent (0..100)
    static float step = 1.0f;    // how fast we change duty per control iteration

    duty += step;

    if (duty >= 100.0f) {
      duty = 100.0f;
      step = -step;
    } else if (duty <= 0.0f) {
      duty = 0.0f;
      step = -step;
    }

    /**
     * This is the actual actuator command.
     * Channel 0 is what we configured in pwm_init() in main.cpp.
     */
    pwm_set_duty(0, duty);

  }
}

void start_control_task() {
  /*
    Pinning control task to core 1 helps reduce interference from load task on core 0.
    This is a practical ESP32 trick.
  */
  xTaskCreatePinnedToCore(control_task, "control_task", CONTROL_TASK_STACK, NULL, CONTROL_TASK_PRIO, NULL, CONTROL_TASK_CORE);
}