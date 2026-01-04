#include <Arduino.h>
#include <climits>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app_config.h"
#include "shared_state.h"
#include "tasks_telemetry.h"


/* ================================= TELEMETRY TASK (OBSERVABILITY) ====================== */

/**
 * This task prints status and is used to observe the statistics. This is important for debugging
 */
static void telemetry_task(void *arg){
  (void)arg; 

  while (true) {
    /* run once per second this isn't timing critical so we can just use taskdelay */
    /* print every 5 seconds*/
    vTaskDelay(pdMS_TO_TICKS(5000));

    /* if control hasn't run yet then there isn't anythign to report */
    uint32_t n = stats.samples;
    if (n == 0){
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
     * JITTER CALCULATION : Jitter = max - min
     * This is important in control since derivative term is sensitive to dt variation (change / time)
     * If dt varies, derivative becomes noisy -> jerky control 
     */
    const float jitter_us = (float)(max_dt - min_dt);

    /**
     * Professional telemetry format:
     * - Clear section headers
     * - All relevant metrics
     * - Units clearly labeled
     */
    Serial.println();
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

void start_telemetry_task() {
    xTaskCreatePinnedToCore(telemetry_task, "telemetry_task", TELEMETRY_TASK_STACK, NULL, TELEMETRY_TASK_PRIO, NULL, TELEMETRY_TASK_CORE);
}
