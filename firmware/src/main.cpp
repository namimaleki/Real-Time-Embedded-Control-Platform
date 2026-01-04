#include <Arduino.h>

/* 
  On ESP32 (Arduino framework), FreeRTOS is already included and running.
  We still include the headers to access FreeRTOS APIs like creating tasks delays and tick timing.
*/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app_config.h"
#include "shared_state.h"
#include "tasks_control.h"
#include "tasks_telemetry.h"
#include "tasks_load.h"

#include <climits>


/* =================== GLOBAL SHARED STATE ================ */
/**
 * In robotics if timing fails, we must fail safely and that means disabling motors and entering a known safe state. This is a robotic
 * Concept where its better to fail safely rather than to continue unpredictably. So we will use a boolean flag 
 */ 
volatile bool safe_mode = false; 

/**
 * Real robots have plenty of other tasks so we will need to create a load to test if our control timing stays stable. 
 * We will do this by stimulate system stress ourselves which will be represented by the CPU load level (0 = no load, 1.0 = heavy load)
 */
volatile float cpu_load_lvl = 0.80f; 

timing_stats_t stats;


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

  
  /*On ESP32 there are 2 cores (0 and 1). Pinning is optional.
    Pinning helps reduce interference: we keep control on core 1,
    load on core 0.
  */

  start_control_task();
  start_telemetry_task();
  start_load_task();

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
