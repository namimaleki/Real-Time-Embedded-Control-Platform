#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app_config.h"
#include "shared_state.h"
#include "tasks_load.h"


/* ===================== LOAD TASK (STRESS TEST) ==================== */
/**
 * This task will add cpu stress (intentialloy burn CPU TImE to stimulate real system laod) to see if our CPU 
 * will still meet the deterministic control loop timing. 
 */

static void load_task (void *arg){
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
 
 
void start_load_task() {
    xTaskCreatePinnedToCore(load_task, "load_task", 2048, NULL, LOAD_TASK_PRIO, NULL, 0);
}
