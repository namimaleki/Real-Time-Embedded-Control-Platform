#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app_config.h"
#include "imu_driver.h"
#include "ipc.h"
#include "tasks_sensor.h"


/* ====================== SENSOR TASK (PRODUCER) ============== */

/** 
 * This task is responsible for reading the IMU sensor periodically and then sends the 
 * latest IMU sample to a freertos queue. 
 * So the sensor tasks basically communicates with the sensor hardware and produces the measurements
 * whereas the control tasks will consume the measurements and compute the actuator output which is like 
 * real robotics firmware.
 * 
 * We will use a queue since it is a thread safe way to pass data between tasks.It avoids race conditions 
 * (no shared state being read and written at the same time) and it provides bufferring. 
 * 
 * The sensor task should not block for long periods and if the queue is full we drop the sample since the latest 
 * data is more importatn than all the data 
 */
static void sensor_task(void* arg) {
    (void)arg; 

    TickType_t last_wake = xTaskGetTickCount(); 

    /* We will count drops to detect if control is not consuming the data fast enough. */
    /* if drops increase it means the queue is too smal, control task isnt reading often enough or cpu load might be too high */
    uint32_t drops = 0; 

    while (true) {
        /* Periodic schedule so using vTaskDelayUntil (similar to control task) */
        vTaskDelayUntil(&last_wake, SENSOR_PERIOD_TICKS);

        /* read the imu sample by using imu_read */
        imu_sample_t sample; 
        imu_result_t r = imu_read(&sample); 

        if (r != IMU_OK){
            /**
             * If the read fails, we skip sending. We could later add logic 
             * where we measure the number of times the read fails and then 
             * possibly triger SAFE mode after reaching a threshhold. 
             * 
             * for now lets just skip the cycle 
             */
            
            continue; 
        }

        /**
         * Non-blocking send to the queue:
         * timeout = 0 means do not wait if queue is full if full we drop the sample
         * Remember we drop instead of block since blocking canc ause sensor task to miss its own schedule. 
         * For control newest measurment is most valuable.
         */
        if (xQueueSend(imu_queue, &sample, 0) != pdTRUE) {
            drops++; 

            if (drops % 100 == 0) {
                Serial.printf("[sensor_task] WARNING: imu_queue full (%lu drops)\n", (unsigned long)drops);
            }
        }
    }
}


/* start func */
void start_sensor_task(void) {
    /**
     * This task requires the IMU queue to exist so ipc_init() should create
     * imu_queue before we start this task. 
     */
    if (imu_queue == NULL) {
        Serial.println("[sensor_task] ERROR: imu_queue is NULL (call ipc_init first)");
        return;
    }

   /**
   * Create the task pinned to a specific core.
   *
   * Parameters:
   * - sensor_task: function
   * - "sensor_task": name (useful in debug tools)
   * - SENSOR_TASK_STACK: stack bytes
   * - NULL: argument
   * - SENSOR_TASK_PRIO: priority
   * - NULL: handle (we don't need it yet)
   * - SENSOR_TASK_CORE: which core to run on
   */
  xTaskCreatePinnedToCore(sensor_task, "sensor_task", SENSOR_TASK_STACK, NULL, SENSOR_TASK_PRIO, NULL, SENSOR_TASK_CORE);
}