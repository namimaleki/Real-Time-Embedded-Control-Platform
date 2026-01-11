#include <Arduino.h>
#include "ipc.h"

QueueHandle_t imu_queue = NULL;

bool ipc_init(void) {
    imu_queue = xQueueCreate(IMU_QUEUE_LENGTH, sizeof(imu_sample_t));
    if (imu_queue == NULL) {
        Serial.println("[ipc] ERROR: failed to create imu_queue");
        return false;
    }
    Serial.printf("[week3_ipc] imu_queue created len=%d item=%u bytes\n",
                IMU_QUEUE_LENGTH, (unsigned)sizeof(imu_sample_t));
    return true;  
}