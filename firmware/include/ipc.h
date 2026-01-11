#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "imu_driver.h"

/* Queue handle shared by tasks */
extern QueueHandle_t imu_queue; 

/* Create queue objects (call in setup before tasks start) */
bool ipc_init(void); 

#define IMU_QUEUE_LENGTH 10 
