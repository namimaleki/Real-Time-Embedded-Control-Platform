#pragma once 
#include <climits> 
#include <cstdint>

/* ================ CONFIGURATION ============== */

/* This is how often the control loop should run. 200 Hz means: 1/200 seconds = 5ms. This is pretty common. */
static const uint32_t CONTROL_HZ = 500;

/* Control period in ticks (200Hz -> 5ms) */ 
static const TickType_t CONTROL_PERIOD_TICKS = pdMS_TO_TICKS(1000 / CONTROL_HZ);


/* We want our sensor task (PRODUCER) to run periodically as well we will set it to the same freq as control for now */
static const uint32_t SENSOR_HZ = 500;
static const TickType_t SENSOR_PERIOD_TICKS = pdMS_TO_TICKS(1000 / SENSOR_HZ);

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
 * Since the sensor task is important aswell we will set its priority just below the control task since 
 * the control task is time critical.
 */
static const int CONTROL_TASK_PRIO = 8;
static const int TELEMETRY_TASK_PRIO = 2; 
static const int LOAD_TASK_PRIO = 1; 
static const int SENSOR_TASK_PRIO = 7;

/* Task stack sizes (bytes). If tasks crash randomly, stack may be too small. */
static const uint32_t CONTROL_TASK_STACK   = 4096;
static const uint32_t TELEMETRY_TASK_STACK = 4096;
static const uint32_t LOAD_TASK_STACK      = 2048;
static const uint32_t SENSOR_TASK_STACK    = 3072; 

/* Which cores to pin tasks to (ESP32 has 2 cores: 0 and 1) */
static const int CONTROL_TASK_CORE   = 1;
static const int TELEMETRY_TASK_CORE = 1;
static const int LOAD_TASK_CORE      = 0;
static const int SENSOR_TASK_CORE    = 1; 


/* IMU CONFIG */
/* we will use a flag for now and when it is 1 we will use simulated IMU data and once the imu arrives we set to 0 to use real i2C driver code */
#define USE_IMU_STUB 1 

/* I2C address (wont be used in stub mode) (0x68 if AD0 pin is low and 0x69 if AD0 pin is high) */
#define IMU_I2C_ADDR 0x68