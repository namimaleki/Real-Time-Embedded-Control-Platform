#pragma once 
#include <climits> 
#include <cstdint>

/* ================ CONFIGURATION ============== */

/* This is how often the control loop should run. 200 Hz means: 1/200 seconds = 5ms. This is pretty common. */
static const uint32_t CONTROL_HZ = 500;

/* Control period in ticks (200Hz -> 5ms) */ 
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

/* Task stack sizes (bytes). If tasks crash randomly, stack may be too small. */
static const uint32_t CONTROL_TASK_STACK   = 4096;
static const uint32_t TELEMETRY_TASK_STACK = 4096;
static const uint32_t LOAD_TASK_STACK      = 2048;

/* Which cores to pin tasks to (ESP32 has 2 cores: 0 and 1) */
static const int CONTROL_TASK_CORE   = 1;
static const int TELEMETRY_TASK_CORE = 1;
static const int LOAD_TASK_CORE      = 0;

