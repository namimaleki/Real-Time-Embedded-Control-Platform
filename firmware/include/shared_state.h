#pragma once 

#include "timing_stats.h"


/* ============================== SHARED STATE ======================= */
/* These are the global variables that are shared across tasks. */

/**
 * In robotics if timing fails, we must fail safely and that means disabling motors and entering a known safe state. This is a robotic
 * Concept where its better to fail safely rather than to continue unpredictably. So we will use a boolean flag 
 */ 
extern volatile bool safe_mode; 

/**
 * Real robots have plenty of other tasks so we will need to create a load to test if our control timing stays stable. 
 * We will do this by stimulate system stress ourselves which will be represented by the CPU load level (0 = no load, 1.0 = heavy load)
 */
extern volatile float cpu_load_lvl; 

/* Timing stats updated by control task and printed by telemetry */
extern timing_stats_t stats;

