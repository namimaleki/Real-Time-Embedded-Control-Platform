#pragma once 
#include <climits> 
#include <cstdint>


/* ========================== TIMING STATISTICS STRUCT ========================*/

/* We will need a structure to keep track of how well we're meeting our timing goals */
typedef struct {
  uint32_t samples; /* how many control loops have run */
  uint32_t missed_deadlines; /* how many times we were late */

  /* Period measurements (time between control loop iterations) */
  int32_t min_period;
  int32_t max_period; 
  int64_t total_time;
  int32_t last_period_time;
} timing_stats_t; 


/* INitialize the timing stats (the first real measurments will replace these) */
static void stats_init(timing_stats_t *s){
  s->samples = 0; 
  s->missed_deadlines = 0; 
  s->min_period = INT32_MAX;
  s->max_period = INT32_MIN;
  s->total_time = 0; 
  s->last_period_time = 0; 
}
