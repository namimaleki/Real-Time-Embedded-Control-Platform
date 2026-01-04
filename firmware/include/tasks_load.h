#pragma once

/* Load task module: INtentionally burns CPU to simulate real system load and lets us test whether 
   the control task remains deterministic */

void start_load_task();