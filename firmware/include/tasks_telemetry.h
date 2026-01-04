#pragma once 


/* Telemetry task module: prints timing stats once per second, and helps us observe jtter
   and missed deadlines (used for debugging and observating the system state) 
*/

void start_telemetry_task();