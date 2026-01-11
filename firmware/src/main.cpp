#include <Arduino.h>

/* 
  On ESP32 (Arduino framework), FreeRTOS is already included and running.
  We still include the headers to access FreeRTOS APIs like creating tasks delays and tick timing.
*/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app_config.h"
#include "shared_state.h"
#include "tasks_control.h"
#include "tasks_telemetry.h"
#include "tasks_load.h"
#include "imu_driver.h"
#include "ipc.h"
#include "tasks_sensor.h"


#include "pwm_driver.h"


#include <climits>


/* =================== GLOBAL SHARED STATE ================ */
/**
 * In robotics if timing fails, we must fail safely and that means disabling motors and entering a known safe state. This is a robotic
 * Concept where its better to fail safely rather than to continue unpredictably. So we will use a boolean flag 
 */ 
volatile bool safe_mode = false; 

/**
 * Real robots have plenty of other tasks so we will need to create a load to test if our control timing stays stable. 
 * We will do this by stimulate system stress ourselves which will be represented by the CPU load level (0 = no load, 1.0 = heavy load)
 */
volatile float cpu_load_lvl = 0.80f; 

timing_stats_t stats;

/* Latest IMU pitch estimate (degrees), computed in control_task. We will store this as a GLOBAL so telemetry_task can print it 
   and control task can update it. Also note that we use volatile since mulktple takss acces it (volatile prevents compiler form caching it) */
volatile float imu_pitch_deg = 0.0f; 


/* ===================== ARDUINO ENTRYPOINTS =================== */
void setup() {
  Serial.begin(115200);

  /*
    Small delay so that when you open Serial Monitor, you don't
    miss the first few print messages.
  */
  delay(800);

  Serial.println("\nESP32 RTOS Week 1: starting...");

  /* Initialize stats before tasks start using them */
  stats_init(&stats);

  /* ============ IPC (QUEUE) SETUP ======== */
  /** 
   * NOTES FOR MYSELF: Remember IPC (inter process communication or ig in RTOS world inter task ccommunication) is used so that 
   * the sensor_task which produces the imu_sample_t into the queue and control_task (consumer) pulls imu_sample_t out of the queue
   * so it allows these tasks to communicate with one another. Also the queue must exist before starting 
   * sensor or control tasks or else they will try to use a null queuee and that's trouble
   */
  if (!ipc_init()) {
    Serial.println("[ipc] init failed -> entering SAFE mode"); 
    safe_mode = true; 
  }

  /* ========= IMU DRIVER INIT ========= */
  /* Initialize the IMU driver */
  imu_config_t imu_cfg = {.i2c_addr = MPU6050_I2C_ADDR, .i2c_clock_hz = 400000};
  imu_result_t ir = imu_init(&imu_cfg); 
  if (ir != IMU_OK) {
    Serial.printf("[imu] init failed: %s\n", imu_error_to_string(ir));
    Serial.println("[imu] entering SAFE mode");
    safe_mode = true;  /* fail-safe: don’t trust control if sensor layer is broken */
  }
  else {
    Serial.println("[imu] init ok");
  }

  /* ========== PWM SETUP ======== */
  /**
   * We now add a real "actuator output" to the system.
   * For now the actuator is just an LED (brightness controlled via PWM).
   *
   * This is still the same skill as motor control:
   * - motors use PWM for speed/torque (through a motor driver)
   * - LEDs use PWM for brightness
   *
   * IMPORTANT: We initialize hardware drivers BEFORE starting tasks
   * so tasks don't try to use uninitialized peripherals.
   */
  pwm_config_t pwm= {.gpio_pin = 18, .channel = 0, .frequency = 5000, .resolution_bits = 8, .initial_duty = 0.0f };

  pwm_result_t result = pwm_init(&pwm);
  if (result != PWM_OK){
    Serial.printf("[pwm] init failed: %s\n", pwm_error_to_string(result));
    Serial.println("[pwm] Entering SAFE mode because actuator driver did not init.");
    safe_mode = true;  // fail safe: if actuator driver is broken, don't run control
  }
  else {
    Serial.println("[pwm] init ok (GPIO18, channel 0)");
  }
  /*On ESP32 there are 2 cores (0 and 1). Pinning is optional.
    Pinning helps reduce interference: we keep control on core 1,
    load on core 0.
  */

  start_control_task();
  start_sensor_task();
  start_telemetry_task();
  start_load_task();

  Serial.println("Tasks created. Open Serial Monitor.");
}

void loop() {
  /*
    We intentionally DO NOT use Arduino's loop() for application logic.

    Why:
      In RTOS-based firmware, you avoid putting major logic in loop().
      Instead, each responsibility becomes a task.

    We leave loop doing nothing important.
  */
  vTaskDelay(pdMS_TO_TICKS(1000));
}
