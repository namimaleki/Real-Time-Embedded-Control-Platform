# Real Time Embedded Control Platform 

I started this project because robotics is genuinely what I’m most excited about, and I wanted a personal project that forces me to build embedded systems the way they’re built in real robots — not just Arduino “loop code”, but actual real-time tasks, driver layers, and a control loop that has to run deterministically.

The goal of this repo is to grow my firmware / embedded skillset by building a small “robotics control stack” on an ESP32 using FreeRTOS. It’s structured like real robot firmware: sensor sampling, inter-task communication, a fixed-rate control loop, and actuator output through a PWM driver, plus timing telemetry to prove it’s actually behaving like a real-time system.

---

## What this project does (high-level)

This firmware is organized into a few core pieces:

- **FreeRTOS task-based architecture**
  - A fixed-rate control task (deterministic loop)
  - A sensor task (producer)
  - A telemetry task (observability/logging)
  - A load task (CPU stress testing)

- **Driver layer (hardware abstraction)**
  - PWM driver (ESP32 LEDC peripheral) for actuator-style output
  - IMU driver interface:
    - Stub backend (so I can develop and test without hardware)
    - Real MPU-6050 backend (register-based I2C driver) once the IMU arrives

- **IPC (inter-task communication)**
  - Queue-based messaging between tasks (sensor → control)
  - Designed so tasks don’t share raw data unsafely

- **Timing + real-time verification**
  - Period tracking (min/max/avg dt)
  - Jitter measurement
  - Deadline miss detection
  - SAFE mode behavior if timing becomes unsafe

---

## Why I built it this way

I wanted a project that demonstrates the skills that show up in robotics / firmware roles:

- Writing code that runs **at a fixed rate** (not “best effort”)
- Understanding how scheduling + priority affects the system
- Building **drivers with clean APIs** and error handling
- Designing a system where tasks communicate safely (IPC)
- Proving behavior with instrumentation (telemetry, timing stats)
- Thinking about safety (SAFE mode / emergency stop patterns)

Even though the final “robot” application is small, the software structure is intentionally what you’d see in real embedded systems.

---

## System architecture (how the pieces connect)

At runtime the data/control flow looks like this:

1) **sensor_task (Producer)**
   - Runs at the same rate as the control loop
   - Reads IMU data using `imu_read()`
     - For now: stub backend generates realistic tilt + gyro signals
     - Later: real MPU-6050 backend reads real registers over I2C
   - Sends `imu_sample_t` into a FreeRTOS queue

2) **control_task (Consumer / Real-time loop)**
   - Runs at a fixed rate using `vTaskDelayUntil`
   - Pulls the latest IMU sample from the queue (non-blocking)
   - Computes control output (PID in progress)
   - Outputs actuator command via PWM (LEDC driver)
   - Tracks timing and triggers SAFE mode if deadlines are repeatedly missed

3) **telemetry_task (Observability)**
   - Runs slower (ex: every few seconds)
   - Prints timing stats + control/sensor values in a readable format
   - This is also where demo output will come from (plots / logs)

4) **load_task (Stress test)**
   - Intentionally burns CPU time to simulate a “busy robot”
   - Lets me verify the control task still runs deterministically under load

---

## Repo structure

- `src/`
  - `main.cpp` — startup + task launches + system init
  - `tasks/`
    - `control_task.cpp` — deterministic real-time loop + deadline tracking
    - `sensor_task.cpp` — produces IMU samples and pushes to queue
    - `telemetry_task.cpp` — prints timing stats + runtime state
    - `load_task.cpp` — CPU stress generator
  - `drivers/`
    - `pwm_driver.*` — PWM driver using ESP32 LEDC peripheral
    - `imu_driver.cpp` — IMU interface + stub backend + real backend (MPU-6050)

- `include/`
  - `app_config.h` — system config (rates, priorities, pin mappings, flags)
  - `shared_state.h` — shared variables + timing stats struct
  - driver/task headers

---

## Demo (in the works)

**In the works** (screenshots/video will be added once the PID + IMU integration is fully tested with hardware)

Planned demo(s):
- **Real-time timing proof**
  - Terminal output showing control loop period / jitter / missed deadlines
  - Run under different CPU load settings (ex: 0%, 30%, 80%)
- **Closed-loop control behavior (PID)**
  - Using IMU stub first: step response to a setpoint change (ex: target angle 0° → 10°)
  - Log `setpoint`, `measured pitch`, and `control output` over time
  - Plot or screenshot of the response (overshoot / settling / stability)
- **Actuator output via PWM**
  - PWM duty cycle driven by control output (LED brightness / scope/logic analyzer if available)
  - SAFE mode forces PWM duty to 0% (emergency stop behavior)

(Once the IMU arrives, I’ll repeat the same tests using the real MPU-6050 driver backend.)

---

## What I’ve implemented so far

- FreeRTOS-based task architecture (ESP32 Arduino framework)
- Deterministic control loop scheduling using `vTaskDelayUntil`
- Timing instrumentation:
  - min/max/avg dt
  - jitter tracking
  - missed deadline detection
- SAFE mode behavior when timing becomes unsafe
- CPU stress task for real-time robustness testing
- Driver work:
  - PWM driver using LEDC peripheral
  - IMU driver interface + stub backend
  - Real MPU-6050 backend written (I2C register reads/writes) pending hardware testing

---

## What I’m currently working on / next steps

- Integrate PID controller into the control task
  - Start with IMU stub input for fast iteration
  - Log PID terms + output for debugging
- Use PWM output as the “actuator”
  - Map control output → duty cycle
- When IMU arrives:
  - Validate I2C wiring + WHO_AM_I check
  - Compare real sensor data to stub expectations
  - Repeat the PID + timing experiments with real hardware
- Optional stretch goals:
  - CSV telemetry output for easy plotting
  - CLI commands over serial (set load %, setpoint, PID gains)
  - Complementary filter / sensor fusion basics

---

## Skills demonstrated (what I’m practicing intentionally)

- Real-time scheduling and determinism (FreeRTOS)
- Task design: priorities, pinning, non-blocking patterns
- IPC using queues (producer/consumer)
- Hardware abstraction: driver design + error codes
- I2C register-level device communication (MPU-6050)
- PWM peripheral configuration (LEDC)
- Control systems fundamentals (PID)
- Instrumentation and debugging (telemetry + timing stats)
- Safety patterns (SAFE mode / emergency stop)

---

## Build / Run

This project is built using PlatformIO in VS Code.

Typical workflow:
- Build
- Upload
- Monitor (Serial at 115200)

(I’ll add a more detailed “setup” section once the repo is public + cleaned up.)

---

## Notes

This project is intentionally structured like “real” firmware — small enough to finish, but designed to demonstrate embedded fundamentals that matter for robotics: drivers, control loops, RTOS timing, and safe task-to-task communication.
