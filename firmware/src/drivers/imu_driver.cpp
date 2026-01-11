#include "imu_driver.h"
#include "app_config.h"

#include <Arduino.h>
#include <math.h>


/* ================= IMU DRIVER ================ */

/**
 * Note that this file containts two "backends": 
 * 
 * If USE_IMU_STUB = 1 (imu hasn't arrived) then we generate realisticish (with the help of gen ai) accel/gyro data 
 * in software. We do this so we can test our architecture rn sensor task -> queue IPC -> control task -> PWM / telemetry 
 * 
 * If USE_IMU_STUB = 0 then we use the actual sensor values. 
 */

 #if USE_IMU_STUB 

 /* =========== STUB BACKEND ========= */

 /* Track initialization state */
 static bool ready = false; 

 /* Gravity magnitude (m/s^2) */
 static const float GRAVITY = 9.81f;

 /* Simulated tilt angle amplitude (degrees) */
 static const float AMPLITUDE = 20.0f;

 /* Simulated tilt freq */
 static const float FREQ = 0.20f; 

 imu_result_t imu_init(const imu_config_t* config){
    (void)config; /* stub doesn't use i2C settings */
    ready = true; 
    return IMU_OK; 
 }

 imu_result_t imu_read(imu_sample_t * out){
    if (!ready) return IMU_ERR_NOT_INITIALIZED;
    if (out == NULL) return IMU_ERR_INVALID_ARG;

    int64_t now = esp_timer_get_time(); 
    /* conver to seconds to use for sin and cos */
    float time = (float) now / 1000000.0f;

    /* omega = 2 * pi * freq (how fast the sine wave changes) */
    float omega = 2.0f * (float)M_PI * FREQ;

    /* pitch (degrees) is a sine wave pitch(t) = Amplitude * sin(omega * t) */
    float pitch = AMPLITUDE * sinf(omega * time); 

    /* Gyro is angular velocity (derivative of angle) */
    float dt_pitch = AMPLITUDE * omega * cosf(omega * time); 

    /* convert degrees to radians for sin/cos of angles */
    float theta = pitch * ((float)M_PI / 180.0f); 

    /* fill output struct */
    out->time_us = now; 
    out->acc_x = GRAVITY * sinf(theta);
    out->acc_y = 0.0f;
    out->acc_z = GRAVITY * cosf(theta);

    /* Simulated gyro: only pitch axis changes in our model */
    out->gyro_x = 0.0f;
    out->gyro_y = dt_pitch;  
    out->gyro_z = 0.0f;
    
    return IMU_OK;
 }

 const char* imu_error_to_string(imu_result_t r) {
    switch (r) {
        case IMU_OK: return "OK";
        case IMU_ERR_INVALID_ARG: return "Invalid argument";
        case IMU_ERR_NOT_INITIALIZED: return "Not initialized";
        case IMU_ERR_HW_NOT_FOUND: return "Hardware not found";
        case IMU_ERR_I2C: return "I2C error";
        case IMU_ERR_INTERNAL: return "Internal error";
        default: return "Unknown error";
  }
}

#else  /* USE_IMU_STUB == 0 */

/* ==================== REAL MPU-6050 BACKEND ================ */


#include <Wire.h> /* Arduino I2C Library */


/* 
 * MPU-6050 REGISTER MAP 
 *
 * Sensors like the MPU-6050 are controlled by writing "configuration registers"
 * and read by reading "data registers".
 *
 * Each register has an address (one byte).
 *
 * Example:
 *   Register 0x6B (PWR_MGMT_1) controls power state (sleep / wake).
 *   Register 0x75 (WHO_AM_I) tells you the sensor identity.
 *
 * Datasheet: shows full register map.
 */

 /* Identity register: lets us confirm the chip exists */
 static const uint8_t MPU6050_REG_WHO_AM_I = 0x75;

 /* Powere managemenet: chip start asleep we must wake it */
 static const uint8_t MPU6050_REG_PWR_MGMT_1 = 0x6B;

 /* Sample rate divider: controls internal sampling frequency */
 static const uint8_t MPU6050_REG_SMPLRT_DIV = 0x19;

/* Digital low-pass filter config (DLPF): smooth noise vs delay */
static const uint8_t MPU6050_REG_CONFIG = 0x1A;

/* Gyro range selection (+/- 250, 500, 1000, 2000 deg/s) */
static const uint8_t MPU6050_REG_GYRO_CONFIG = 0x1B;

/* Accel range selection (+/- 2g, 4g, 8g, 16g) */
static const uint8_t MPU6050_REG_ACCEL_CONFIG = 0x1C;

/**
 * Data burst start register:
 * If you start reading from ACCEL_XOUT_H, the MPU will let you read
 * multiple consecutive registers in one burst:
 *   accel xyz (6 bytes), temp (2 bytes), gyro xyz (6 bytes) = 14 bytes total
 *
 * That is the fastest / cleanest way to read the IMU.
 */
static const uint8_t MPU6050_REG_ACCEL_XOUT_H = 0x3B;

/* WHO_AM_I expected value is typically 0x68 for MPU-6050 */
static const uint8_t MPU6050_WHOAMI_EXPECTED = 0x68;

/* Driver internal state */
static bool ready = false; 
static uint8_t addr = MPU6050_I2C_ADDR; 


/* ================================== HELPER FUNCTIONS ============================ */

/**
 * Write one byte to a device register. I2C write transactions look like :
 * START
 * [device address + write bit] 
 * [register address]
 * pdata byte] 
 * STOP
 * 
 * Returns true if the device ACKed everything 
 */
static bool i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t val) {
    Wire.beginTransmission(addr); /* Start + address (write) */
    Wire.write(reg); /* register address */
    Wire.write(val); /* data to wrtie) */
    return (Wire.endTransmission() == 0); /* STOP */
}

/**
 * Read one byte from a device register
 * 
 * A register read goes as follows; first we write the register address we want (no stop, repeated start)
 * and then we read the data from that register. 
 */

 static bool i2c_read_reg(uint8_t addr, uint8_t reg, uint8_t* out) {
    if (!out) return false; 

    Wire.beginTransmission(addr); /* START + address (write) */
    Wire.write(reg); /* tell chip which register we want */
    if (Wire.endTransmission(false) != 0) return false; /* repeated start */

    /* req 1 byte from the device */
    if (Wire.requestFrom((int)addr, 1) != 1) return false; 

    *out = (uint8_t)Wire.read(); 
    return true;
 }


/* Burst read multiple consecutive registers. This is important as its faster than just 
   reading a single register and reduces I2C overhead */
static bool i2c_read_bytes(uint8_t addr, uint8_t start_reg, uint8_t* buf, size_t len) {
    if (!buf || len == 0) return false; 
    
    Wire.beginTransmission(addr);
    Wire.write(start_reg);
    if (Wire.endTransmission(false) != 0) return false;

    int got = Wire.requestFrom((int)addr, (int)len);
    if (got != (int)len) return false;

    for (size_t i = 0; i < len; i++) {
        buf[i] = (uint8_t)Wire.read();
    }
    return true;
}


/**
 * convert two bytes (big endian) into signed 16 bit integer. MPU stores HIGH byte then LOW byte
 */
static inline int16_t be16(uint8_t hi, uint8_t lo) {
  return (int16_t)((hi << 8) | lo);
}

/* 
 * Scaling constants (raw counts -> physical ish units)
 *
 * The sensor gives "raw counts".
 * To interpret them, we convert counts into physical units.
 *
 * For our config (range settings):
 *   accel range +/- 2g -> 16384 LSB per 1g
 *   gyro range  +/- 250 deg/s -> 131 LSB per 1 deg/s
 *
 * If you change the range registers later, these scale factors must change too.
 */
static const float ACC_LSB_PER_G = 16384.0f;    /* +/-2g */
static const float GYRO_LSB_PER_DPS = 131.0f;   /* +/-250 deg/s */
static const float G_MPS2 = 9.81f;              /* gravity magnitude */

/* 
 * imu_init(): one-time hardware setup
 *
 * This is called once at boot (in setup()).
 *
 * What it does:
 *   1) Start I2C (Wire.begin)
 *   2) Confirm the IMU exists (WHO_AM_I)
 *   3) Wake up the chip (PWR_MGMT_1)
 *   4) Configure rates/filters/ranges
 */
imu_result_t imu_init(const imu_config_t* config) {
  if (!config) return IMU_ERR_INVALID_ARG;

  /* Store address so reads/writes know which device to talk to */
  g_addr = config->i2c_addr;

  /**
   * Start I2C.
   * If you wired default pins, this is enough.
   * If you used different SDA/SCL pins, use:
   *   Wire.begin(SDA_PIN, SCL_PIN);
   */
  Wire.begin();

  /* Set I2C bus speed (400kHz is common "fast mode") */
  Wire.setClock(config->i2c_clock_hz);

  /* ---------------- 1) Verify chip identity ---------------- */
  uint8_t who = 0;

  /* If we cannot even read WHO_AM_I, the bus is probably wrong or device missing */
  if (!i2c_read_reg(g_addr, MPU6050_REG_WHO_AM_I, &who)) return IMU_ERR_I2C;

  /**
   * If WHO_AM_I doesn't match, likely causes:
   * - wrong I2C address (0x68 vs 0x69)
   * - wiring wrong (SDA/SCL swapped, no pullups)
   * - not actually an MPU-6050
   */
  if (who != MPU6050_WHOAMI_EXPECTED) return IMU_ERR_HW_NOT_FOUND;

  /* ---------------- 2) Wake chip up ----------------
   * MPU-6050 starts in sleep mode.
   * Writing 0x00 to PWR_MGMT_1 clears sleep bit.
   */
  if (!i2c_write_reg(g_addr, MPU6050_REG_PWR_MGMT_1, 0x00)) return IMU_ERR_I2C;

  /* ---------------- 3) Basic configuration ----------------
   * These are "reasonable defaults" for bring-up.
   * You can tune them later.
   */

  /* Set sample rate divider */
  if (!i2c_write_reg(g_addr, MPU6050_REG_SMPLRT_DIV, 0x04)) return IMU_ERR_I2C;

  /* Configure digital low-pass filter (reduces noise) */
  if (!i2c_write_reg(g_addr, MPU6050_REG_CONFIG, 0x03)) return IMU_ERR_I2C;

  /* Set gyro range: 0x00 => +/- 250 deg/s */
  if (!i2c_write_reg(g_addr, MPU6050_REG_GYRO_CONFIG, 0x00)) return IMU_ERR_I2C;

  /* Set accel range: 0x00 => +/- 2g */
  if (!i2c_write_reg(g_addr, MPU6050_REG_ACCEL_CONFIG, 0x00)) return IMU_ERR_I2C;

  /* Mark ready so imu_read() is allowed */
  g_ready = true;
  return IMU_OK;
}

/*
 * imu_read(): fast data read (called frequently)
 *
 * Called by sensor_task at a fixed rate (ex: 200 Hz).
 *
 * What it does:
 *   1) Burst read 14 bytes starting at ACCEL_XOUT_H
 *   2) Convert bytes into signed int16 raw values
 *   3) Scale to real-ish units
 *   4) Return imu_sample_t
 */
imu_result_t imu_read(imu_sample_t* out) {
  if (!g_ready) return IMU_ERR_NOT_INITIALIZED;
  if (!out) return IMU_ERR_INVALID_ARG;

  /**
   * Layout of the 14 bytes read from 0x3B:
   *   [0..5]   accel X,Y,Z (each 2 bytes)
   *   [6..7]   temperature (2 bytes)  (unused for now)
   *   [8..13]  gyro X,Y,Z (each 2 bytes)
   */
  uint8_t buf[14];

  /* If this fails, it's usually wiring, address, or bus error */
  if (!i2c_read_bytes(g_addr, MPU6050_REG_ACCEL_XOUT_H, buf, sizeof(buf))) {
    return IMU_ERR_I2C;
  }

  /* Convert bytes -> raw signed 16-bit integers */
  int16_t ax_raw = be16(buf[0],  buf[1]);
  int16_t ay_raw = be16(buf[2],  buf[3]);
  int16_t az_raw = be16(buf[4],  buf[5]);

  /* temp is buf[6], buf[7] if you want it later */

  int16_t gx_raw = be16(buf[8],  buf[9]);
  int16_t gy_raw = be16(buf[10], buf[11]);
  int16_t gz_raw = be16(buf[12], buf[13]);

  /* Timestamp when sample was read */
  out->time_us = esp_timer_get_time();

  /* ---------------- Convert accel raw -> m/s^2 ----------------
   * Step 1: raw / ACC_LSB_PER_G gives "g" units
   * Step 2: multiply by 9.81 to convert g -> m/s^2
   */
  out->acc_x = ((float)ax_raw / ACC_LSB_PER_G) * G_MPS2;
  out->acc_y = ((float)ay_raw / ACC_LSB_PER_G) * G_MPS2;
  out->acc_z = ((float)az_raw / ACC_LSB_PER_G) * G_MPS2;

  /* ---------------- Convert gyro raw -> deg/s ----------------
   * raw / 131 converts to deg/s for +/-250 range
   */
  out->gyro_x = (float)gx_raw / GYRO_LSB_PER_DPS;
  out->gyro_y = (float)gy_raw / GYRO_LSB_PER_DPS;
  out->gyro_z = (float)gz_raw / GYRO_LSB_PER_DPS;

  return IMU_OK;
}

/* 
 * Error-to-string helper (for debugging)
 *
 */
const char* imu_error_to_string(imu_result_t r) {
  switch (r) {
    case IMU_OK: return "OK";
    case IMU_ERR_INVALID_ARG: return "Invalid argument";
    case IMU_ERR_NOT_INITIALIZED: return "Not initialized";
    case IMU_ERR_HW_NOT_FOUND: return "WHO_AM_I mismatch / IMU not found";
    case IMU_ERR_I2C: return "I2C communication error";
    case IMU_ERR_INTERNAL: return "Internal error";
    default: return "Unknown error";
  }
}

#endif  // USE_IMU_STUB