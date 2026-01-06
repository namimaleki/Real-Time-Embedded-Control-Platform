
#include "pwm_driver.h"
#include <Arduino.h>

/**
 * ======================== PWM DRIVER IMPLEMENTATION (ARDUINO ESP32 LEDC)================ 
 * This driver provides a clena abstractino over the ESP32's LEDC peripheral for PWM generation. it uses 
 * the ARduino ESP32 LEDC wrapper functions. 
 */

/* =========== DRIVER STATE (PRIVATE) ========== */
/**
 * Channel Tracking Arrays
 * 
 * We maintain state for each LEDC channel to:
 * 1. Prevent using channels before initialization
 * 2. Cache duty cycle for fast telemetry reads
 * 3. Store resolution bits for duty conversion
 * 
 * Why per-channel tracking?
 * - Different channels may have different resolutions
 * - Channels can be initialized independently
 * - Allows validation before every operation
 */
#ifndef LEDC_CHANNEL_MAX
/* On most arduino esp32, channels are 0..15 so if this macro doesn't exist in our environment we define it just in case */ 
#define LEDC_CHANNEL_MAX 16
#endif

/* Prevent using driver before set up */
static bool driver_initialized = false;
static bool channel_initialized[LEDC_CHANNEL_MAX] = {false};

/* Cache the duty so telemetry can read it in an efficient manner. */
static float channel_duty_perc[LEDC_CHANNEL_MAX] = {0.0f};

/* Cache resolution bits per channel so we can compute duty integer correctly */
static int channel_res_bits[LEDC_CHANNEL_MAX] = {0};

/* ============== HELPER FUNCTIONS (PRIVATE) ============== */
static bool is_valid_channel(int ch) {
    return (ch >= 0 && ch < LEDC_CHANNEL_MAX);
}

static bool is_valid_duty(float duty_percent) {
    return (duty_percent >= 0.0f && duty_percent <= 100.0f);
}

static bool is_valid_freq(int freq){
    return (freq > 0); 
}

static bool is_valid_resolution(int res_bits){
    return (res_bits >= 1 && res_bits <= 16);
}

/**
 * Convert duty percentage to hardware integer value
 * 
 * Hardware expects duty as integer count, not percentage.
 * Conversion depends on resolution.
 * 
 * Formula:
 *   duty_int = (duty_percent / 100.0) * (2^res_bits - 1)
 */
 static uint32_t duty_percent_to_int(float duty_percent, int res_bits){
    /* We will bring the value to valid range if not valid */
    if (duty_percent < 0.0f) duty_percent = 0.0f;
    if (duty_percent > 100.0f) duty_percent = 100.0f;

    /* Caclualte maxium duty value for this resulotion */
    uint32_t max_duty = (1u << res_bits) - 1u; 
    float duty_0_to_1 = duty_percent / 100.0f;
    return (uint32_t)(duty_0_to_1 * (float)max_duty);
}


/* ======================== PUBLIC API ===================== */

/**
 * Function to initialize PWM on specified channel and GPIO pin
 * 
 * This configures the ESP32 LEDC hardware for PWM generation.
 * 
 * Steps performed:
 * 1. Validate all configuration parameters
 * 2. Configure LEDC timer and channel (via ledcSetup)
 * 3. Attach channel to GPIO pin (via ledcAttachPin)
 * 4. Set initial duty cycle (via ledcWrite)
 * 5. Update driver state tracking
 * 
 * After this call, the GPIO pin will output PWM at the specified frequency.
 * input: config Pointer to configuration structure
 * will return PWM_OK on success, error code on failure
 */
pwm_result_t pwm_init(const pwm_config_t* config){

    /* Check to make sure everything is valid */
    if (config == NULL) return PWM_ERR_INVALID_ARG;
    if (!is_valid_channel(config->channel)) return PWM_ERR_INVALID_CHANNEL; 
    if (config->gpio_pin < 0) return PWM_ERR_INVALID_PIN;
    if (!is_valid_freq(config->frequency)) return PWM_ERR_INVALID_FREQ;
    if (!is_valid_resolution(config->resolution_bits)) return PWM_ERR_INVALID_RES;
    if (!is_valid_duty(config->initial_duty)) return PWM_ERR_INVALID_DUTY;


    /**
     * To set up the hardware we use ledcSetup(channel, freq, resolution bits) which will
     * configure a PWM generator for the specifed channel and return the actual frequency 
     * achieved (0 on failure). This could fail due to invalid freq/resolution combination, 
     * or internal hardware config issues. 
     */
    double result = ledcSetup(config->channel, config->frequency, config->resolution_bits);
    if (result <= 0) return PWM_ERR_HARDWARE; 

    /* Route the channel output to the GPIO pin by using ledcAttatchPin(gpio, chanel) after this the pin will output PWM */
    ledcAttachPin(config->gpio_pin, config->channel);

    /**
     * Set Initial Duty Cycle
     * Convert duty percentage to integer count and write to hardware.
     * ledcWrite(channel, duty_integer) updates the hardware immediately.
     */
    uint32_t duty_int = duty_percent_to_int(config->initial_duty, config->resolution_bits);
    ledcWrite(config->channel, duty_int); 

    /* UPDATE THE DRIVER STATE */
    channel_initialized[config->channel] = true;
    channel_duty_perc[config->channel] = config->initial_duty;
    channel_res_bits[config->channel] = config->resolution_bits;

    driver_initialized = true;
    return PWM_OK;
}


/**
 * Set PWM duty cycle
 * Changes the pulse width of the PWM signal.
 * Takes effect immediately - no glitches or delays.
 */
pwm_result_t pwm_set_duty(int channel, float duty_percent) {
    if(!driver_initialized) return PWM_ERR_NOT_INITIALIZED;
    if (!is_valid_channel(channel)) return PWM_ERR_INVALID_CHANNEL;
    if (!channel_initialized[channel]) return PWM_ERR_NOT_INITIALIZED;
    if (!is_valid_duty(duty_percent)) return PWM_ERR_INVALID_DUTY;

    /* Conver percent to integer count expected by ledcWrite. ledcWrite(channel, duty_int) will 
       update the hardware duty cycle register. */
    int res_bits = channel_res_bits[channel];
    uint32_t duty_int = duty_percent_to_int(duty_percent, res_bits); 
    ledcWrite(channel, duty_int);

    /* Cache for telemetry */
    channel_duty_perc[channel] = duty_percent; 
    return PWM_OK; 
}

/* Returns the current duty cycle. */
pwm_result_t pwm_get_duty(int channel, float* duty_percent_out){
    if (!driver_initialized) {
        return PWM_ERR_NOT_INITIALIZED;
    }
    if (!is_valid_channel(channel)) {
        return PWM_ERR_INVALID_CHANNEL;
    }
    if (!channel_initialized[channel]) {
        return PWM_ERR_NOT_INITIALIZED;
    }
    if (duty_percent_out == NULL) {
        return PWM_ERR_INVALID_ARG;
    }

    *duty_percent_out = channel_duty_perc[channel];
    return PWM_OK;
}

/* Emergency stop single channel used in fault conditions */
pwm_result_t pwm_emergency_stop(int channel) {
    /* force output to 0% immediately. IN a real robot safe mode would call this for certain mortor channels and then require operator command to resume */
    return pwm_set_duty(channel, 0.0f); 
}

pwm_result_t pwm_emergency_stop_all(void){
    if (!driver_initialized) {
        return PWM_ERR_NOT_INITIALIZED;
    }

    for (int ch = 0; ch < LEDC_CHANNEL_MAX; ch++) {
        if (channel_initialized[ch]) {
            // Ignore return value here; we’re best-effort stopping everything.
            pwm_set_duty(ch, 0.0f);
        }
    }

    return PWM_OK;
}

const char* pwm_error_to_string(pwm_result_t result) {
    switch (result) {
        case PWM_OK:               return "Success";
        case PWM_ERR_INVALID_ARG:  return "Invalid argument (null pointer or bad input)";
        case PWM_ERR_INVALID_PIN:  return "Invalid GPIO pin";
        case PWM_ERR_INVALID_CHANNEL: return "Invalid PWM channel";
        case PWM_ERR_INVALID_DUTY: return "Invalid duty (must be 0..100%)";
        case PWM_ERR_INVALID_FREQ: return "Invalid frequency (must be > 0)";
        case PWM_ERR_INVALID_RES:  return "Invalid resolution bits (expected 1..16)";
        case PWM_ERR_HARDWARE:     return "Hardware PWM setup failed";
        case PWM_ERR_NOT_INITIALIZED: return "Driver/channel not initialized";
        default:                   return "Unknown error";
    }
}