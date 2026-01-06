#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ================================ ERROR CODES ================================ */
typedef enum {
    PWM_OK = 0,
    PWM_ERR_INVALID_ARG,       /* null pointer / bad input */
    PWM_ERR_INVALID_PIN,       /* invalid GPIO pin */
    PWM_ERR_INVALID_CHANNEL,   /* LEDC channel out of range */
    PWM_ERR_INVALID_DUTY,      /* duty not in [0..100] */
    PWM_ERR_INVALID_FREQ,      /* frequency <= 0 */
    PWM_ERR_INVALID_RES,       /* resolution bits out of supported range */
    PWM_ERR_HARDWARE,          /* hardware setup failed */
    PWM_ERR_NOT_INITIALIZED    /* using driver before init */
} pwm_result_t;



/* ================================ CONFIGURATION ====================== */
typedef struct {
    int gpio_pin;
    int channel; 
    int frequency; 
    int resolution_bits; 
    float initial_duty;
} pwm_config_t;




/* Initialize PWM on a given pin + channel. This sets up the ESP32 LEDC hardware PWM and starts output at initial_duty% */
pwm_result_t pwm_init(const pwm_config_t* config); 

/* Set duty cycle in percent. INput is the LEDC channel, and the duty_percent */
pwm_result_t pwm_set_duty(int channel, float duty_percent); 

/* Retreive the last duty cycle we set (cached value). This will return what we think the duty is */
pwm_result_t pwm_get_duty(int channel, float* duty_percent_out);

/* Emergyncy stop for a single channel. This forces the output to 0% duty. We will use this in safe mode and fault handling */
pwm_result_t pwm_emergency_stop(int channel);

/* Stop all initialized channels. Used for system wide shutdown */
pwm_result_t pwm_emergency_stop_all(void); 


/* Convert error codes to readable strings (usefule for telemetry) */
const char* pwm_error_to_string(pwm_result_t result);

#ifdef _cplusplus
}
#endif