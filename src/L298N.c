#include "L298N.h"

#include <stdio.h>
#include <stdbool.h>
#include <driver/gpio.h>
#include <driver/mcpwm.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Define GPIO pins for L298N control
#define IN1  GPIO_NUM_16  // Motor A direction control 1
#define IN2  GPIO_NUM_17  // Motor A direction control 2
#define IN3  GPIO_NUM_18  // Motor B direction control 1
#define IN4  GPIO_NUM_19  // Motor B direction control 2

#define PWM_A GPIO_NUM_4  // PWM signal for Motor A speed (ENA on L298N)
#define PWM_B GPIO_NUM_23 // PWM signal for Motor B speed (ENB on L298N)

void L298N_init(void){
    // Configure GPIOs as outputs
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << IN1) | (1ULL << IN2) | 
                        (1ULL << IN3) | (1ULL << IN4),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    // Initialize all GPIOs to LOW (motors off)
    gpio_set_level(IN1, 0);
    gpio_set_level(IN2, 0);
    gpio_set_level(IN3, 0);
    gpio_set_level(IN4, 0);

    // Configure MCPWM Timer
    mcpwm_config_t pwm_config = {
        .frequency = 10000,     // PWM frequency = 10 kHz
        .cmpr_a = 0.0f,           // Initial duty cycle for PWM A = 0%
        .cmpr_b = 0.0f,           // Initial duty cycle for PWM B = 0%
        .counter_mode = MCPWM_UP_COUNTER,
        .duty_mode = MCPWM_DUTY_MODE_0,  // Active-high duty
    };
    
    // Initialize MCPWM for Unit 0, Timer 0
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);

    // Assign GPIOs to PWM outputs
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, PWM_A);  
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, PWM_B); 
}

// Helper: Set motor A direction
static void motor_A_direction(bool forward) {
    gpio_set_level(IN1, forward ? 1 : 0);
    gpio_set_level(IN2, forward ? 0 : 1);
}

// Helper: Set motor B direction
static void motor_B_direction(bool forward) {
    gpio_set_level(IN3, forward ? 1 : 0);
    gpio_set_level(IN4, forward ? 0 : 1);
}

// Start motor A with speed (0 to 100%)
void motor_A_start(float duty_percent, bool forward) {
    motor_A_direction(forward);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty_percent);
}

// Start motor B with speed (0 to 100%)
void motor_B_start(float duty_percent, bool forward) {
    motor_B_direction(forward);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, duty_percent);
}

// Stop motor A
void motor_A_stop(void) {
    gpio_set_level(IN1, 0);
    gpio_set_level(IN2, 0);
    mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
}

// Stop motor B
void motor_B_stop(void) {
    gpio_set_level(IN3, 0);
    gpio_set_level(IN4, 0);
    mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B);
}