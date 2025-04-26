/**
 * @file L298N.h
 * @brief Motor driver control header for L298N dual H-bridge motor driver using ESP32 MCPWM module.
 *
 * Provides initialization and control functions for two DC motors (Motor A and Motor B)
 * using PWM for speed control and GPIOs for direction control.
 */

 #ifndef L298N_H
 #define L298N_H
 
 #include <stdint.h>
 #include <stdbool.h>
 
 /**
  * @brief Initialize the L298N motor driver GPIOs and PWM channels.
  */
 void L298N_init(void);
 
 /**
  * @brief Start Motor A with specified speed and direction.
  *
  * @param duty_percent Motor speed (0 to 100% duty cycle).
  * @param forward Direction control (true = forward, false = backward).
  */
 void motor_A_start(float duty_percent, bool forward);
 
 /**
  * @brief Start Motor B with specified speed and direction.
  *
  * @param duty_percent Motor speed (0 to 100% duty cycle).
  * @param forward Direction control (true = forward, false = backward).
  */
 void motor_B_start(float duty_percent, bool forward);
 
 /**
  * @brief Stop Motor A (sets PWM to 0 and disables direction pins).
  */
 void motor_A_stop(void);
 
 /**
  * @brief Stop Motor B (sets PWM to 0 and disables direction pins).
  */
 void motor_B_stop(void);
 
 #endif // L298N_H
 