#ifndef L298N_H
#define	L298N_H

#include <stdint.h>
#include <stdbool.h>

void L298N_init(void);
void motor_A_start(float duty_percent, bool forward);
void motor_B_start(float duty_percent, bool forward);
void motor_A_stop(void);
void motor_B_stop(void);


#endif //L298N_H