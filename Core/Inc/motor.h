#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

void Motor_Init(void);                      
void Motor_Set(int8_t left, int8_t right);   
void Motor_Stop(void);
void esc_calibrate(void);

#endif