#ifndef __MOTOR_H
#define __MOTOR_H
#include "stm32f10x.h"

void MOTOR_Init(void);
void MOTOR_A_SetSpeed(int16_t speed);
void MOTOR_B_SetSpeed(int16_t speed);
#endif
