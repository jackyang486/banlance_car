#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f10x.h"
void ENCODER_A_Init(void);
void ENCODER_B_Init(void);
uint16_t ENCODER_A_GetCounter(void);
void ENCODER_A_SetCounter(uint16_t count);
uint16_t ENCODER_B_GetCounter(void);
void ENCODER_B_SetCounter(uint16_t count);

#endif
