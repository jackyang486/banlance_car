#ifndef __USART_H
#define __USART_H

#include "stdio.h"
#include "stm32f10x.h"


/*------------------------------------------------ 函数声明 -------------------------------------------*/


void Usart1_Init(u32 bound);// usart1初始化函数
uint8_t AX_UART1_GetData(uint8_t *pbuf); //获取串口帧数据 
void USART1_SendString(char *str); //发送字符串

#endif //__USART_H

