#include "motor.h"

#define MOTOR_AIN1_PIN    GPIO_Pin_0
#define MOTOR_AIN2_PIN    GPIO_Pin_1
#define MOTOR_BIN1_PIN    GPIO_Pin_8
#define MOTOR_BIN2_PIN    GPIO_Pin_9
#define MOTOR_STBY_PIN    GPIO_Pin_10

#define MOTOR_AIN1_PORT   GPIOC
#define MOTOR_AIN2_PORT   GPIOC
#define MOTOR_BIN1_PORT   GPIOC
#define MOTOR_BIN2_PORT   GPIOC
#define MOTOR_STBY_PORT   GPIOC

static void MOTOR_A_SetDirection(int16_t speed)
{
	if (speed > 0)
	{
		GPIO_SetBits(MOTOR_AIN1_PORT, MOTOR_AIN1_PIN);
		GPIO_ResetBits(MOTOR_AIN2_PORT, MOTOR_AIN2_PIN);
	}
	else if (speed < 0)
	{
		GPIO_ResetBits(MOTOR_AIN1_PORT, MOTOR_AIN1_PIN);
		GPIO_SetBits(MOTOR_AIN2_PORT, MOTOR_AIN2_PIN);
	}
	else
	{
		GPIO_ResetBits(MOTOR_AIN1_PORT, MOTOR_AIN1_PIN);
		GPIO_ResetBits(MOTOR_AIN2_PORT, MOTOR_AIN2_PIN);
	}
}

static void MOTOR_B_SetDirection( int16_t speed)
{
	if (speed > 0)
	{
		GPIO_ResetBits(MOTOR_BIN1_PORT, MOTOR_BIN1_PIN);
		GPIO_SetBits(MOTOR_BIN2_PORT, MOTOR_BIN2_PIN);
	}
	else if (speed < 0)
	{
		GPIO_SetBits(MOTOR_BIN1_PORT, MOTOR_BIN1_PIN);
		GPIO_ResetBits(MOTOR_BIN2_PORT, MOTOR_BIN2_PIN);
	}
	else
	{
		GPIO_ResetBits(MOTOR_BIN1_PORT, MOTOR_BIN1_PIN);
		GPIO_ResetBits(MOTOR_BIN2_PORT, MOTOR_BIN2_PIN);
	}
}

/**
  * @简  述  电机PWM控制初始化，PWM频率为20KHZ	
  * @参  数  无
  * @返回值  无
  */
void MOTOR_Init(void)
{ 
	
	GPIO_InitTypeDef GPIO_InitStructure; 
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure; 

	//GPIO功能时钟使能
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);
	//TIM时钟使能
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	//配置IO口为复用功能-定时器通道
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = MOTOR_AIN1_PIN | MOTOR_AIN2_PIN |
		MOTOR_BIN1_PIN | MOTOR_BIN2_PIN | MOTOR_STBY_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	MOTOR_A_SetDirection(0);
	MOTOR_B_SetDirection(0);
	GPIO_SetBits(MOTOR_STBY_PORT, MOTOR_STBY_PIN);

	//Time base configuration
	TIM_TimeBaseStructure.TIM_Period = 3600-1;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	//PWM1 Mode configuration: Channel1 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;	    //占空比初始化
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	//PWM1 Mode configuration: Channel3
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
	
	//PWM1 Mode configuration: Channel4
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM4, ENABLE);

	//TIM enable counter
	TIM_Cmd(TIM4, ENABLE);   
	
	//使能MOE位
	TIM_CtrlPWMOutputs(TIM4,ENABLE);	


}

/**
  * @简  述 电机PWM速度控制
  * @参  数 speed 电机转速数值，范围-3600~3600
  * @返回值 无
  */
void MOTOR_A_SetSpeed(int16_t speed)
{
	int16_t temp;
	
	temp = speed;
	
	if(temp>3600){
		temp = 3600;
	}else if(temp<-3600){
		temp = -3600;
	}else{
		temp = 0;
	}
	//设置方向
	MOTOR_A_SetDirection(temp);
	if (temp < 0)
		temp = -temp;
	//设置速度
	TIM_SetCompare3(TIM4, (uint16_t)temp);
	//if (temp > 0){
	//	TIM_SetCompare3(TIM4,(3600 - temp));
	//}else{
	//	TIM_SetCompare3(TIM4,(3600 + temp));
	//}
}

/**
  * @简  述 电机PWM速度控制
  * @参  数 speed 电机转速数值，范围-3600~3600
  * @返回值 无
  */
void MOTOR_B_SetSpeed(int16_t speed)
{
	int16_t temp;
	
	temp = speed;
	
	if(temp>3600){
		temp = 3600;
	}else if(temp<-3600){
		temp = -3600;
	}else{
		temp = 0;
	}
	//设置方向
	MOTOR_B_SetDirection(temp);
	if (temp < 0)
		temp = -temp;
	//设置速度
	TIM_SetCompare4(TIM4, (uint16_t)temp);
	//if (temp > 0){
	//	TIM_SetCompare4(TIM4,(3600 - temp));
	//}else{
	//	TIM_SetCompare4(TIM4,(3600 + temp));
	//}
}


/******************* (C) 版权 2023 XTARK **************************************/
