#include "encoder.h" 

/**
  * @简  述  编码器初始化
  * @参  数  无
  * @返回值  无
  */
void ENCODER_A_Init(void)
{ 
	GPIO_InitTypeDef GPIO_InitStructure; 
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;   

	//GPIO功能时钟使能
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	//配置为TIM2_CH1和TIM2_CH2功能编码器功能
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);

	//配置IO口为复用功能-定时器通道
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_1|GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//Timer configuration in Encoder mode 
	TIM_DeInit(TIM2);
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

	TIM_TimeBaseStructure.TIM_Prescaler = 0x0;  // No prescaling 
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;  //65536
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;   
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

  // TIM_EncoderMode_TI12: 在TI1和TI2的上升沿和下降沿都计数 (四倍频模式)
  // TIM_ICPolarity_Rising: 极性设置为上升沿，硬件会自动处理双边沿检测
	TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = 6;
	TIM_ICInit(TIM2, &TIM_ICInitStructure);

	//Reset counter
	TIM_ClearFlag(TIM2,TIM_FLAG_Update);
	ENCODER_A_SetCounter(0);

	TIM_Cmd(TIM2, ENABLE);    
}

/**
  * @简  述  编码器获取计数器数值
  * @参  数  无
  * @返回值  计数器当前值
  */
uint16_t ENCODER_A_GetCounter(void)
{
	return TIM2->CNT;
}

/**
  * @简  述  编码器设置计数器数值
  * @参  数  count  计数器数值
  * @返回值  无
  */
void ENCODER_A_SetCounter(uint16_t count)
{
	TIM2->CNT = count;
}

/**
  * @简  述  编码器初始化
  * @参  数  无
  * @返回值  无
  */
void ENCODER_B_Init(void)
{ 
	GPIO_InitTypeDef GPIO_InitStructure; 
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;   

	//GPIO功能时钟使能
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC| RCC_APB2Periph_AFIO, ENABLE);
	//开启TIM3全映射
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	//配置IO口为复用功能-定时器通道
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	//Timer configuration in Encoder mode
	TIM_DeInit(TIM3);
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

	TIM_TimeBaseStructure.TIM_Prescaler = 0x0;  // No prescaling 
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;  
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;   
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = 6;
	TIM_ICInit(TIM3, &TIM_ICInitStructure);

		//Reset counter
	TIM_ClearFlag(TIM3,TIM_FLAG_Update);
	ENCODER_B_SetCounter(0);

	TIM_Cmd(TIM3, ENABLE); 
}

/**
  * @简  述  编码器获取计数器数值
  * @参  数  无
  * @返回值  计数器当前值
  */
uint16_t ENCODER_B_GetCounter(void)
{
	return TIM3->CNT;
}

/**
  * @简  述  编码器设置计数器数值
  * @参  数  count  计数器数值
  * @返回值  无
  */
void ENCODER_B_SetCounter(uint16_t count)
{
	TIM3->CNT = count;
}

/******************* (C) 版权 2023 XTARK **************************************/
