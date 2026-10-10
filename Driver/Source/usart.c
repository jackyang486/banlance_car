/***
	************************************************************************************************
	*	@file					usart.c
	*	@version			V1.0
	*	@date					2023-11-28
	*	@author				Makerbase
	*	@brief				usart1相关函数
  ************************************************************************************************
  * @description
	*
	*	实验平台：DRG ST-4 STM32F103RTC6
	*	Q群：366182133，Q群答疑团队随时为您提供技术支持
	*	B站：https://space.bilibili.com/486637340
	*	官方网站：www.genbotter.com
	*	开发板购买：makerbase.taobao.com
	*	微信公众号：GenBotter
	************************************************************************************************
>>>>> 文件说明：
	*
	*  初始化USART1的引脚 PA9/PA10，
	*  配置USART1工作在收发模式、数位8位、停止位1位、无校验、不使用硬件控制流控制，
	*  重定义fputc函数,用以支持使用printf函数打印数据
	*
	************************************************************************************************
***/

#include "usart.h"
#include "string.h"
#include "stdlib.h"
#include "led.h"
#include "robot.h"
#include "lcd_spi_169.h"

static uint8_t uart1_rx_ok = 0;	 // 接收成功标志
static u8 UART_RX_BUF[40];		 // 接收缓冲，数据内容小于等于32Byte
static u8 UART_RX_CON = 0;		 // 接收计数器
static u8 UART_RX_CHECKSUM;		 // 帧头部分校验和

/***************************************************************************************************
 *	函 数 名: Usart_Init
 *	入口参数: u32 bound
 *	返 回 值: 无
 *	函数功能: usart1初始化
 *	说    明: bound -- 波特率
 ****************************************************************************************************/
void Usart1_Init(u32 bound)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE); // 使能USART1，GPIOA时钟

	// USART1_TX   GPIOA.9
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; // PA.9
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // 复用推挽输出
	GPIO_Init(GPIOA, &GPIO_InitStructure);			// 初始化GPIOA.9

	// USART1_RX	  GPIOA.10初始化
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;			  // PA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; // 浮空输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);				  // 初始化GPIOA.10

	// Usart1 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; // 抢占优先级2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		  // 子优先级1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  // IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);							  // 根据指定的参数初始化VIC寄存器

	// USART 初始化设置

	USART_InitStructure.USART_BaudRate = bound;										// 串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;						// 字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;							// 一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;								// 无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					// 收发模式

	USART_Init(USART1, &USART_InitStructure);	   // 初始化串口1
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); // 开启串口接受中断
	USART_Cmd(USART1, ENABLE);					   // 使能串口1
}


/***************************************************************************************************
 *	函 数 名: USART1_IRQHandler
 *	入口参数: 无
 *	返 回 值: 无
 *	函数功能: usart1接受中断服务
 *	说    明: 无
 ****************************************************************************************************/
void USART1_IRQHandler(void)
{
	uint8_t Res;

	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		Res = USART_ReceiveData(USART1);
		USART_SendData(USART1, Res);
		if (UART_RX_CON < 3)
		{
			//==接收帧头 + 长度
			if (UART_RX_CON == 0)
			{
				if (Res == 0xAA)
				{ // 接收帧头 1 0xAA
					UART_RX_BUF[0] = Res;
					UART_RX_CON = 1;
				}
			}
			else if (UART_RX_CON == 1)
			{ // 接收帧头 2 0x55
				if (Res == 0x55)
				{
					UART_RX_BUF[1] = Res;
					UART_RX_CON = 2;
				}
				else
				{
					UART_RX_CON = 0;
				}
			}
			else
			{ // 数据长度帧
				UART_RX_BUF[2] = Res;
				UART_RX_CON = 3;
				UART_RX_CHECKSUM = (0xFF + Res) & 0xff; // 计算校验和，0xFF为0xAA、0x55校验
			}
		}
		else
		{ //==数据帧
			if (UART_RX_CON < (UART_RX_BUF[2] - 1))
			{
				UART_RX_BUF[UART_RX_CON] = Res;
				UART_RX_CON++;
				UART_RX_CHECKSUM = (UART_RX_CHECKSUM + Res) & 0xff; // 计算校验和，排除最后一个字节的校验和
			}
			else
			{ // 判断校验和与传过来的时候是否相等
				if (Res == UART_RX_CHECKSUM)
				{ // 如果相等，则解析
					// 数据接收OK
					uart1_rx_ok = 1;
					// 手机APP完成蓝牙连接
					if (UART_RX_BUF[3] == ID_BLERX_CM)
					{
						if (UART_RX_BUF[4] == 0x55)
						{
							// 切换到手机APP控制
							ax_control_mode = CTL_APP;
							// 执行蜂鸣器鸣叫提示
							ax_beep_ring = BEEP_SHORT;
						}
					}
					// 小车参数调试
					if (UART_RX_BUF[3] < ID_BLERX_CM)
					{
						// 直立平衡PID参数
						if (UART_RX_BUF[3] == ID_URX_BLC)
						{
							ax_balance_kp = (int16_t)((UART_RX_BUF[4] << 8) | UART_RX_BUF[5]);
							ax_balance_kd = (int16_t)((UART_RX_BUF[6] << 8) | UART_RX_BUF[7]);
						}

						// 速度控制PID参数
						if (UART_RX_BUF[3] == ID_URX_BLV)
						{
							ax_velocity_kp = (int16_t)((UART_RX_BUF[4] << 8) | UART_RX_BUF[5]);
							ax_velocity_ki = (int16_t)((UART_RX_BUF[6] << 8) | UART_RX_BUF[7]);
						}

						// 转向控制PID参数
						if (UART_RX_BUF[3] == ID_URX_BLT)
						{
							ax_turn_kp = (int16_t)((UART_RX_BUF[4] << 8) | UART_RX_BUF[5]);
							ax_turn_kd = (int16_t)((UART_RX_BUF[6] << 8) | UART_RX_BUF[7]);
						}

						// 执行蜂鸣器鸣叫提示
						ax_beep_ring = BEEP_SHORT;
					}
				}
				else
				{
				}
				UART_RX_CON = 0;
			}
		}
		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	}
}

/**
  * @简  述  获取接收的数据
  * @参  数  *pbuf：接收数据指针,第1个字节为帧编码，后面为数据
  * @返回值	 0-无数据接收，other-需要读取的数据字节个数
  */
uint8_t AX_UART1_GetData(uint8_t *pbuf)
{
	uint8_t cnt,i;
	
	if(uart1_rx_ok != 0)
	{
		cnt = UART_RX_BUF[2]-4;
		
		for(i=0; i<cnt; i++)
		{
			*(pbuf+i) = UART_RX_BUF[3+i];
		}
	
		uart1_rx_ok = 0;
		return cnt;
	}
	else
	{
		return 0;
	}	
}

/*************************************************************************************************
 *	函 数 名:	fputc
 *	入口参数:	ch - 要输出的字符 ，  f - 文件指针（这里用不到）
 *	返 回 值:	正常时返回字符，出错时返回 EOF（-1）
 *	函数功能:	重定向 fputc 函数，目的是使用 printf 函数
 *	说    明:	无
 *************************************************************************************************/

int fputc(int c, FILE *fp)
{

	USART_SendData(USART1, (u8)c); // 发送单字节数据
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
		; // 等待发送完毕

	return (c); // 返回字符
}

// send string
void USART1_SendString(char *str)
{
	while (*str != '\0')
	{
		USART_SendData(USART1, *str);
		while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
			; // 等待发送完毕
		str++;
	}
}
