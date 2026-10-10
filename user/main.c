	
#include "stm32f10x.h"
#include "delay.h"
#include "led.h"  
#include "key.h"
#include "usart.h"
#include "mpu6050.h"
#include "mpu6050_dmp.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "balance.h"
#include "lcd_model.h"
#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"
#include "robot.h"
#include "encoder.h"
#include "motor.h"
#include "task.h"

#define START_TASK_PRIO		1
#define START_STK_SIZE 		256  
TaskHandle_t StartTask_Handler;
void start_task(void *pvParameters);

//LCD任务
#define LCD_TASK_PRIO		2
#define LCD_STK_SIZE 		256  
TaskHandle_t LCDTask_Handler;
void lcd_task(void *pvParameters);

//平衡车控制任务
#define BALANCE_TASK_PRIO		4
#define BALANCE_STK_SIZE 		512
TaskHandle_t ControlTask_Handler;
void balance_task(void *pvParameters);
bool pattern = true;

//控制和功能任务
#define CONTROL_TASK_PRIO		3     
#define CONTROL_STK_SIZE 		256 
TaskHandle_t Control_Task_Handle = NULL;
void Control_Task(void *pvParameters);


/***************************************************************************************************
* 函数名：main
* 入口参数：无
* 返回值：无
* 函数功能：初始化系统并启动任务调度器
* 说明：无
****************************************************************************************************/

int main(void)
{
	AX_DELAY_Init();		// 延时初始化
	LED_Init();			// LED 初始化
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);// 配置系统中断优先级分组
	KEY_Init();
	SPI_LCD_Init();			// LCD 初始化
	
	LED1_ON;
	LED2_OFF;
	LCD_Clear(); 											
	// MPU6050 初始化
	AX_MPU6050_Init();  // MPU6050 初始化
	AX_MPU6050_DMP_Init();  // DMP 初始化
	// 串口初始化
	Usart1_Init(9600);
	USART1_SendString("AT+BAUD=3\r\n");
	USART1_SendString("AT+MMTU=20\r\n");
	USART1_SendString("AT+RESET\r\n");
	// 编码器初始化
	ENCODER_A_Init();	
  ENCODER_B_Init();
	// 电机初始化
	MOTOR_Init();
	
	// 创建开始任务
	xTaskCreate((TaskFunction_t )start_task,            // 任务函数
								(const char*    )"start_task",          // 任务名称
								(uint16_t       )START_STK_SIZE,        // 任务堆栈大小
								(void*          )NULL,                  // 任务参数
								(UBaseType_t    )START_TASK_PRIO,       // 任务优先级
								(TaskHandle_t*  )&StartTask_Handler);   // 任务句柄
	// 启动调度器
  vTaskStartScheduler();   
   while (1);
}

// 应用任务创建函数
void start_task(void *pvParameters)
{
	
	// 进入临界区
	taskENTER_CRITICAL();       

	//创建机器人控制任务
	xTaskCreate((TaskFunction_t )Control_Task, /* 任务入口函数 */
			 (const char*    )"Control_Task",/* 任务名字 */
			 (uint16_t       )CONTROL_STK_SIZE,   /* 任务栈大小 */
			 (void*          )NULL,	/* 任务入口函数参数 */
			 (UBaseType_t    )CONTROL_TASK_PRIO,	    /* 任务的优先级 */
			 (TaskHandle_t*  )&Control_Task_Handle);/* 任务控制块指针 */
								
	// 创建 LCD 任务
	xTaskCreate((TaskFunction_t )lcd_task,            // 任务函数
	              (const char*    )"lcd_task",          // 任务名称
	              (uint16_t       )LCD_STK_SIZE,        // 任务堆栈大小
	              (void*          )NULL,                  // 任务参数
	              (UBaseType_t    )LCD_TASK_PRIO,       // 任务优先级
	              (TaskHandle_t*  )&LCDTask_Handler);   // 任务句柄
	
	//创建平衡车控制任务
	xTaskCreate((TaskFunction_t )balance_task,
				(const char*    )"balance_task",
				(uint16_t       )BALANCE_STK_SIZE,
				(void*          )NULL,
				(UBaseType_t    )BALANCE_TASK_PRIO,
				(TaskHandle_t*  )&ControlTask_Handler);
								
  vTaskDelete(StartTask_Handler); 							
	// 退出临界区
	taskEXIT_CRITICAL();  	
	
} 





