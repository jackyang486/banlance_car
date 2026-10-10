/**			                                                    
		   ____                    _____ _______ _____       XTARK@塔克创新
		  / __ \                  / ____|__   __|  __ \ 
		 | |  | |_ __   ___ _ __ | |       | |  | |__) |
		 | |  | | '_ \ / _ \ '_ \| |       | |  |  _  / 
		 | |__| | |_) |  __/ | | | |____   | |  | | \ \ 
		  \____/| .__/ \___|_| |_|\_____|  |_|  |_|  \_\
				| |                                     
				|_|                OpenCTR   机器人控制器
									 
  ****************************************************************************** 
  *           
  * 版权所有： XTARK@塔克创新  版权所有，盗版必究
  * 公司网站： www.xtark.cn   www.tarkbot.com
  * 淘宝店铺： https://xtark.taobao.com  
  * 塔克微信： 塔克创新（关注公众号，获取最新更新资讯）
  *      
  ******************************************************************************
  * @作  者  Musk Han@XTARK
  * @内  容  机器人任务处理文件
  * 
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "tasks.h"
#include "robot.h"
#include "lcd_spi_169.h"
#include "light.h"
#include "control.h"
#include "balance.h"
#include "rgb.h"
#include "servo.h"
//#include "ax_function.h"

//函数定义
void JOINT_Control(void);       //舵机云台处理

/**
  * @简  述  机器人控制和功能管理任务
  * @参  数  无
  * @返回值  无
  */
void Control_Task(void* parameter)
{	
	while(1)
	{		
		//控制模式，控制频率约50HZ
		if(ax_control_mode < CTL_FN1)
		{
			//控制方式选择，ROS模式不执行任何操作
			//if(ax_control_mode == CTL_PS2)          AX_CTL_Ps2();    //PS2手柄			
			//else if (ax_control_mode == CTL_APP)    AX_CTL_App();    //APP控制		
			//else if (ax_control_mode == CTL_RMS)    AX_CTL_Rms();    //航模遥控器控制
			AX_CTL_App();
			
			//舵机云台控制
			JOINT_Control();
		
			//延时
			vTaskDelay(20); 
		}
		else  //功能模式
		{
			//判断是否开启功能
			if(ax_function_enable != 0)
			{
				//不同模式差异化显示
				switch(ax_control_mode)
				{
					case CTL_FN1://功能-雷达避障行驶
//						AX_FUN_List1();
						break;
					case CTL_FN2://功能-雷达警报
//						AX_FUN_List2();
						break;
					case CTL_FN3://功能-雷达跟随
//						AX_FUN_List3();
						break;			
					case CTL_FN4://功能-雷达走直线
//						AX_FUN_List4();
						break;				
					case CTL_FN5://功能-CCD巡线
//						AX_FUN_List5();
						break;
					default://空循环
						vTaskDelay(100);   
						break;
				}			
			}
			else
			{
				//延时
				vTaskDelay(100);   
			}			
		}                                
	}	
}

/**
  * @简  述  舵机云台控制函数
  * @参  数  无 
  * @返回值  无
  */
void JOINT_Control(void)   
{
	//幅度限制保护
	if(ax_joint_angle[0] > JOINTA_UP_LIMIT)	   ax_joint_angle[0] = JOINTA_UP_LIMIT;
	if(ax_joint_angle[0] < JOINTA_LOW_LIMIT)   ax_joint_angle[0] = JOINTA_LOW_LIMIT;
	
	if(ax_joint_angle[1] > JOINTB_UP_LIMIT)	   ax_joint_angle[1] = JOINTB_UP_LIMIT;
	if(ax_joint_angle[1] < JOINTB_LOW_LIMIT)   ax_joint_angle[1] = JOINTB_LOW_LIMIT;
	
	//设置舵机角度
	AX_SERVO_S1_SetAngle( ax_joint_angle[0] + JOINTA_ANGLE_OFFSET);
	AX_SERVO_S2_SetAngle( -ax_joint_angle[1] - JOINTB_ANGLE_OFFSET);	
	
	//查看舵机角度
	//printf("@%d %d \r\n ", ax_joint_angle[0], ax_joint_angle[1]);		
}




/**
  * @简  述  灯光处理任务
  * @参  数  无
  * @返回值  无
  */
void Light_Task(void* parameter)
{	

	while (1)
	{	
        //延时30ms，30HZ显示频率		
		vTaskDelay(30); 
		
		//判断前灯是否打开
		if(ax_light_enable != 0)
		{
			//正常显示
			AX_LIGHT_Show();
		}
		else
		{
			//关闭状态
			AX_RGB_SetFullColor(0x00, 0x00, 0x00);
		}
	}			
}



//lcd显示任务
void lcd_task(void *pvParameters){
	
	LCD_SetAsciiFont(&ASCII_Font20);
	LCD_SetColor(LIGHT_YELLOW);
	LCD_DisplayString(2,15,"B angle:");	 //平衡角度
	LCD_DisplayString(2,35,"B gyro:");   //平衡角速度
	LCD_DisplayString(2,55,"T angle:");  //转向角度
	LCD_DisplayString(2,75,"T gyro:");   //转向角速度
	
	LCD_SetColor(LIGHT_RED);
	LCD_DisplayString(2,95,"wheel L:");   //左轮速度
  LCD_DisplayString(2,115,"wheel R:");   //右轮速度

	LCD_SetColor(LIGHT_GREEN);
	LCD_DisplayString(2,135,"blc_kp:");   //角度KP
	LCD_DisplayString(120,135,"blc_kd:");   //角度KI
	
	LCD_SetColor(LIGHT_GREEN);
	LCD_DisplayString(2,155,"blv_kp:");   //速度KP
	LCD_DisplayString(120,155,"blv_ki:");   //速度KI
	
	LCD_SetColor(LIGHT_GREEN);
	LCD_DisplayString(2,175,"blt_kp:");   //转向KP
	LCD_DisplayString(120,175,"blt_kd:");   //转向KD
	
	
	while(1){
		LCD_SetColor(LIGHT_BLUE);
		//显示陀螺仪的角度和角速度
		LCD_DisplayFloat(85,15,mpu_angle.balanceAngle,"%.2f",6);
	  LCD_DisplayFloat(85,35,mpu_angle.balanceGyro,"%.2f",6);
		LCD_DisplayFloat(85,55,mpu_angle.turnAngle,"%.2f",6);
		LCD_DisplayFloat(85,75,mpu_angle.turnGyro,"%.2f",6);
		//显示轮子实时速度
		LCD_DisplayFloat(85, 95, (ax_wheel_vel_l*100), "%.2f",6);
		LCD_DisplayFloat(85, 115, (ax_wheel_vel_r*100), "%.2f",6);
		//显示角度控制PK和PD
		LCD_DisplayNum(75, 135, ax_balance_kp,4);
		LCD_DisplayNum(190, 135, ax_balance_kd,4);
		//显示速度控制PK和KI
		LCD_DisplayNum(75, 155, ax_velocity_kp,4);
		LCD_DisplayNum(190, 155, ax_velocity_ki,4);
		//显示转向控制PK和PD
		LCD_DisplayNum(75, 175, ax_turn_kp,4);
		LCD_DisplayNum(190, 175, ax_turn_kd,4);
		vTaskDelay(200);
	 }
}

//平衡控制任务
void balance_task(void *pvParameters)
{
	TickType_t last_wake_time = xTaskGetTickCount();
	while (1)
	{
		Balance_execute();
		vTaskDelayUntil(&last_wake_time, 10);
	}
}



/******************* (C) ��Ȩ 2023 XTARK **************************************/

