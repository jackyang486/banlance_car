/**			                                                    
		   ____                    _____ _______ _____       @塔克创新
		  / __ \                  / ____|__   __|  __ \ 
		 | |  | |_ __   ___ _ __ | |       | |  | |__) |
		 | |  | | '_ \ / _ \ '_ \| |       | |  |  _  / 
		 | |__| | |_) |  __/ | | | |____   | |  | | \ \ 
		  \____/| .__/ \___|_| |_|\_____|  |_|  |_|  \_\
				| |                                     
				|_|                OpenCTR   机器人控制器
									 
  ****************************************************************************** 
  *           
  * 版权所有： @塔克创新  版权所有，盗版必究
  * 公司网站： www.xtark.cn   www.tarkbot.com
  * 淘宝店铺： https://xtark.taobao.com  
  * 塔克微信： 塔克创新（关注公众号，获取最新更新资讯）
  *      
  ******************************************************************************
  * @作  者  Musk Han@XTARK
  * @内  容  机器人全局变量
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AX_ROBOT_H
#define __AX_ROBOT_H

/* Includes ------------------------------------------------------------------*/	 
#include "stm32f10x.h"

//C库函数的相关头文件
#include <stdio.h> 
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

//FreeRTOS头文件
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"
 
//外设相关头文件
#include "sys.h"      //系统设置
#include "delay.h"    //软件延时

#include "mpu6050.h"  //MPU6050操作函数头文件
#include "mpu6050_dmp.h" //MPU6050 DMP功能函数

//RGB灯效数据
typedef struct  
{
	uint8_t  M;    //灯效主模式
	uint8_t  S;    //灯效从模式
	uint8_t  T;    //灯效时间参数
	
	uint8_t  R;     //灯效颜色 R
	uint8_t  G;     //灯效颜色 G
	uint8_t  B;     //灯效颜色 B
	
}ROBOT_Light;

//杂类
#define  PI           3.1416     //圆周率PI
#define  SQRT3        1.732      //3开平方
#define  PID_RATE     100         //PID频率

//2S锂电池电量电压关系
#define  VBAT_40P    710      //电池40%电压
#define  VBAT_20P    675      //电池20%电压
#define  VBAT_10P    656      //电池10%电压

//机器人软件版本
#define  ROBOT_FW_VER   "V1.00"

//平衡机器人型号定义
#define ROBOT_R5

/******机器人参数*************************************/
#define  WHEEL_DIAMETER	      0.065	  //轮子直径
#define  WHEEL_BASE           0.162	  //轮距，左右轮的距离
#define  WHEEL_RESOLUTION     1560.0  //编码器分辨率(13线),减速比30,13x30x4=1560
#define  WHEEL_SCALE          (PI*WHEEL_DIAMETER*PID_RATE/WHEEL_RESOLUTION)  //轮子速度m/s与编码器转换系数

/******机器人通信协议*************************************/
//USB串口通信帧头定义
#define  ID_UTX_ROS     0x10    //发送的ROS综合数据
#define  ID_UTX_DBG     0x11    //发送的平衡车调试数据

#define  ID_URX_VEL     0x50    //接收的ROS速度数据
#define  ID_URX_BLC     0x02    //直立PID控制参数
#define  ID_URX_BLV     0x03    //速度PID控制参数
#define  ID_URX_BLT     0x04    //转向PID控制参数

//蓝牙APP通信帧头定义
#define  ID_BLERX_CM    0x30    //APP蓝牙发送 蓝牙连接指令
#define  ID_BLERX_YG    0x31    //APP蓝牙发送 摇杆模式 控制指令
#define  ID_BLERX_SB    0x32    //APP蓝牙发送 手柄模式 控制指令
#define  ID_BLERX_ZL    0x33    //APP蓝牙发送 重力模式 控制指令
#define  ID_BLERX_TK    0x34    //APP蓝牙发送 坦克模式 控制指令
#define  ID_BLERX_AM    0x3A    //APP蓝牙发送 机械臂模式 控制指令
#define  ID_BLERX_LG    0x41    //APP蓝牙发送 灯光控制指令
#define  ID_BLERX_LS    0x42    //APP蓝牙发送 保存灯光效果指令

//灯光模式
#define  LEFFECT1    0x01    //单色模式  
#define  LEFFECT2    0x02    //呼吸模式  
#define  LEFFECT3    0x03    //彩色模式  
#define  LEFFECT4    0x04    //警灯模式 
#define  LEFFECT5    0x05    //定义模式  
#define  LEFFECT6    0x06    //定义模式

//机器人控制方式
#define  CTL_PS2    0x01    //PS2手柄控制
#define  CTL_APP    0x02    //APP控制
#define  CTL_RMS    0x03    //航模遥控器控制
#define  CTL_ROS    0x04    //ROS控制

//功能清单
#define  CTL_FN1    0x11    //功能-雷达避障行驶
#define  CTL_FN2    0x12    //功能-雷达警报
#define  CTL_FN3    0x13    //功能-雷达跟随
#define  CTL_FN4    0x14    //功能-雷达沿直线
#define  CTL_FN5    0x15    //功能-CCD巡线
 
#define  CTL_FN_MAX    CTL_FN5    //模式最大值

//蜂鸣器鸣长短
#define  BEEP_SHORT   0x01    //蜂鸣器短鸣叫一声(200ms)
#define  BEEP_LONG    0x02    //蜂鸣器长鸣叫一声(1000ms)

//云台关节角度限制，单位角度放大10倍
#define JOINTA_UP_LIMIT    900
#define JOINTA_LOW_LIMIT  -900
#define JOINTB_UP_LIMIT    900
#define JOINTB_LOW_LIMIT  -900

//舵机云台零偏修正角 
#define JOINTA_ANGLE_OFFSET   (0)     //S1舵机，水平关节
#define JOINTB_ANGLE_OFFSET   (0)     //S2舵机，垂直关节

//全局变量
extern int16_t ax_robot_vx;
extern int16_t ax_robot_vw;
extern uint8_t ax_robot_move_enable;
extern uint8_t ax_function_enable;
extern uint8_t ax_control_mode;

extern int16_t ax_acc_data[3];
extern int16_t ax_gyro_data[3];
extern int16_t ax_angle_data[3];  

extern  ROBOT_Light  R_Light; 
extern uint8_t ax_light_enable;
extern uint8_t ax_beep_ring;  
extern  uint16_t ax_battery_vel; 

extern double ax_wheel_vel_l;
extern double ax_wheel_vel_r;

extern int16_t  ax_middle_angle;  

extern int16_t  ax_balance_angle; 
extern int16_t  ax_balance_gyro;  
extern int16_t  ax_balance_out;   
extern int16_t  ax_balance_kp;
extern int16_t  ax_balance_kd;	

extern int16_t  ax_velocity;  
extern int16_t  ax_velocity_out;	
extern int16_t  ax_velocity_kp;	
extern int16_t  ax_velocity_ki;	   

extern int16_t  ax_turn_gyro;   
extern int16_t  ax_turn_out;	 
extern int16_t  ax_turn_kp;	
extern int16_t  ax_turn_kd;	


extern int16_t ax_joint_angle[2];

//任务句柄
extern TaskHandle_t Control_Task_Handle;
extern TaskHandle_t Key_Task_Handle;
extern TaskHandle_t Disp_Task_Handle;
extern TaskHandle_t Trivia_Task_Handle;

#endif

/******************* (C) 版权 2023 XTARK **************************************/
