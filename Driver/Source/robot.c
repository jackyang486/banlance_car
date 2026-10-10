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

/* Includes ------------------------------------------------------------------*/
#include "robot.h"

//机器人RGB数据
ROBOT_Light  R_Light;

//IMU数据
int16_t ax_acc_data[3];
int16_t ax_gyro_data[3];
int16_t ax_angle_data[3];

//机器人运动使能开关,默认打开状态
uint8_t ax_robot_move_enable = 0;

//机器人功能使能开关,默认关闭状态
uint8_t ax_function_enable = 0;

//灯光开关失能，默认打开状态
uint8_t ax_light_enable = 1;

//灯光效果，效果保存
uint8_t ax_light_save_flag = 0;

//蜂鸣器鸣叫一声
uint8_t ax_beep_ring = 0;

//控制方式选择
uint8_t ax_control_mode = CTL_ROS;

//机器人电池电压数据
uint16_t ax_battery_vel; 

//舵机云台关节角度
int16_t ax_joint_angle[2] = {0};  

//机器人速度
int16_t ax_robot_vx = 0;  //前进为正，单位mm/s 
int16_t ax_robot_vw = 0;  //逆时针为正，单位度/s

//机器人平衡中值
int16_t  ax_middle_angle = 0;   //小车机械中间角度，默认为0

//机器人左右轮实时速度
double ax_wheel_vel_l = 0;
double ax_wheel_vel_r = 0;
 
//机器人平衡控制
int16_t  ax_balance_angle; //平衡倾角 
int16_t  ax_balance_gyro;  //平衡陀螺仪角速度
int16_t  ax_balance_out;   //平衡控制输出
int16_t  ax_balance_kp = 2700; //1800;  //2700;	//平衡kp   
int16_t  ax_balance_kd = 135;	//60; 平衡Kd 

//机器人速度控制
int16_t  ax_velocity = 0;  //速度值
int16_t  ax_velocity_out;	//速度控制输出
int16_t  ax_velocity_kp = 8000;	  //速度kp
int16_t  ax_velocity_ki = 4000;	    //速度ki

//机器人转向控制
int16_t  ax_turn_gyro;    //转向陀螺仪角速度
int16_t  ax_turn_out;	  //转向控制输出
int16_t  ax_turn_kp = 1000;	//转向kp系数
int16_t  ax_turn_kd = 5000;	//转向kd系数

/******************* (C) 版权 2023 XTARK **************************************/

