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
  * @内  容  机器人控制处理文件
  * 
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "control.h"
#include "robot.h"
#include "usart.h"


/**
  * @简  述  处理手机APP控制命令
  * @参  数  无
  * @返回值  无
  */
void AX_CTL_App(void)
{
	static uint8_t comdata[16];
	
	//接收蓝牙APP串口数据
	if(AX_UART1_GetData(comdata))
	{
		//摇杆模式运动控制帧
		if((comdata[0] == ID_BLERX_YG))
		{
			ax_robot_vx = (int16_t)(  5*(int8_t)comdata[4] );
			ax_robot_vw = (int16_t)( -2*(int8_t)comdata[1] );
		}
		
		//手柄模式运动控制帧
		if((comdata[0] == ID_BLERX_SB))
		{
			ax_robot_vx = (int16_t)(  5*(int8_t)comdata[4] );
			ax_robot_vw = (int16_t)( -2*(int8_t)comdata[1] );
		}
		
		//重力模式运动控制帧
		if(comdata[0] == ID_BLERX_ZL)
		{
			ax_robot_vx = (int16_t)( -5*(int8_t)comdata[3] );
			ax_robot_vw = (int16_t)( -2*(int8_t)comdata[2] );
		}
		
		//灯光效果控制帧
		if(comdata[0] == ID_BLERX_LG)
		{
			R_Light.M  = comdata[1];
			R_Light.S  = comdata[2];
			R_Light.T  = comdata[3];
			R_Light.R  = comdata[4];
			R_Light.G  = comdata[5];
			R_Light.B  = comdata[6];
		}
	}
}


/******************* (C) 版权 2023 XTARK **************************************/

