/**			                                                    
		   ____                    _____ _______ _____       XTARK@   ˴   
		  / __ \                  / ____|__   __|  __ \ 
		 | |  | |_ __   ___ _ __ | |       | |  | |__) |
		 | |  | | '_ \ / _ \ '_ \| |       | |  |  _  / 
		 | |__| | |_) |  __/ | | | |____   | |  | | \ \ 
		  \____/| .__/ \___|_| |_|\_____|  |_|  |_|  \_\
				| |                                     
				|_|                OpenCTR        ˿     
									 
  ****************************************************************************** 
  *           
  *   Ȩ   У  XTARK@   ˴       Ȩ   У     ؾ 
  *   ˾  վ   www.xtark.cn   www.tarkbot.com
  *  Ա    ̣  https://xtark.taobao.com  
  *     ΢ ţ     ˴  £   ע   ںţ   ȡ   ¸     Ѷ  
  *      
  ******************************************************************************
  * @        Musk Han@XTARK
  * @                     ļ 
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TASKS_H
#define __TASKS_H

/* Includes ------------------------------------------------------------------*/	 
#include "stm32f10x.h"
void Control_Task(void* parameter);
void Light_Task(void* parameter);
void Ps2_Task(void* parameter);
void lcd_task(void *pvParameters);
void balance_task(void *pvParameters);

#endif

/******************* (C)   Ȩ 2023 XTARK **************************************/
