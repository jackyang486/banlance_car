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
  * @内  容  MPU6050 DMP操作
  *
  ******************************************************************************
  * @说  明
  *
  * 1.MPU6050 DMP姿态解算
  *
  ******************************************************************************
  */  

#include "mpu6050_dmp.h" 
//#include "lcd_spi_169.h"
#include <stdio.h>
#include <math.h>

//#include "ax_basis.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"

#define DEFAULT_MPU_HZ  (100)  //输出频率
#define q15   32768.0f   
#define q30   1073741824.0f

short ax_gyro[3], ax_accel[3], sensors;
float ax_pitch,ax_roll,ax_yaw; 
float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;

static signed char gyro_orientation[9] = { 1, 0, 0,
                                           0, 1, 0,
                                           0, 0, 1};

//										   
static uint16_t inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}

//设置方向
static  uint16_t inv_orientation_matrix_to_scalar(const signed char *mtx)
{
    unsigned short scalar;
    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;

    return scalar;
}

//自测试校准
static void run_self_test(void)
{
    int result;
    long gyro[3], accel[3];

    result = mpu_run_self_test(gyro, accel);
	
    if (result == 0x3) 
	{
        /* Test passed. We can trust the gyro data here, so let's push it down
         * to the DMP.
         */
        float sens;
        unsigned short accel_sens;
		
        mpu_get_gyro_sens(&sens);
        gyro[0] = (long)(gyro[0] * sens);
        gyro[1] = (long)(gyro[1] * sens);
        gyro[2] = (long)(gyro[2] * sens);
        dmp_set_gyro_bias(gyro);
		
        mpu_get_accel_sens(&accel_sens);
        accel[0] *= accel_sens;
        accel[1] *= accel_sens;
        accel[2] *= accel_sens;
        dmp_set_accel_bias(accel);
    }
}
//void lcd_printf(char* str){
//	LCD_SetAsciiFont(&ASCII_Font16);		
//	LCD_DisplayString(30,170,str);
//}
/**
  * @简  述  MPU6050 DMP初始化
  * @参  数  无	  
  * @返回值  无
  */
void AX_MPU6050_DMP_Init(void)
{
    u8 res=0;

	if(mpu_init() == 0)
	{
		res=mpu_set_sensors(INV_XYZ_GYRO|INV_XYZ_ACCEL);//设置所需要的传感器
		//if(res) lcd_printf("mpu_set_sensor error\r\n");
		
		res=mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);//设置FIFO
		//if(res) lcd_printf("mpu_configure_fifo error\r\n"); 
		
		res=mpu_set_sample_rate(DEFAULT_MPU_HZ);	//设置采样率
		//if(res) lcd_printf("mpu_set_sample_rate error\r\n");
		
		res=dmp_load_motion_driver_firmware();		//加载dmp固件
		//if(res) lcd_printf("dmp_load_motion_driver_firmware error\r\n"); 
		
		res=dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation));//设置陀螺仪方向
		//if(res) lcd_printf("dmp_set_orientation error\r\n");; 
		
		res=dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT|DMP_FEATURE_TAP|	//设置dmp功能
		    DMP_FEATURE_ANDROID_ORIENT|DMP_FEATURE_SEND_RAW_ACCEL|DMP_FEATURE_SEND_CAL_GYRO|
		    DMP_FEATURE_GYRO_CAL);
		//if(res) lcd_printf("dmp_enable_feature error\r\n");
		
		res=dmp_set_fifo_rate(DEFAULT_MPU_HZ);	//设置DMP输出速率(最大不超过200Hz)
		//if(res) lcd_printf("dmp_set_fifo_rate error.\r\n");
		
		run_self_test();  //校准
		
		res=mpu_set_dmp_state(1);  //使能DMP
		//if(res) lcd_printf("mpu_set_dmp_state error.\r\n"); 
	}
}

/**
  * @简  述  MPU6050 MDP获取解算数据
  * @参  数  无	  
  * @返回值  无
  * @全  局
	data[0-2] 陀螺仪
    data[3-5] 加速度
    data[6-8] 欧拉角 横滚，俯仰，航向，实际角度扩大100倍
    pitch:俯仰角 精度:0.1°   范围:-90.0° <---> +90.0°
	roll:横滚角  精度:0.1°   范围:-180.0°<---> +180.0°
	yaw:航向角   精度:0.1°   范围:-180.0°<---> +180.0°
  */

uint8_t AX_MPU6050_DMP_GetData(int16_t *gyro, int16_t *acc, int16_t *angle)
{
	unsigned long sensor_timestamp;
	unsigned char more;
    int result;
	long quat[4];
    result = dmp_read_fifo(gyro, acc, quat, &sensor_timestamp, &sensors, &more);
    if (result != 0 || !(sensors & INV_WXYZ_QUAT))
        return 1;
	
	if ( sensors & INV_WXYZ_QUAT )
	{
		 q0=quat[0] / q30;
		 q1=quat[1] / q30;
		 q2=quat[2] / q30;
		 q3=quat[3] / q30;		
		
		 ax_pitch = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3; 	//pitch
		 ax_roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; //roll
		 ax_yaw  = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;	//yaw
		
		 angle[0] = ax_roll*100;		
		 angle[1] = ax_pitch*100;	
		 angle[2] = ax_yaw*100;	
	}
    return 0;
}


/******************* (C) 版权 2023 XTARK **************************************/
