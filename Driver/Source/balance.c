/* Includes ------------------------------------------------------------------*/
#include "balance.h"
#include "robot.h"
#include "encoder.h"
#include "motor.h"

//内部函数定义
int16_t Balance_AngleCtl(int16_t target, float angle, float gyro);  
int16_t Balance_VelocityCtl(int16_t t_vx, float r_vx ); 
int16_t Balance_TurnCtl(int16_t t_vw, float r_vw);  
MPU_Angle mpu_angle;
void Balance_FallProtect(void);
void Balance_PutDown(void);
void ROS_SendDataHandle(void); 
void DBG_SendDataHandle(void); 
volatile uint8_t mpu6050_dmp_ready = 0;

/**
  * @简  述  平衡小车控制核心代码
             MPU6050的INT中断触发，10ms一次，严格时序同步
  * @参  数  无
  * @返回值  无
  */
void EXTI9_5_IRQHandler(void)
{
	//确认中断,进入控制周期
	if(EXTI_GetITStatus(EXTI_Line5) != RESET) 
	{		
		mpu6050_dmp_ready = 1;
		//清除中断标志位
		EXTI_ClearITPendingBit(EXTI_Line5);
		
	}
}

void Balance_execute(void){
		
	static float balance_angle,balance_gyro,turn_gyro;
	static float wheel_vel;
	int16_t wheel_pwm_l,wheel_pwm_r;
	
	if (mpu6050_dmp_ready)
		{
      	 mpu6050_dmp_ready = 0;        
        	//获取IMU数据
		if (AX_MPU6050_DMP_GetData(ax_gyro_data,ax_acc_data,ax_angle_data) != 0)
		{
			MOTOR_A_SetSpeed(0);
			MOTOR_B_SetSpeed(0);
			return;
		}
		mpu_angle.balanceGyro  = (float)ax_gyro_data[1] * 0.061f;  //航向角，单位度/s
		mpu_angle.balanceAngle = (float)ax_angle_data[1] * 0.01f;  //俯仰角，单位度
		mpu_angle.turnGyro  = (float)ax_gyro_data[2] * 0.061f;  //转向角速度，单位度/s
		mpu_angle.turnAngle = (float)ax_angle_data[2] * 0.01f;  //转向角，单位度
		ax_balance_angle = ax_angle_data[1];  //平衡倾角，前倾负，单位0.01度
		ax_balance_gyro  = ax_gyro_data[1];   //平衡角速度，前倾负，量程±2000度/s，范围±32768，一个数字代表2000/32768=0.061，可查手册
		ax_turn_gyro     = ax_gyro_data[2];   //转向角速度，逆时针正，量程±2000度/s，范围±32768，一个数字代表2000/32768=0.061，可查手册
		
		//换算到浮点数
		balance_angle = 0.01f * ax_balance_angle;   //倾角浮点数，单位度    
		balance_gyro  = 0.061f * ax_balance_gyro;   //平衡角速度浮点数，单位度/s
		turn_gyro     = 0.061f * ax_turn_gyro;      //转动角速度浮点数，单位度/s
		
		//通过编码器获取车轮实时转速m/s，前进正，后退负
		ax_wheel_vel_l = (float) ((int16_t)ENCODER_A_GetCounter()*WHEEL_SCALE);  //左轮转速
		ENCODER_A_SetCounter(0);
		ax_wheel_vel_r = (float)-((int16_t)ENCODER_B_GetCounter()*WHEEL_SCALE);  //右轮转速
		ENCODER_B_SetCounter(0);	
		wheel_vel = (ax_wheel_vel_l + ax_wheel_vel_r)/2;	//计算小车速度	

		
		//PID运动控制
		ax_balance_out  = Balance_AngleCtl(ax_middle_angle,balance_angle, balance_gyro);  //角度控制
	  ax_velocity_out = Balance_VelocityCtl(ax_robot_vx, wheel_vel);   //行驶速度控制
	  ax_turn_out     = Balance_TurnCtl(ax_robot_vw, turn_gyro);	   //转向速度控制	

		//计算左右轮速度值
		wheel_pwm_l  = ax_balance_out + ax_velocity_out - ax_turn_out;
		wheel_pwm_r  = ax_balance_out + ax_velocity_out + ax_turn_out;	
			
		//电机是否运行由该标志位控制
		if(ax_robot_move_enable == 1)
		{	
			MOTOR_A_SetSpeed( -wheel_pwm_l);//控制左侧电机
			MOTOR_B_SetSpeed(  wheel_pwm_r);//控制右侧电机
		}
		else
		{
			//电机速度设置为0
			MOTOR_A_SetSpeed(0);
			MOTOR_B_SetSpeed(0);
		}
		//安全放下启动，小车直立（角度小于5度），轮子有转动
		Balance_PutDown();
		//平衡角度小于>45度或者小于-45度角时，停止转动。
		Balance_FallProtect();	
   }

}


/**
  * @简  述  平衡小车直立PID控制
  * @参  数  target 目标平衡角度，小车机械中间角度，默认为0
  *          angle  当前平衡角度，单位度
  *          gyro   当前平衡角速度，单位度/s
  * @返回值  PID控制输出值电机PWM值
  */
int16_t Balance_AngleCtl(int16_t target, float angle, float gyro)
{ 
	
	float angle_bias,gyro_bias;  //角度和角速度偏差
	int16_t p_out,d_out,out;    //PID输出值
	
	//计算平衡角度偏差
	angle_bias = target - angle;  
	
	//计算平衡角速度偏差
	gyro_bias  = 0 - gyro;  
	
	//计算平衡角度控制PWM输出，pk，pd为系数，out = kp*e(k) + kd*[e(k)-e(k-1)]
	p_out = ax_balance_kp * angle_bias * 0.1f;
	d_out = ax_balance_kd * gyro_bias * 0.1f;
	
	//计算总输出值
	out =  p_out + d_out;
	
	//调试数据查看
	//printf("@%d %d %d \r\n ",out, p_out, d_out); 
	
	return out;
}

/**
  * @简  述  速度PID控制
  * @参  数  t_vx 目标速度，单位为mm/s，范围0~700
  *          r_vx 实时速度，单位m/s
  * @返回值  PID控制输出值电机PWM值
  */
int16_t Balance_VelocityCtl(int16_t t_vx, float r_vx)
{ 
	
	static float velocity_bias = 0, velocity_bias_update;
	static float velocity_integral = 0;
	float  velocity_target;  //速度目标值
	int16_t p_out,i_out,out; //PID输出值
	
	//目标速度限幅
	if(t_vx > 700) t_vx = 700;
	else if(t_vx <-700) t_vx = -700;
	
	//计算速度偏差，目标速度为0
	velocity_bias_update = 0 - r_vx;
	
	//一阶低通滤波器, 减小速度波动对平衡控制影响
	velocity_bias = (velocity_bias*0.6) + (velocity_bias_update*0.4); 
	
	//速度积分计算出位移，单位m，积分时间10ms
	velocity_integral += velocity_bias*0.01f;  
	
	//计算目标速度（单位为m/s）,vx单位为mm/s,积分时间0.01s
	velocity_target = t_vx*0.001f*0.01f;
	
	//加入目标速度控制，控制行驶速度
	velocity_integral += velocity_target; 
	
	//如果电机停止，则清除积分项
	if(ax_robot_move_enable == 0)
	{
		velocity_integral = 0;
	}
	
	//限制积分饱和
	if(velocity_integral > 1.5)  velocity_integral = 1.5;
	else if(velocity_integral < -1.5)  velocity_integral = -1.5;

	//计算速度控制PWM输出，pk，pi为系数
	p_out = -ax_velocity_kp * velocity_bias;
	i_out = -ax_velocity_ki * velocity_integral;
	
	//计算总输出值
	out =  p_out + i_out;
	
    //调试数据查看
	//printf("@%d %d %d \r\n ", out, p_out,i_out);   
	//printf("@%d %d \r\n ",vx, (int)(-velocity_bias_update*1000) );
	
	return out;

}


/**
  * @简  述  转向PID控制
  * @参  数  t_vw 目标旋转速度，逆时针为正，单位度/s
                  范围0~360相对安全，过快速度会破坏平衡
  *          r_vw 实时旋转速度，陀螺仪转向速度，单位度/s
  * @返回值  PID控制输出值电机PWM值
  */
int16_t Balance_TurnCtl(int16_t t_vw, float r_vw)
{
	static float bias = 0;
	static float bias_last = 0;
	
	//PID输出值
	static int16_t p_out=0,d_out=0,out=0;  


	//如果电机停止，则清除转向输出
	if(ax_robot_move_enable == 0)
	{
		out = 0;
	}	

	//计算转向速度偏差
	bias = t_vw - r_vw;
	
	//计算平衡转向控制PWM输出
	p_out =  ax_turn_kp * bias * 0.001f;  
    d_out =  ax_turn_kd * (bias-bias_last) * 0.001f;
	
	//计算总输出值
	out +=  p_out + d_out;
	
	//记录上次偏差
	bias_last = bias;
	
    //调试数据查看
	//printf("@%d %d %d \r\n ", (int)(bias*10), (int)(t_vw*10), (int)(r_vw*10)); 
	//printf("@%d %d %d \r\n ", out, p_out,d_out);   
	
	return out;
}

/**
  * @简  述  平衡小车放下检测
  *          小车直立放下（角度小于5度）,轮子有轻微转动
  * @参  数  无
  * @返回值  无
  */
void Balance_PutDown(void)
{
	static uint16_t cont;
	int16_t wheel_l,wheel_r;
	
	//获取左右轮转速
	wheel_l = (int16_t)(ax_wheel_vel_l*1000);
	wheel_r = (int16_t)(ax_wheel_vel_r*1000);
	
	//小车平衡系统关闭状态下
	if(ax_robot_move_enable == 0)
	{
		if(cont == 0)
		{
			//判断1，角度小于5度，编码器速度为0
			if(ax_balance_angle<500 && ax_balance_angle>-500 && wheel_l==0 &&  wheel_r==0)
			{
				cont++;
			}		
		}
		else
		{			
			if(cont<50)
			{
				cont++;
				
				//判断2，轻轻前后推动，左右轮产生速度
				if(wheel_l>1 || wheel_l<-1 || wheel_r>1 || wheel_r<-1) 
				{
					//速度设置为零
					ax_robot_vx = 0;
					ax_robot_vw = 0;
					
					//打开电机使能
					ax_robot_move_enable = 1;   
					
					//清理计数器
					cont = 0;
				}
			}
			else  //超时500ms退出
			{
				//清理计数器
				cont = 0;
			}
		}		
	}
}

/**
  * @简  述  小车摔倒保护，角度大于45度，认为小车摔倒
  * @参  数  无
  * @返回值  无
  */
void Balance_FallProtect(void)
{
	//倾角大于45度进入保护状态
	if((ax_balance_angle>4500)||(ax_balance_angle<-4500))
	{
		//电机运动使能
		ax_robot_move_enable=0;
	}
}

/**
  * @简  述  ROS处理函数，包含IMU数据封装、速度正解和数据发送
  * @参  数  无
  * @返回值  无
  */
void ROS_SendDataHandle(void)   
{
	int16_t temp[3];
	
	//串口发送数据
	static uint8_t comdata[20]; 	

	//加速度 = (ax_acc/32768) * 2G 
	temp[0] =  ax_acc_data[1];   //ROS坐标X轴对应IMU的Y轴
	temp[1] = -ax_acc_data[0];   //ROS坐标Y轴对应IMU的X轴反向
	temp[2] =  ax_acc_data[2];   //ROS坐标Z轴对应IMU的Z轴
	
	comdata[0] = (u8)( temp[0] >> 8 );  
	comdata[1] = (u8)( temp[0] );
	comdata[2] = (u8)( temp[1] >> 8 );
	comdata[3] = (u8)( temp[1] );
	comdata[4] = (u8)( temp[2] >> 8 );
	comdata[5] = (u8)( temp[2] );
	
	//陀螺仪角速度 = (ax_gyro/32768) * 2000
	temp[0] =  ax_gyro_data[1];   //ROS坐标X轴对应IMU的Y轴
	temp[1] = -ax_gyro_data[0];   //ROS坐标Y轴对应IMU的X轴反向
	temp[2] =  ax_gyro_data[2];   //ROS坐标Z轴对应IMU的Z轴	

	comdata[6] = (u8)( temp[0] >> 8 );   
	comdata[7] = (u8)( temp[0] );
	comdata[8] = (u8)( temp[1] >> 8 );
	comdata[9] = (u8)( temp[1] );
	comdata[10] = (u8)( temp[2] >> 8 );
	comdata[11] = (u8)( temp[2] );
	
	//运动学正解析，由机器人轮子速度计算机器人速度
	//机器人速度值 单位为m/s，放大1000倍
	temp[0] = ((ax_wheel_vel_l + ax_wheel_vel_r)/2)*1000;  //机器人实时X速度
	temp[1] = 0;  //机器人实Y速度
	temp[2] = ((-ax_wheel_vel_l + ax_wheel_vel_r)/WHEEL_BASE)*1000;		//机器人实W速度
	
	comdata[12] = (u8)( temp[0] >> 8 );  
	comdata[13] = (u8)( temp[0] );
	comdata[14] = (u8)( temp[1] >> 8 );
	comdata[15] = (u8)( temp[1] );
	comdata[16] = (u8)( temp[2] >> 8 );
	comdata[17] = (u8)( temp[2] );	

	//电池电压
	comdata[18] = (u8)( ax_battery_vel >> 8 );
	comdata[19] = (u8)( ax_battery_vel );	
	
	//USB串口发送数据（与调试串口共用，需要调试时可注释掉该句）
  //  AX_UART1_SendPacket(comdata, 20, ID_UTX_ROS);	
		
}

/**
  * @简  述  调试数据，包含平衡角度、角速度、控制量等数据发送
  * @参  数  无
  * @返回值  无
  */
void DBG_SendDataHandle(void)   
{
	int16_t temp[2];
	
	//串口发送数据
	static uint8_t comdata[14]; 	

	//平衡控制相关
	temp[0] =  ax_balance_angle;  //平衡角度
	temp[1] =  ax_balance_gyro;   //平衡角度度，陀螺仪测量
	
	comdata[0] = (u8)( temp[0] >> 8 );  
	comdata[1] = (u8)( temp[0] );
	comdata[2] = (u8)( temp[1] >> 8 );
	comdata[3] = (u8)( temp[1] );

	//速度控制数据
	temp[0] =  ax_robot_vx;   //目标速度数据，设定值
	temp[1] =  ax_velocity;   //实际速度数据，编码器测量值

	comdata[4] = (u8)( temp[0] >> 8 );  
	comdata[5] = (u8)( temp[0] );
	comdata[6] = (u8)( temp[1] >> 8 );
	comdata[7] = (u8)( temp[1] );
	
	//转向控制数据
	temp[0] = ax_robot_vw * 10;  //目标转向速度数据，设定值
	temp[1] = 0.61f * ax_turn_gyro;  //实际转向速度数据，陀螺仪测量值
	
	comdata[8] = (u8)( temp[0] >> 8 );  
	comdata[9] = (u8)( temp[0] );
	comdata[10] = (u8)( temp[1] >> 8 );
	comdata[11] = (u8)( temp[1] );
	
	//USB串口发送数据（与调试串口共用，需要调试时可注释掉该句）
  //  AX_UART1_SendPacket(comdata, 12, ID_UTX_DBG);	
		
}

/******************* (C) 版权 2023 XTARK **************************************/

