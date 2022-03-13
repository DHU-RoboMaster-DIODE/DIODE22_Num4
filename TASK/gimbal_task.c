/**
  ****************************(C) COPYRIGHT 2021 DIODE****************************
  * @file       gimbal_task.c/h
  * @brief      云台控制任务
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2021 DIODE****************************
  */
#include "gimbal_task.h"
#include "chassis_task.h"
#include "pid.h"
#include "bsp_can.h"
#include "INS_task.h"
#define pitch_angle_min	-15
#define pitch_angle_max	30

extern uint16_t RC_FLAG;
uint16_t time_gimbal;
float Kp=70,Ki=0,Kd=300,Kps=100,Kis=0,Kds=50;

double imu_last_anglesum;
double imu_angle_bias;
double imu_agbias_sum;
float pit_speed;
float lowpassp;
float lowpassp_last = 0;
float lowpassy;
float lowpassy_last = 0;
float imukal_speed;
uint8_t  flag_case2=1;

extern fp32 INS_gyro[];
extern float GM6020speed;
extern float GM6020speed_pit;
extern double imu_last_anglesum;
double last_imu_yaw;
//extKalman_t imu_speed;
//extKalman_t pit_data_speed;

void angle_sum(void);
void gimbal_rc_ctrl(void);
void gimbal_pc_ctrl(void);
void gimbal_control_loop(void);
/**
  * @brief          云台任务，绝对延时 CHASSIS_CONTROL_TIME_MS 2ms
  * @param[in]      pvParameters: 空
  * @retval         none
  */
void gimbal_task(void const *pvParameters)
{  
//		KalmanCreate(&vis_data_yaw,20,200);
    PID_Init(&PID_GM6020[0],POSITION_PID,1000,1000,60,0,500);	//45,0,200  50,1,300云台底部电机500,0,1500  自瞄：1000，0.3，1500     500,0,1500     1200,0.1,4200
    PID_Init(&PID_GM6020_speed[0],POSITION_PID,30000,10000,100,0,50);//50,0.01,4 50,0,3  0.0001   5 0 0    4000,0,4000
		PID_Init(&PID_GM6020[1],POSITION_PID,1000,1000,60,0,200);		// 云台侧面电机1600
    PID_Init(&PID_GM6020_speed[1],POSITION_PID,30000,10000,70,0,0);//0.0001
		PID_Init(&PID_M3508_ANGLE[6],DELTA_PID,20,20,5,0,0);//云台roll轴电机
		PID_Init(&PID_M3508[6],POSITION_PID,8000,2500,1.5,0,1);
	
	
    static portTickType lastWakeTime;   
	  yaw_angle_set=INS_angle[0];
	
    while (1)
    { 
			 lastWakeTime = xTaskGetTickCount(); 
       time_gimbal++;
			 if(rc_flag)
			 {
				  switch(rc.sw2){
						case 1:
							gimbal_pc_ctrl();
							break;
						case 3:
							actChassis=CHASSIS_FOLLOW_GIMBAL;	//底盘跟随云台
							gimbal_rc_ctrl();
							
						  break;
						case 2:
							actChassis=CHASSIS_GYROSCOPE;	//底盘小陀螺
							gimbal_rc_ctrl(); 
						  break;
						default:
              break;
					}
			 }
			 else    //若遥控器失控关闭清空电流值
			 {
        		CAN_GM6020[0].set_voltage=0;
            CAN_GM6020[1].set_voltage=0;
			 }
       CAN_Gimbal_SendVoltage();
       //系统绝对延时
       osDelayUntil(&lastWakeTime,GIMBAL_CONTROL_TIME_MS);
    }
}

void gimbal_rc_ctrl(void)
{
	
	switch(rc.sw2){
		case 2:		
		case 3:
			pitch_angle_set -= (float)rc.ch1/4000.0f;//俯仰轴
			if(pitch_angle_set>=pitch_angle_max) pitch_angle_set= pitch_angle_max;
			else if(pitch_angle_set<=pitch_angle_min) pitch_angle_set= pitch_angle_min; 
		  yaw_angle_set -=(float)rc.ch0/3000.0f;
		  yaw_angle_set=Find_MIN_ANGLE(yaw_angle_set,0);
		  
      CAN_GM6020[0].set_angle_speed=PID_Calculate(&PID_GM6020[0],Find_MIN_ANGLE(yaw_angle_set,INS_angle[0]*57.295779513082f),0);
			CAN_GM6020[0].set_voltage = PID_Calculate(&PID_GM6020_speed[0],CAN_GM6020[0].set_angle_speed,INS_gyro[0]*57.295779513082f);
		  CAN_GM6020[1].set_angle_speed=PID_Calculate(&PID_GM6020[1],pitch_angle_set,INS_angle[1]*57.295779513082f);
			CAN_GM6020[1].set_voltage = -PID_Calculate(&PID_GM6020_speed[1],CAN_GM6020[1].set_angle_speed,INS_gyro[1]*57.295779513082f);
		  break;			
  }
}

void gimbal_pc_ctrl(void)
{  
	 pitch_angle_set += (float)rc.mouse_y/300.0f;//俯仰轴
	 if(pitch_angle_set>=pitch_angle_max) pitch_angle_set= pitch_angle_max;
	 else if(pitch_angle_set<=pitch_angle_min) pitch_angle_set= pitch_angle_min; 
	 yaw_angle_set -=(float)rc.mouse_x/300.0f;
	
	 yaw_angle_set=Find_MIN_ANGLE(yaw_angle_set,0);
   CAN_GM6020[0].set_angle_speed=PID_Calculate(&PID_GM6020[0],Find_MIN_ANGLE(yaw_angle_set,INS_angle[0]*57.295779513082f),0);
	 CAN_GM6020[0].set_voltage = PID_Calculate(&PID_GM6020_speed[0],CAN_GM6020[0].set_angle_speed,INS_gyro[0]*57.295779513082f);
	 CAN_GM6020[1].set_angle_speed=PID_Calculate(&PID_GM6020[1],pitch_angle_set,INS_angle[1]*57.295779513082f);
	 CAN_GM6020[1].set_voltage = PID_Calculate(&PID_GM6020_speed[1],CAN_GM6020[1].set_angle_speed,INS_gyro[1]*57.295779513082f);	
}


fp32 Find_MIN_ANGLE(float set,float feed)
{
    fp32 temp = set - feed;
    if(temp >=180)
        temp -= 360;
    else if(temp < -180)
        temp += 360;
    return temp;
}
