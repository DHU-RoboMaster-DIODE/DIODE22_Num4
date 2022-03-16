/**
  ****************************(C) COPYRIGHT 2021 DIODE****************************
  * @file       chassis_task.c/h
  * @brief      chassis control task,
  *             ���̿�������
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
#include "chassis_task.h"
#include "rc_control.h"
#include "INS_task.h"
#include "arm_math.h"

uint16_t time_chassis;

#define WHEEL_RADIUS  15//���Ӱ뾶cm
#define LENGTH_A  40  //�־�
#define LENGTH_B  36  //���
#define CHASSIS_DECELE_RATIO  19 //���ٱ�

//�����˶� 
uint16_t classis_speed,spin_speed;   //���̻����ٶ�,С�����ٶȸ��ݲ���ϵͳ�ı�
uint8_t SpinStartFlag,IsSpinFlag;//С���ݱ�־
uint8_t FollowSwitchFlag,IsFollowFlag=1,LastIsFollowFlag=1;
float fTotalCurrentLimit;  //��������  ƽ������µ�������
float WARNING_REMAIN_POWER = 60;
float fChasCurrentLimit = 36000;

eChassisAction actChassis=CHASSIS_FOLLOW_GIMBAL;   //Ĭ�ϵ��̸�����̨����
eChassisAction actChassis_last = CHASSIS_FOLLOW_GIMBAL;
//�����˶�����
chassis_move_t chassis_move;
chassis_speed_t absolute_chassis;
/**
  * @brief          ���̿��ƺ���
  * @param[in]      pvParameters: ��
  * @retval         none
  */
void chassis_rc_ctrl(void);
void chassis_pc_ctrl(void);
void chassis_control_loop(void); 
void Chassis_Power_Limit(void);
/**
  * @brief          Ѱ����С�Ƕȣ��������汾�����ڱ�׼��-pi~pi
  * @param[in]      pvParameters: ���ýǶȺ�
  * @retval         none
  */	
fp32 Find_MIN_ANGLE_Enconder(float set,float feed);

/**
  * @brief          ����Ϊ�����ĸ�������ٶ�ֵ
  * @param[in]      pvParameters: ���������ٶȽṹ�壬����ṹ��
  * @retval         none
  */	
void mecanum_calc(chassis_speed_t *speed,chassis_move_t *out);
	
float logistic(int* x);

/**
  * @brief          �������񣬼�� CHASSIS_CONTROL_TIME_MS 2ms
  * @param[in]      pvParameters: ��
  * @retval         none
  */
void chassis_task(void const *pvParameters)
{
    //����һ��ʱ��
    osDelay(CHASSIS_TASK_INIT_TIME);
		classis_speed=4500;//�ƶ��ٶ�
		spin_speed=1000;//С�����ٶ�
    static portTickType lastWakeTime;  
    for(int i=0; i<4; i++)	// �ĸ����̵��
    {
        PID_Init(&PID_M3508[i],POSITION_PID,16000,5000,3,0.02,5);//5,0.01
    }
		PID_Init(&PID_M3508_Follow,POSITION_PID,1000,1000,3,0,5);
		chassis_move.chassis_init_angle_set=5013;
    while (1)
    {
			  lastWakeTime = xTaskGetTickCount(); 
				time_chassis++;
				if(rc_flag)
				{
				  switch(rc.sw2){
						case 1:

							chassis_pc_ctrl();
							break;
						case 3:
							actChassis=CHASSIS_FOLLOW_GIMBAL;	//���̸�����̨
							chassis_rc_ctrl();
						  break;
						case 2:
							actChassis=CHASSIS_GYROSCOPE;	//����С����
						
							chassis_rc_ctrl();
						  break;
						default:
              break;
					}
          chassis_control_loop(); //�����ٶȷֽ⼰pid����
				}
				else    //��ң����ʧ�عر���յ���ֵ
				{
					  CAN_M3508[0].set_current=0;
            CAN_M3508[1].set_current=0;
            CAN_M3508[2].set_current=0;
            CAN_M3508[3].set_current=0;
				}
				//CANͨ��������ĸ�������͵���ֵ
				Chassis_Power_Limit();
				CAN_Chassis_SendCurrent();
        //ϵͳ������ʱ
        osDelayUntil(&lastWakeTime,CHASSIS_CONTROL_TIME_MS);
    }
}


void chassis_rc_ctrl(void)
{  
    switch(actChassis)
    {
    case CHASSIS_FOLLOW_GIMBAL://������̨
        chassis_move.vx_set =(fp32)rc.ch3/2.5f; //ǰ�����
        chassis_move.vy_set =(fp32)rc.ch2/2.5f; //���Ҽ���
        chassis_move.wz_set=rc.ch0/800.0f+PID_Calculate(&PID_M3508_Follow,
		Find_MIN_ANGLE_Enconder(chassis_move.chassis_init_angle_set,CAN_GM6020[0].angle),0)/1000.0f;//PIDʹ���̸�����̨�ٶ�
        break;
    case CHASSIS_NORMAL://��������̨
        chassis_move.vx_set=(fp32)rc.ch3/3.0f;
        chassis_move.vy_set=(fp32)rc.ch2/3.0f;
        chassis_move.wz_set=0;
        break;
    case CHASSIS_GYROSCOPE:		//С����ģʽ
        chassis_move.vx_set=(fp32)rc.ch3/3.0f;
        chassis_move.vy_set=(fp32)rc.ch2/3.0f;
        chassis_move.wz_set=-2;
        break;
		case CHASSIS_SLOW:		//פ��ģʽ
        chassis_move.vx_set=0;
        chassis_move.vy_set=0;
        chassis_move.wz_set=0;
        break;
    default:
        break;
    }
}
//pc����ȫ�ֱ��������ڵ��Ժ��ڸ�Ϊ��̬�ṹ��
	float speedx,speedy;
  float speedxx,speedyy;
  int x1,x2,y1,y2;

void chassis_pc_ctrl(void)
{  
	  if(rc.key[5]) 
			actChassis=CHASSIS_GYROSCOPE;
		else 
			actChassis=CHASSIS_FOLLOW_GIMBAL;		
	
		if(rc.key[0]) speedx=logistic(&x1);  //w
		if(rc.key[1]) speedx=-logistic(&x2); //s
		if(rc.key[2]) speedy=-logistic(&y1); //a
		if(rc.key[3]) speedy=logistic(&y2);  //d
		if(speedx>=660) speedx=660;
		if(speedx<=-660) speedx=-660;
		if(speedy>=660) speedy=660;
		if(speedy<=-660) speedy=-660;
		if(!(rc.key[0]||rc.key[1])) {
			speedx=0;
			x1=x2=0;
		}
		if(!(rc.key[2]||rc.key[3])) {
			speedy=0;
			y1=y2=0;
		}
    switch(actChassis)
    {
    case CHASSIS_FOLLOW_GIMBAL://������̨
        chassis_move.vx_set =(fp32)speedx/2.0f; //ǰ�����
        chassis_move.vy_set =(fp32)speedy/2.0f; //���Ҽ���
        chassis_move.wz_set=rc.mouse_x/800.0f+PID_Calculate(&PID_M3508_Follow,
		Find_MIN_ANGLE_Enconder(chassis_move.chassis_init_angle_set,CAN_GM6020[0].angle),0)/1000.0f;//PIDʹ���̸�����̨�ٶ�
        break;
    case CHASSIS_NORMAL://��������̨
        chassis_move.vx_set=(fp32)speedx/2.0f;
        chassis_move.vy_set=(fp32)speedy/2.0f;
        chassis_move.wz_set=0;
        break;
    case CHASSIS_GYROSCOPE:		//С����ģʽ
        chassis_move.vx_set=(fp32)speedx/2.0f;
        chassis_move.vy_set=(fp32)speedy/2.0f;
        chassis_move.wz_set=-2;
        break;
		case CHASSIS_SLOW:		//פ��ģʽ
        chassis_move.vx_set=0;
        chassis_move.vy_set=0;
        chassis_move.wz_set=0;
        break;
    default:
        break;
    }
}

void Gimbal2Chassis(chassis_move_t *gimbal, chassis_speed_t *chassis,float angle)
{
    float angle_hd = angle * PI / 180;
    chassis->wz = gimbal->wz_set;
    chassis->vx = gimbal->vx_set * cos(angle_hd) + gimbal->vy_set * sin(angle_hd);
    chassis->vy = gimbal->vx_set * sin(angle_hd) - gimbal->vy_set * cos(angle_hd);
	
}

void chassis_control_loop(void){
  if(actChassis==CHASSIS_FOLLOW_GIMBAL)
	  Gimbal2Chassis(&chassis_move,&absolute_chassis,abs((int)CAN_GM6020[0].angle-chassis_move.chassis_init_angle_set)>3000?180:0);			
	else
  {
	  Gimbal2Chassis(&chassis_move,&absolute_chassis,
	  ((fp32)(CAN_GM6020[0].angle-chassis_move.chassis_init_angle_set))*0.043945f);			
  }
	mecanum_calc(&absolute_chassis,&chassis_move);
	
  for(int i=0; i<4; i++) 
     CAN_M3508[i].set_current =  PID_Calculate(&PID_M3508[i], chassis_move.setSpeed[i], CAN_M3508[i].speed);
}



void mecanum_calc(chassis_speed_t *speed,chassis_move_t *out)
{
	
    float wheel_rpm_ratio;

    wheel_rpm_ratio = 1/ (WHEEL_RADIUS * 3.14159f) * CHASSIS_DECELE_RATIO*60;
	
    out->setSpeed[0] = ( speed->vx - speed->vy + speed->wz * (LENGTH_A + LENGTH_B)) * wheel_rpm_ratio;
    out->setSpeed[1] = (-speed->vx - speed->vy + speed->wz * (LENGTH_A + LENGTH_B)) * wheel_rpm_ratio;
    out->setSpeed[2] = ( speed->vx + speed->vy + speed->wz * (LENGTH_A + LENGTH_B)) * wheel_rpm_ratio;
    out->setSpeed[3] = (-speed->vx + speed->vy + speed->wz * (LENGTH_A + LENGTH_B)) * wheel_rpm_ratio;

}


fp32 Find_MIN_ANGLE_Enconder(float set,float feed)
{
    fp32 temp = set - feed;
    if(temp >=4096)
        temp -= 8192;
    else if(temp < -4096)
        temp += 8192;
//		if(temp>2048)  temp-=4096;
//		else if(temp<-2048)  temp+=4096;
    return temp;
}
float logistic(int* x)
{
	float speed;
	(*x)++;
	if((*x)>=60) (*x)=60;
	speed=300/(1+exp(-(0.6*(*x)-10)));
	return speed;
}

////void Smooth_control(float vx,float vy,float vz)
////{
////	float step=0.02;

////	if     (vx>smooth_control.VX) smooth_control.VX+=step;
////	else if(vx<smooth_control.VX) smooth_control.VX-=step;
////	else                          smooth_control.VX =vx;
////	
////	if     (vy>smooth_control.VY) smooth_control.VY+=step;
////	else if(vy<smooth_control.VY) smooth_control.VY-=step;
////	else                          smooth_control.VY =vy;
////	
////	if     (vz>smooth_control.VZ) smooth_control.VZ+=step;
////	else if(vz<smooth_control.VZ) smooth_control.VZ-=step;
////	else                          smooth_control.VZ =vz;
////	
////	if(vx==0&&smooth_control.VX<0.05f&&smooth_control.VX>-0.05f) smooth_control.VX=0;
////	if(vy==0&&smooth_control.VY<0.05f&&smooth_control.VY>-0.05f) smooth_control.VY=0;
////	if(vz==0&&smooth_control.VZ<0.05f&&smooth_control.VZ>-0.05f) smooth_control.VZ=0;
////}


uint16_t JUDGE_fGetRemainEnergy()
{
	return power_heat_data.chassis_power;
}

bool_t JUDGE_sGetDataState()
{
	return 1;
}

void Chassis_Power_Limit(void)
{	
	/*********************�洫�㷨*************************/
	float    kLimit = 0;//��������ϵ��
	float    chassis_totaloutput = 0;//ͳ�����������
	float    Joule_Residue = 0;//ʣ�ཹ����������
	int16_t  judgDataCorrect = 0;//����ϵͳ�����Ƿ����	
	static int32_t judgDataError_Time = 0;
	
	
	judgDataCorrect = JUDGE_sGetDataState();//����ϵͳ�����Ƿ����
	Joule_Residue = JUDGE_fGetRemainEnergy();//ʣ�ཹ������	
	
	//ͳ�Ƶ��������
	chassis_totaloutput = abs(CAN_M3508[0].current) + abs(CAN_M3508[1].current)
							+ abs(CAN_M3508[2].current) + abs(CAN_M3508[3].current);
	
	if(!judgDataCorrect)//����ϵͳ��Чʱǿ������
	{
		judgDataError_Time++;
		if(judgDataError_Time > 100)
		{
			fTotalCurrentLimit = 9000;//��Ϊ����1/4
		}
	}
	else
	{
		judgDataError_Time = 0;
		//ʣ�ཹ������С,��ʼ�������,����ϵ��Ϊƽ����ϵ
		if(Joule_Residue < WARNING_REMAIN_POWER)
		{
			kLimit = (float)(Joule_Residue / WARNING_REMAIN_POWER)
						* (float)(Joule_Residue / WARNING_REMAIN_POWER);
			
			fTotalCurrentLimit = kLimit * fChasCurrentLimit;
		}
		else   //���������ָ���һ����ֵ
		{
			fTotalCurrentLimit = fChasCurrentLimit;
		}
	}

	//���̸�����������·���
	if (chassis_totaloutput > fTotalCurrentLimit)
	{
		CAN_M3508[0].current = (int16_t)(CAN_M3508[0].current / chassis_totaloutput * fTotalCurrentLimit);
		CAN_M3508[1].current = (int16_t)(CAN_M3508[0].current / chassis_totaloutput * fTotalCurrentLimit);
		CAN_M3508[2].current = (int16_t)(CAN_M3508[0].current / chassis_totaloutput * fTotalCurrentLimit);
		CAN_M3508[3].current = (int16_t)(CAN_M3508[0].current / chassis_totaloutput * fTotalCurrentLimit);	
	}
}


