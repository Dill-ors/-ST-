#include "move.h"
#include "usart.h"
#include "sys.h"
#include "Gyro.h"
#include "MS5837.h"
#include "openmv.h"

extern float level_angle_y;	//ˮƽʱ�ĽǶ�
extern float angle_y;  // ������ʵ�ʽǶ�
extern int32_t Pressure; //ʵ�����
extern float ov_angle, ov_offset;
extern uint8_t err_mode;

float corner_flag_right, corner_flag_left;
float target_Pressure;   // 4-22֮�� Խ����Խ��
float speed_behind_coner,speed_front_coner;   //ʶ�𵽹սǵ��ٶ�
float ov_angle_errnom,ov_offset_errnom,ov_angle_err_cornerright,ov_offset_err_cornerright; // �ս�ƫ��ֵ����

int CornerLeftOVOffset_err,CornerLeftOVAngle_err,CornerRightOVOffset_err,CornerRightOVAngle_err,LineOVOffset_err,LineOVAngle_err;
Pid_t Pid_Gyro;// ������PID
Pid_t Pid_Depth;// ���PID
Pid_t Pid_ovAngle;
Pid_t Pid_ovAngle_line;
Pid_t Pid_ovAngle_corner_right; 
Pid_t Pid_ovAngle_corner_left; // ��ʱ���ǶԵ�
Pid_t Pid_ovOffset;
float speed_front;   // ǰ����������
float speed_behind;	// ������������
void pid_cal(Pid_t *pid, float _usMeasure, float _usTarget){
	
	float dError = 0;
    float error  = 0;
    float tempP  = 0;
    float tempD  = 0;
	
	
	error = _usTarget - _usMeasure;
	tempP = pid -> P * error;
	
	dError = error - pid->PrevError;
	tempD = pid->D * dError;
	pid -> PrevError = error;
	
	pid->PID = tempP + tempD;
}


void Pid_Init(void){

	Pid_Depth.P = 8 ;	//  ���� 5
	Pid_Depth.I = 0.5; 
	Pid_Depth.D = 3;	// ΢��
	target_Pressure = 7;
	
	
	Pid_Gyro.P = 1.3;
	Pid_Gyro.D = 0.8;

	Pid_ovAngle_corner_left.P = 2.5;
	Pid_ovAngle_corner_left.D = 1.2;
	Pid_ovAngle_corner_right.P = 2.5;
	Pid_ovAngle_corner_right.D = 1.2;
	
	Pid_ovAngle_line.P = 1.5; 
	Pid_ovAngle_line.D = 0.6;
	
	Pid_ovOffset.P = 1.7;
	Pid_ovOffset.D = 1;
	
	speed_front = 50;   // ǰ���������ӵĳ��ٶ�
	speed_behind = 35;	// �����������ӵĳ��ٶ�
	speed_front_coner = 20;
	speed_behind_coner = 20;
	
	corner_flag_right = 20; // ���Ƕȴ������ֵʱ���ж�Ϊ�ս�
	corner_flag_left = 20; // ���Ƕȴ������ֵʱ���ж�Ϊ�ս�

	
	
	CornerLeftOVOffset_err = -25;
	CornerLeftOVAngle_err = -10;
	CornerRightOVOffset_err = 0;
	CornerRightOVAngle_err = 0;
	LineOVOffset_err = -8;
	LineOVAngle_err = -2;
	
	
	Pid_Depth.PrevError = 0;
	Pid_Depth.SumError = 0;
	Pid_Gyro.PrevError = 0;
	Pid_Gyro.SumError = 0;
	Pid_ovAngle_line.PrevError = 0;
	Pid_ovAngle_line.SumError = 0;
}

void depth_balance(void)//������ ���
{

	float val_r2,val_l2; //���ҵ����ֵ
	//���PID
	float Depth_dError = 0;
    float Depth_error  = 0;
    float Depth_tempP  = 0;
	  float Depth_tempI  = 0;
    float Depth_tempD  = 0;
    float Depth_tempPID = 0;

	
	//������PID	
	float Gyro_dError = 0;
    float Gyro_error  = 0;
    float Gyro_tempP  = 0;
    float Gyro_tempD  = 0;
    float Gyro_tempPID = 0;
	
	//���PID����
    Depth_error = target_Pressure - (float)Pressure;  /*���㵱ǰ���*/
  	Depth_tempP = Pid_Depth.P * Depth_error;/*����P���ֵ*/
	
     
	   Pid_Depth.SumError += Depth_error;  // �ۻ����
     Depth_tempI = Pid_Depth.I * Pid_Depth.SumError;  // ����I���ֵ
	   if(err_mode==1){Pid_Depth.SumError=0;}
	
	
    Depth_dError = Depth_error - Pid_Depth.PrevError;/*΢�����*/
    Depth_tempD  = Pid_Depth.D * Depth_dError;/*����D���ֵ*/ 
		
    Pid_Depth.PrevError = Depth_error ;
		
	Depth_tempPID = Depth_tempP + Depth_tempI + Depth_tempD;
	
	
	//������PID
    Gyro_error = level_angle_y - (float)angle_y;  /*���㵱ǰ���*/
	Gyro_tempP = Pid_Gyro.P * Gyro_error;/*����P���ֵ*/
	
	Gyro_dError = Gyro_error - Pid_Gyro.PrevError;/*΢�����*/
    Gyro_tempD  = Pid_Gyro.D * Gyro_dError;/*����D���ֵ*/ 
    Pid_Gyro.PrevError = Gyro_error ;
	Gyro_tempPID = Gyro_tempP + Gyro_tempD;
	//printf("level_angle_y:%f angle_y: %f Gyro_tempPID:%f\r\n",(float)level_angle_y,(float)angle_y,Gyro_tempPID);
	
	val_l2 = -1*Depth_tempPID + Gyro_tempPID;
	val_r2 = -1*Depth_tempPID - Gyro_tempPID; 
	
	//�޷�
	val_l2 = val_l2<200 ? val_l2:200;
	val_l2 = val_l2>-200 ? val_l2:-200;
	val_r2 = val_r2<200 ? val_r2:200;
	val_r2 = val_r2>-200 ? val_r2:-200;
	
	l2_motor((int)(1500));
	r2_motor((int)(1500));

}

void tracting(void){
	
	float val_r1,val_r3,val_l1,val_l3;
	// ƫ��PID
	float Off_dError = 0;
 
	float Off_error  = 0;
    float Off_tempP  = 0;
    float Off_tempD  = 0;
    float Off_tempPID = 0;
	
	// �Ƕ�PID
	float Angle_dError = 0;
    float Angle_error  = 0;
    float Angle_tempP  = 0;
    float Angle_tempD  = 0;
    float Angle_tempPID = 0;
	float SPEED_FRONT = 0;
	float SPEED_BEHIND = 0;
	// �Ƕȼ�С ��ͷ��ƫ
	// ƫ�ü�С ������ƫ

	if( ov_angle<(-1*corner_flag_right)  ){  // ��ʱ��սǵ�ʱ��OK
		
		Pid_ovAngle.P =Pid_ovAngle_corner_left.P;
		Pid_ovAngle.D =Pid_ovAngle_corner_left.D;
		SPEED_FRONT = speed_front_coner;
		SPEED_BEHIND = speed_behind_coner;
		ov_angle = ov_angle + CornerRightOVAngle_err;			// ���ڽǶ�
		ov_offset = ov_offset + CornerRightOVOffset_err;			// ����ƫ��
	}	
	else if(ov_angle>corner_flag_left){		//˳ʱ��սǵ�ʱ����
		
		Pid_ovAngle.P =Pid_ovAngle_corner_right.P;
		Pid_ovAngle.D =Pid_ovAngle_corner_right.D;
		SPEED_FRONT = speed_front_coner;
		SPEED_BEHIND = speed_behind_coner;
		ov_angle = ov_angle + CornerLeftOVAngle_err;			// ���ڽǶ�
		ov_offset = ov_offset + CornerLeftOVOffset_err;			// ����ƫ��
	}
	else{									// ֱ��
		
		Pid_ovAngle.P =Pid_ovAngle_line.P;//2.5//1.3
		Pid_ovAngle.D =Pid_ovAngle_line.D;//0.5
		SPEED_FRONT = speed_front;
		SPEED_BEHIND = speed_behind;
		ov_angle = ov_angle + LineOVAngle_err;              // ���ڽǶ�
		ov_offset = ov_offset + LineOVOffset_err;			// ����ƫ��
	}
	
	// ƫ��PID����
	Off_error = ov_offset;
	Off_tempP = Pid_ovOffset.P * Off_error;
	
	Off_dError = Pid_ovOffset.PrevError - Off_error ;/*΢�����*/
    Off_tempD  = Pid_ovOffset.D * Off_dError;/*����D���ֵ*/ 
	Pid_ovOffset.PrevError = Off_error ;

	Off_tempPID = Off_tempP + Off_tempD;
	Off_tempPID = Off_tempPID<100?Off_tempPID:100;
	Off_tempPID = Off_tempPID>-100?Off_tempPID:-100; //��ֵ
	
	// �Ƕ�PID����
	Angle_error = ov_angle;
	Angle_tempP = Pid_ovAngle.P * Angle_error;
	
	Angle_dError = Pid_ovAngle.PrevError - Angle_error ;/*΢�����*/
    Angle_tempD  = Pid_ovAngle.D * Angle_dError;/*����D���ֵ*/ 
	Pid_ovAngle.PrevError = Angle_error ;

	Angle_tempPID = Angle_tempP + Angle_tempD;
	Angle_tempPID = Angle_tempPID<100?Angle_tempPID:100;
	Angle_tempPID = Angle_tempPID>-100?Angle_tempPID:-100; //��ֵ
	
	
	// ������м���
	val_r1 = SPEED_FRONT - ( Off_tempPID<0? Off_tempPID:0 ) + 0.2f*Angle_tempPID;
	val_l1 = SPEED_FRONT + ( Off_tempPID>0? Off_tempPID:0 ) + 0.2f*Angle_tempPID;
	
	val_r3 = SPEED_BEHIND - Angle_tempPID;
	val_l3 = SPEED_BEHIND + Angle_tempPID;

	r1_motor(1450);	
	l1_motor(1500);
	r3_motor(1300);
	l3_motor(1450);
}



void r1_motor(int pwm){
	
	TIM_SetCompare2(TIM5,1500-1-pwm);
}
void r2_motor(int pwm){
	
	TIM_SetCompare2(TIM4,1500-1-pwm);
}
void r3_motor(int pwm){
	
	TIM_SetCompare4(TIM5,1500-1-pwm);
}
void l1_motor(int pwm){
	
	TIM_SetCompare3(TIM5,1500-1+pwm);
}

void l2_motor(int pwm){
	
	TIM_SetCompare3(TIM4,1500-1-pwm);
}
void l3_motor(int pwm){
	
	TIM_SetCompare1(TIM4,1500-1+pwm);
}



