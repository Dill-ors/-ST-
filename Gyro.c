#include "Gyro.h"
#include "sys.h"
#include "string.h"

float level_angle_y=0;

struct STime		stcTime;
struct SAcc 		stcAcc;
struct SGyro 		stcGyro;
struct SAngle 		stcAngle;
struct SMag 		stcMag;
struct SDStatus 	stcDStatus;
struct SPress 		stcPress;
struct SLonLat 		stcLonLat;
struct SGPSV 		stcGPSV;
struct SQ       	stcQ;

float angle_z=0;
float angle_y=0;
float angle_x=0;
float velocity_x = 0;
float velocity_y = 0;
float velocity_z = 0;
//���ٶ�
float a_x = 0;
float a_y = 0;
float a_x_last = 0;
float a_y_last = 0;
//�ٶ�
float v_x = 0;
float v_y = 0;
float v_x_last = 0;
float v_y_last = 0;
//λ��
float x_x = 0;
float x_y = 0;

//ƫ����
float Angle_z_init = 0;
float Angle_z = 0;

//��̬У��ʹ�ܱ�־
u8 flag_posture_control = 0;

float speed_inc = 0, speed_inc_exit = 0;

u16 speed_inc_switch = 0;

///////////////////////////////////////////
//��������Gyro_Uart_Rx
//���ܣ����������Ǵ�������
//ʹ�÷�����
//CopeSerial2Data((unsigned char)USART2->DR);
///////////////////////////////////////////
void Gyro_Uart_Rx(unsigned char ucData)
{
	static unsigned char ucRxBuffer[250];
	static unsigned char ucRxCnt = 0;	
	static unsigned char ucSum = 0;//У����
	

	ucRxBuffer[ucRxCnt++]=ucData;	//���յ������ݴ��뻺������

	if (ucRxBuffer[0]!=0x55) //����ͷ���ԣ������¿�ʼѰ��0x55����ͷ
	{
		
		
		ucRxCnt=0;
		ucSum = 0;//У���������
		return;
	}
	if (ucRxCnt<11) //���ݲ���11�����򷵻�
	{
		
		ucSum += ucData;                //У��
		return;
	}
	
	else
	{
		if((ucSum & 0xFF) == ucData)
		{
			switch(ucRxBuffer[1])//�ж��������������ݣ�Ȼ���俽������Ӧ�Ľṹ���У���Щ���ݰ���Ҫͨ����λ���򿪶�Ӧ������󣬲��ܽ��յ�������ݰ�������
			{
				case 0x50:	memcpy(&stcTime,&ucRxBuffer[2],8);break;//memcpyΪ�������Դ����ڴ濽��������������"string.h"�������ջ��������ַ����������ݽṹ�����棬�Ӷ�ʵ�����ݵĽ�����
				case 0x51:	memcpy(&stcAcc,&ucRxBuffer[2],8);break;
				case 0x52:	memcpy(&stcGyro,&ucRxBuffer[2],8);break;
				case 0x53:	memcpy(&stcAngle,&ucRxBuffer[2],8);break;
				case 0x54:	memcpy(&stcMag,&ucRxBuffer[2],8);break;
				case 0x55:	memcpy(&stcDStatus,&ucRxBuffer[2],8);break;
				case 0x56:	memcpy(&stcPress,&ucRxBuffer[2],8);break;
				case 0x57:	memcpy(&stcLonLat,&ucRxBuffer[2],8);break;
				case 0x58:	memcpy(&stcGPSV,&ucRxBuffer[2],8);break;
				case 0x59:	memcpy(&stcQ,&ucRxBuffer[2],8);break;
			}
			
			angle_z = (float)stcAngle.Angle[2]/32768*180;
			angle_x = (float)stcAngle.Angle[0]/32768*180;
			angle_y = (float)stcAngle.Angle[1]/32768*180;
			velocity_x=(float)stcGyro.w[0]/32768*2000;
			velocity_y=(float)stcGyro.w[1]/32768*2000;
			velocity_z=(float)stcGyro.w[2]/32768*2000;
				
		}
		ucRxCnt=0;//��ջ�����
		ucSum = 0;//У���	����
	}
}

///////////////////////////////////////////
//��������Gyro_Distance_Calculate
//���ܣ��Լ��ٶ��Ʋ�λ��
//ʹ�÷�����
//
///////////////////////////////////////////
void Gyro_Displacement_Calculate(float time_cal_ms)
{
	a_x_last = a_x;
	a_y_last = a_y;
	if(((float)stcAcc.a[0]/32768*16*9.8f >= 0.1f) || ((float)stcAcc.a[0]/32768*16*9.8f <= -0.1f))
		a_x = (float)stcAcc.a[0]/32768*16*9.8f;
	else
		a_x = 0;
	
	if(((float)stcAcc.a[1]/32768*16*9.8f >= 0.1f) || ((float)stcAcc.a[1]/32768*16*9.8f <= -0.1f))
		a_y = (float)stcAcc.a[1]/32768*16*9.8f;
	else
		a_y = 0;
	
	v_x_last = v_x;
	v_y_last = v_y;
	
	v_x += (a_x + a_x_last)/2.0f*time_cal_ms/10.0f;
	v_y += (a_y + a_y_last)/2.0f*time_cal_ms/10.0f;
	
	x_x += (v_x + v_x_last)/2.0f*time_cal_ms/1000.0f;
	x_y += (v_y + v_y_last)/2.0f*time_cal_ms/1000.0f;
	
}


// ��ȡ���״̬�ĽǶ�ֵ
void get_star_ange(void){
	
	level_angle_y = angle_y;
}
