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
//加速度
float a_x = 0;
float a_y = 0;
float a_x_last = 0;
float a_y_last = 0;
//速度
float v_x = 0;
float v_y = 0;
float v_x_last = 0;
float v_y_last = 0;
//位移
float x_x = 0;
float x_y = 0;

//偏航角
float Angle_z_init = 0;
float Angle_z = 0;

//姿态校正使能标志
u8 flag_posture_control = 0;

float speed_inc = 0, speed_inc_exit = 0;

u16 speed_inc_switch = 0;

///////////////////////////////////////////
//函数名：Gyro_Uart_Rx
//功能：接收陀螺仪串口数据
//使用方法：
//CopeSerial2Data((unsigned char)USART2->DR);
///////////////////////////////////////////
void Gyro_Uart_Rx(unsigned char ucData)
{
	static unsigned char ucRxBuffer[250];
	static unsigned char ucRxCnt = 0;	
	static unsigned char ucSum = 0;//校验用
	

	ucRxBuffer[ucRxCnt++]=ucData;	//将收到的数据存入缓冲区中

	if (ucRxBuffer[0]!=0x55) //数据头不对，则重新开始寻找0x55数据头
	{
		
		
		ucRxCnt=0;
		ucSum = 0;//校验变量置零
		return;
	}
	if (ucRxCnt<11) //数据不满11个，则返回
	{
		
		ucSum += ucData;                //校验
		return;
	}
	
	else
	{
		if((ucSum & 0xFF) == ucData)
		{
			switch(ucRxBuffer[1])//判断数据是哪种数据，然后将其拷贝到对应的结构体中，有些数据包需要通过上位机打开对应的输出后，才能接收到这个数据包的数据
			{
				case 0x50:	memcpy(&stcTime,&ucRxBuffer[2],8);break;//memcpy为编译器自带的内存拷贝函数，需引用"string.h"，将接收缓冲区的字符拷贝到数据结构体里面，从而实现数据的解析。
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
		ucRxCnt=0;//清空缓存区
		ucSum = 0;//校验变	置零
	}
}

///////////////////////////////////////////
//函数名：Gyro_Distance_Calculate
//功能：以加速度推测位移
//使用方法：
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


// 获取最初状态的角度值
void get_star_ange(void){
	
	level_angle_y = angle_y;
}
