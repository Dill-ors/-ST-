#include "openmv.h"

float ov_angle=0;
float ov_offset=0;//openmv通信

void openmv_rx(unsigned char ucData)
{

	static unsigned char angle;
	static unsigned char offset;
	static unsigned char sign;
	static unsigned char ucRxBuffer[250];
	static unsigned char ucRxCnt = 0;	
	static unsigned char ucSum = 0;//校验用
	
	ucRxBuffer[ucRxCnt++]=ucData;	//将收到的数据存入缓冲区中

	//ucSum=ucData;
	if (ucRxBuffer[0]!=0x42) //数据头不对，则重新开始寻找0x55数据头
	{
		ucRxCnt=0;
		ucSum = 0;//校验变量置零
		return;
	}
	if (ucRxCnt<5) //数据不满5个，则返回
	{
		ucSum += ucData;                //校验
		return;
	}
	else if(ucRxCnt==5)//
	{
		if((ucSum & 0xff) == ucData)
		{
			sign=ucRxBuffer[1];
			angle=ucRxBuffer[2]-10;
			offset=ucRxBuffer[3]-10;
			
			if(sign==10){ov_angle=(float)angle;ov_offset=(float)offset;}
			else if(sign==11){ov_angle=(float)angle;ov_offset=-(float)offset;}
			else if(sign==12){ov_angle=-(float)angle;ov_offset=(float)offset;}
			else if(sign==13){ov_angle=-(float)angle;ov_offset=-(float)offset;}	
			
		
		}
		ucRxCnt=0;//清空缓存区
		ucSum = 0;//校验变量置零
	}

}

