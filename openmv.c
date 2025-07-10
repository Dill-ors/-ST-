#include "openmv.h"

float ov_angle=0;
float ov_offset=0;//openmvͨ��

void openmv_rx(unsigned char ucData)
{

	static unsigned char angle;
	static unsigned char offset;
	static unsigned char sign;
	static unsigned char ucRxBuffer[250];
	static unsigned char ucRxCnt = 0;	
	static unsigned char ucSum = 0;//У����
	
	ucRxBuffer[ucRxCnt++]=ucData;	//���յ������ݴ��뻺������

	//ucSum=ucData;
	if (ucRxBuffer[0]!=0x42) //����ͷ���ԣ������¿�ʼѰ��0x55����ͷ
	{
		ucRxCnt=0;
		ucSum = 0;//У���������
		return;
	}
	if (ucRxCnt<5) //���ݲ���5�����򷵻�
	{
		ucSum += ucData;                //У��
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
		ucRxCnt=0;//��ջ�����
		ucSum = 0;//У���������
	}

}

