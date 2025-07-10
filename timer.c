#include "timer.h"
#include "led.h"
#include "Gyro.h"
#include "MS5837.h"
#include "move.h"
#include "delay.h"
#include "hc05.h" 

extern float angle_z,angle_x,angle_y;	// �����ǽǶ�
extern float ov_angle,ov_offset;
extern uint8_t mode; 					// 1ң�ؿ��� 0�Զ�Ѳ��
extern int32_t Pressure; 				//ʵ�����
extern uint8_t mode, pid_speed, fb_val, lr_val, turn_val, ug_val; // Ѳ��ģʽ, PID��λ��ǰ���ٶȣ� ����ƽ���ٶȣ�ת���ٶȣ���ǳ�ٶȡ�
//ͨ�ö�ʱ��3�жϳ�ʼ��
//arr���Զ���װֵ��
//psc��ʱ��Ԥ��Ƶ��
//��ʱ�����ʱ����㷽��:Tout=((arr+1)*(psc+1))/Ft us.
//Ft=��ʱ������Ƶ��,��λ:Mhz
void timer_init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  ///ʹ��TIM3ʱ��
	
	TIM_TimeBaseInitStructure.TIM_Period = timer_arr; 	//�Զ���װ��ֵ
	TIM_TimeBaseInitStructure.TIM_Prescaler=timer_psc;  //��ʱ����Ƶ
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);//��ʼ��TIM3
	
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE); //����ʱ��3�����ж�
	TIM_Cmd(TIM3,ENABLE); //ʹ�ܶ�ʱ��3
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM3_IRQn; //��ʱ��3�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x0; //��ռ���ȼ�2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x0; //�����ȼ�2
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}

	
void TIM3_IRQHandler(void)
{

	if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET)
	{ 
		
		get_depth();		// ��ȡˮ��

		send_state();
		//printf("%d\r\n",(int)(Pressure));
		
		//printf("fb_val:%d,lr_val:%d,turn_val:%d,ug_val:%d,pid_speed:%d,mode:%d\r\n",fb_val,lr_val,turn_val,ug_val,pid_speed,mode);
		if (mode==0){

			fish_Control();
		}
		else{
			
			depth_balance(); //�Ƕ��Լ���ȵ���
			tracting();
		}
		 
		LEDtimer=!LEDtimer;

		
	}
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);  //����жϱ�־λ
}
