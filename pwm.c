#include "pwm.h"
#include "usart.h"
#include "delay.h"

//84M/(psc+1)=f Hz ��fΪ����Ƶ�ʣ�û��Ƶ�ʼ���ֵ��1�� ��װ��ֵarr��һ����arr+1������
//����PWMƵ��Ϊ f/(arr+1)
// ��ʼ��PWM����Ҫ��ʼ���ܶණ�� time4 5
//time4 CH123  PD12 PD13 PD14
//time5 CH234  PA1 2 3
TIM_OCInitTypeDef  TIM_OCInitStructure;
GPIO_InitTypeDef GPIO_InitStructure;
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

void MOTOR_PWM_Init(void)//arr�Զ�װ��ֵ pscԤ��Ƶϵ��
{		 					 

	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);  	//TIM4ʱ��ʹ��    
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);  	//TIM5ʱ��ʹ��    
	
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 	//ʹ��GPIO��ʱ�� ������ʹ�ܵ�GPIOA
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE); 	//ʹ��GPIO��ʱ�� ������ʹ�ܵ�GPIOD
	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource12,GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource13,GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource14,GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_TIM5);
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_TIM5);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_TIM5); // gpio���Ÿ���
	
	//time4
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //����
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14;           //GPIOD12 13 14
	GPIO_Init(GPIOD,&GPIO_InitStructure);              //��ʼ��PB0��PB1
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;;           //GPIOA1 2 3
	GPIO_Init(GPIOA,&GPIO_InitStructure);              //��ʼ��PA1 2 3
	
	
	TIM_TimeBaseStructure.TIM_Prescaler=pwm_psc;  //��ʱ����Ƶ��ԭʱ��Ϊ84Mhz  
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period=pwm_arr;   //�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);//��ʼ����ʱ��4
	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseStructure);//��ʼ����ʱ��4
	
	//��ʼ��TIM14 Channel1 PWMģʽ	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ը� ��Чֵ��
	TIM_OCInitStructure.TIM_Pulse=1500 - 1;//����ռ�ձ�
	
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������
	
	TIM_OC2Init(TIM5, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������
	TIM_OC3Init(TIM5, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������
	TIM_OC4Init(TIM5, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������
	
	
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);  //ʹ��TIM4��CCR1��ͨ��һ���ϵ�Ԥװ�ؼĴ���
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);  //ʹ��TIM4��CCR2��ͨ�������ϵ�Ԥװ�ؼĴ���
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);  //ʹ��TIM4��CCR3��ͨ�������ϵ�Ԥװ�ؼĴ���
	
	TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable);  //ʹ��TIM5��CCR2��ͨ�������ϵ�Ԥװ�ؼĴ���
	TIM_OC3PreloadConfig(TIM5, TIM_OCPreload_Enable);  //ʹ��TIM5��CCR3��ͨ�������ϵ�Ԥװ�ؼĴ���
	TIM_OC4PreloadConfig(TIM5, TIM_OCPreload_Enable);  //ʹ��TIM5��CCR4��ͨ���ģ��ϵ�Ԥװ�ؼĴ���
	
	TIM_ARRPreloadConfig(TIM4,ENABLE);//ARPEʹ�� 
	TIM_ARRPreloadConfig(TIM5,ENABLE);//ARPEʹ�� 
	TIM_Cmd(TIM4, ENABLE);  //ʹ��TIM3			  
	TIM_Cmd(TIM5, ENABLE);  //ʹ��TIM3			  
	
	TIM_CtrlPWMOutputs(TIM4,ENABLE);   //PWMʹ�����	
	TIM_CtrlPWMOutputs(TIM5,ENABLE);   //PWMʹ�����	
	
	delay_s(3);
	
}  

