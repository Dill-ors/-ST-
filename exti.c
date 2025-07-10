#include "exti.h"
#include "delay.h" 
#include "stm32f4xx_exti.h"
#include "led.h" 
extern int led_flag;
extern int flag_nano;
//extern int flag_gd;
//�ⲿ�жϳ�ʼ������
//��ʼ��PD0,1,2Ϊ�ж�����.
void extix_Init(void)
{
	NVIC_InitTypeDef   NVIC_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;
	//������Ӧ��IO�ڳ�ʼ��
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//ʹ��SYSCFGʱ��
	
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; //KEY0 KEY1 KEY2��Ӧ����
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//��ͨ����ģʽ
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//����
  GPIO_Init(GPIOE, &GPIO_InitStructure);//��ʼ��GPIOE2,3,4
	
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_0; //KEY0 KEY1 KEY2��Ӧ����
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//��ͨ����ģʽ
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//����
//  GPIO_Init(GPIOE, &GPIO_InitStructure);//��ʼ��GPIOE2,3,4
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource0);//PE2 ���ӵ��ж���0
	
	

//	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource1);//PE1 ���ӵ��ж���1
	/* ����EXTI_Line2,3,4 */
	EXTI_InitStructure.EXTI_Line = EXTI_Line0;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//�ж��¼�
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; 
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;//�ж���ʹ��
  EXTI_Init(&EXTI_InitStructure);//����
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;//�ⲿ�ж�2
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;//��ռ���ȼ�3
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;//�����ȼ�2
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//ʹ���ⲿ�ж�ͨ��
  NVIC_Init(&NVIC_InitStructure);//����

  
//	
//	EXTI_InitStructure.EXTI_Line = EXTI_Line1;//LINE0
//  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//�ж��¼�
//  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; //�����ش��� 
//  EXTI_InitStructure.EXTI_LineCmd = ENABLE;//ʹ��LINE0
//  EXTI_Init(&EXTI_InitStructure);//����
//	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;//�ⲿ�ж�3
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;//��ռ���ȼ�2
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;//�����ȼ�2
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//ʹ���ⲿ�ж�ͨ��
//  NVIC_Init(&NVIC_InitStructure);//����
}
void EXTI0_IRQHandler(void)
{   
//	delay_ms(10);	//����
	if(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_0)==1&&led_flag==0)	  
	{				 
		LED1=1; 
		led_flag=1;
	}	
	else if(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_0)==0&&led_flag==1)	  
	{				 
		LED1=0; 
		led_flag=0;
	}	

	 EXTI_ClearITPendingBit(EXTI_Line0);//���LINE2�ϵ��жϱ�־λ 
}


//void EXTI1_IRQHandler(void)
//{
//	delay_ms(10);	//����
//	if(flag_gd==0)
//		flag_gd=1;
//	EXTI_ClearITPendingBit(EXTI_Line1);  //���LINE3�ϵ��жϱ�־λ  
//}
