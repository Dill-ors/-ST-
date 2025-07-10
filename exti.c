#include "exti.h"
#include "delay.h" 
#include "stm32f4xx_exti.h"
#include "led.h" 
extern int led_flag;
extern int flag_nano;
//extern int flag_gd;
//外部中断初始化程序
//初始化PD0,1,2为中断输入.
void extix_Init(void)
{
	NVIC_InitTypeDef   NVIC_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;
	//按键对应的IO口初始化
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//使能SYSCFG时钟
	
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; //KEY0 KEY1 KEY2对应引脚
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//普通输入模式
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//上拉
  GPIO_Init(GPIOE, &GPIO_InitStructure);//初始化GPIOE2,3,4
	
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_0; //KEY0 KEY1 KEY2对应引脚
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//普通输入模式
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//下拉
//  GPIO_Init(GPIOE, &GPIO_InitStructure);//初始化GPIOE2,3,4
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource0);//PE2 连接到中断线0
	
	

//	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource1);//PE1 连接到中断线1
	/* 配置EXTI_Line2,3,4 */
	EXTI_InitStructure.EXTI_Line = EXTI_Line0;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//中断事件
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; 
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;//中断线使能
  EXTI_Init(&EXTI_InitStructure);//配置
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;//外部中断2
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;//抢占优先级3
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;//子优先级2
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能外部中断通道
  NVIC_Init(&NVIC_InitStructure);//配置

  
//	
//	EXTI_InitStructure.EXTI_Line = EXTI_Line1;//LINE0
//  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//中断事件
//  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; //上升沿触发 
//  EXTI_InitStructure.EXTI_LineCmd = ENABLE;//使能LINE0
//  EXTI_Init(&EXTI_InitStructure);//配置
//	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;//外部中断3
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;//抢占优先级2
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;//子优先级2
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能外部中断通道
//  NVIC_Init(&NVIC_InitStructure);//配置
}
void EXTI0_IRQHandler(void)
{   
//	delay_ms(10);	//消抖
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

	 EXTI_ClearITPendingBit(EXTI_Line0);//清除LINE2上的中断标志位 
}


//void EXTI1_IRQHandler(void)
//{
//	delay_ms(10);	//消抖
//	if(flag_gd==0)
//		flag_gd=1;
//	EXTI_ClearITPendingBit(EXTI_Line1);  //清除LINE3上的中断标志位  
//}
