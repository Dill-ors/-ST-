#include "led.h" 
#include "delay.h"


void LED_Init(void)
{    	 
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//ʹ��GPIOFʱ��

	//GPIOF9,F10��ʼ������
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
	GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��

	GPIO_SetBits(GPIOB,GPIO_Pin_0);//���øߣ�����
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);//ʹ��GPIOFʱ��

}

void led_on(void){
	GPIO_ResetBits(GPIOB,GPIO_Pin_0);//���õͣ�����
}
void led_off(void){
	GPIO_SetBits(GPIOB,GPIO_Pin_0);//���õͣ�����
}

void erro(void){ 	// �������
	
	while(1){
		led_on();
		delay_ms(20);
		led_off();
		delay_ms(20);
	}
}

