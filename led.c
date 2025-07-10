#include "led.h" 
#include "delay.h"


void LED_Init(void)
{    	 
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能GPIOF时钟

	//GPIOF9,F10初始化设置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化

	GPIO_SetBits(GPIOB,GPIO_Pin_0);//设置高，灯灭
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);//使能GPIOF时钟

}

void led_on(void){
	GPIO_ResetBits(GPIOB,GPIO_Pin_0);//设置低，灯亮
}
void led_off(void){
	GPIO_SetBits(GPIOB,GPIO_Pin_0);//设置低，灯亮
}

void erro(void){ 	// 程序错误
	
	while(1){
		led_on();
		delay_ms(20);
		led_off();
		delay_ms(20);
	}
}

