#include "stm32f4xx.h"
#include "led.h"
#include "usart.h"
#include "delay.h"
#include "pwm.h"
#include "move.h"
#include "Gyro.h"
#include "timer.h"
#include "MS5837.h"
#include "hc05.h"
#include "ai.h"

// 陀螺仪角度
extern float angle_z;
extern float angle_y;
extern float angle_x;
extern int32_t Pressure,atmospheric;
int StarPressure = 6;

//深度传感器
float real_depth = 0;

int main(void)
{
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	LED_Init();
	delay_init(168);	
	
	openmv_uart_init(115200);
	gyro_uart_init(115200);
	hc_uart_init(115200);
	
	depth_init();
	MOTOR_PWM_Init();//推进器初始化
	readAI_Init();
	get_star_ange();
	Pid_Init();
	
	while (1){
		get_depth();
		if (Pressure > StarPressure)break;
		
	r1_motor(40);
			
	l1_motor(50);
		
	r3_motor(50);
		
	l3_motor(40);
		
	l2_motor((int)(50));
		
	r2_motor((int)(50));
	delay_ms(1000);
	}
	timer_init();
	

	while(1){
	}
}




