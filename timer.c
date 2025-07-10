#include "timer.h"
#include "led.h"
#include "Gyro.h"
#include "MS5837.h"
#include "move.h"
#include "delay.h"
#include "hc05.h" 

extern float angle_z,angle_x,angle_y;	// 陀螺仪角度
extern float ov_angle,ov_offset;
extern uint8_t mode; 					// 1遥控控制 0自动巡检
extern int32_t Pressure; 				//实际深度
extern uint8_t mode, pid_speed, fb_val, lr_val, turn_val, ug_val; // 巡检模式, PID挡位，前后速度， 左右平移速度，转向速度，升浅速度。
//通用定时器3中断初始化
//arr：自动重装值。
//psc：时钟预分频数
//定时器溢出时间计算方法:Tout=((arr+1)*(psc+1))/Ft us.
//Ft=定时器工作频率,单位:Mhz
void timer_init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  ///使能TIM3时钟
	
	TIM_TimeBaseInitStructure.TIM_Period = timer_arr; 	//自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler=timer_psc;  //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);//初始化TIM3
	
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE); //允许定时器3更新中断
	TIM_Cmd(TIM3,ENABLE); //使能定时器3
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM3_IRQn; //定时器3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x0; //抢占优先级2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x0; //子优先级2
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}

	
void TIM3_IRQHandler(void)
{

	if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET)
	{ 
		
		get_depth();		// 获取水深

		send_state();
		//printf("%d\r\n",(int)(Pressure));
		
		//printf("fb_val:%d,lr_val:%d,turn_val:%d,ug_val:%d,pid_speed:%d,mode:%d\r\n",fb_val,lr_val,turn_val,ug_val,pid_speed,mode);
		if (mode==0){

			fish_Control();
		}
		else{
			
			depth_balance(); //角度以及深度调整
			tracting();
		}
		 
		LEDtimer=!LEDtimer;

		
	}
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);  //清除中断标志位
}
