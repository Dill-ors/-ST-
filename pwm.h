#ifndef _TIMER_H
#define _TIMER_H
#include "sys.h"
#define pwm_arr 20000-1 //自动重装载值
#define pwm_psc 83  //预分频值

//arr自动装载值 psc预分频系数
//84M/(psc+1)=f Hz ；f为计数频率，没个频率计数值加1； 重装载值arr，一共计arr+1个数字
//所以PWM频率为 f/(arr+1)

//zkb=zkb_flag/arr

void MOTOR_PWM_Init(void);


#endif

