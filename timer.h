#ifndef _TIMER_H
#define _TIMER_H
#include "sys.h"


//定时中断：tim3 10ms
//定时器溢出时间:Tout=((arr+1)*(psc+1))/Ft.
//Ft:定时器工作频率84Mhz

// 84 000 000 / 840 = 1 000 00
// 1 000 00 / 20 000 = 5 一次进行 

#define timer_arr 10000//自动重装值
#define timer_psc 840-1//时钟预分频数

#define ABS(a) ((a)<0?((a)*(-1)):(a))


void timer_init(void);


#endif


