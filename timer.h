#ifndef _TIMER_H
#define _TIMER_H
#include "sys.h"


//��ʱ�жϣ�tim3 10ms
//��ʱ�����ʱ��:Tout=((arr+1)*(psc+1))/Ft.
//Ft:��ʱ������Ƶ��84Mhz

// 84 000 000 / 840 = 1 000 00
// 1 000 00 / 20 000 = 5 һ�ν��� 

#define timer_arr 10000//�Զ���װֵ
#define timer_psc 840-1//ʱ��Ԥ��Ƶ��

#define ABS(a) ((a)<0?((a)*(-1)):(a))


void timer_init(void);


#endif


