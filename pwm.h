#ifndef _TIMER_H
#define _TIMER_H
#include "sys.h"
#define pwm_arr 20000-1 //�Զ���װ��ֵ
#define pwm_psc 83  //Ԥ��Ƶֵ

//arr�Զ�װ��ֵ pscԤ��Ƶϵ��
//84M/(psc+1)=f Hz ��fΪ����Ƶ�ʣ�û��Ƶ�ʼ���ֵ��1�� ��װ��ֵarr��һ����arr+1������
//����PWMƵ��Ϊ f/(arr+1)

//zkb=zkb_flag/arr

void MOTOR_PWM_Init(void);


#endif

