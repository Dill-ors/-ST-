#ifndef __LED_H
#define __LED_H
#include "sys.h"


#define LEDtimer PBout(0)  //LED��˸����

void LED_Init(void);//��ʼ��		 				    
void led_on(void);
void led_off(void);
void erro(void);
#endif
