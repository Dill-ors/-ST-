#ifndef __HC05_H
#define __HC05_H
#include "sys.h" 
 
// �������㺯��
uint8_t fish_Control(void);
void ControlData_deal(uint8_t data);
void send_atmospheric(void); // ���ʹ���ѹֵ
void send_state(void);
#endif
