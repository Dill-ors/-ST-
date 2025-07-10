#ifndef __HC05_H
#define __HC05_H
#include "sys.h" 
 
// 蓝牙控鱼函数
uint8_t fish_Control(void);
void ControlData_deal(uint8_t data);
void send_atmospheric(void); // 发送大气压值
void send_state(void);
#endif
