#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h" 


void openmv_uart_init(u32 bound);  	//USART1
void gyro_uart_init(u32 bound);    	//USART3
void hc_uart_init(u32 bound); 	 	//USART4

void openmv_rx(unsigned char ucData);

#endif


