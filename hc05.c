#include "hc05.h"
#include "led.h"
#include "usart.h"
#include "move.h"
#include "MS5837.h"
#include "ai.h"

/*
mode： 				1自主巡检 / 0遥控控制
pid_speed： 		0低速模式 / 1高速模式 
*/
uint8_t mode=1;
uint8_t err_mode;
uint8_t send_atm = 0;				// 发送大气压值，如果为1则进行发送 
int fb_val,lr_val,turn_val,ug_val; 	// 巡检模式, PID挡位，前后速度， 左右平移速度，转向速度，升浅速度。
extern uint32_t atmospheric; 		// 大气压值
extern int32_t Pressure;

// 数据处理 确保传回的数据是正确的 如果回传数据正确则将其赋值给hc_order 否则hc_order保持原来的值
void ControlData_deal(uint8_t data){
	
	uint8_t highNibble,lowNibble; //存放8位数据
	// 对数据进行提取
	highNibble  = (data & 0xE0) >> 5;
	lowNibble = data & 0x1F;
	if(highNibble == 0X00) fb_val = (int)lowNibble-8; 		// 前后值
	if(highNibble == 0X01) turn_val = (int)lowNibble-7; 	// 转向值
	if(highNibble == 0X02) ug_val = (int)lowNibble-8; 		// 升潜值
	if(highNibble == 0X03) lr_val = (int)lowNibble-8; 		// 平移值
	if(highNibble == 0X04) {
		err_mode = (data & 0X10)>>4; 						// PIDdangwe
		mode = data & 0X01;									// PID巡检值
	}
	if(highNibble == 0X07)send_atm = 1;						// 发送大气压标志位
}


// 蓝牙进行控鱼函数
uint8_t fish_Control(void){
	static int r1,r2,r3,l1,l2,l3;	// 电机转速
	
	if (mode == 1) return 0;		// 如果为巡检模式，则退出
	
	// 对机器鱼运动进行结算
	r1 = (int)((fb_val-turn_val-lr_val)*600/8);
	r3 = (int)((fb_val-turn_val+lr_val)*600/8);
	l1 = (int)((fb_val+turn_val+lr_val)*600/8);
	l3 = (int)((fb_val+turn_val-lr_val)*600/8);
	r2 = (int)(ug_val*200/8);
	l2 = (int)(ug_val*200/8);
	
	// 限制最大值  
	r1 = (r1 > 200) ? 200 : (r1 < -200) ? -200 : r1;  
	r3 = (r3 > 200) ? 200 : (r3 < -200) ? -200 : r3;  
	l1 = (l1 > 200) ? 200 : (l1 < -200) ? -200 : l1;  
	l3 = (l3 > 200) ? 200 : (l3 < -200) ? -200 : l3;
	r2 = (r2 > 200) ? 200 : (r2 < -200) ? -200 : r2;
	l2 = (l2 > 200) ? 200 : (l2 < -200) ? -200 : l2;
	
	r1_motor((int)r1);
	r3_motor((int)r3);
	l1_motor((int)l1);
	l3_motor((int)l3);
	r2_motor((int)r2);
	l2_motor((int)l2);
	
	return 0;
	
}

void send_atmospheric(void){ // 发送大气压值数据
	static int message; 
	
	if (send_atm==1){
		// send_atmospheric进行解析，方便发送数据;
		message = atmospheric - 900;
		
		if(message<256 && message>=0){	
			
			USART_SendData(UART4,(uint8_t)message);	
		}
		else erro();	// 不在范围内会发送失败，进行报错处理
		
		send_atm=0;		// 发送之后其值设置为0
	}
}

void send_state(void){ 		// 发送当前气压值以及检测状态
	static uint8_t message,ai_stated;
	if(mode==1){						//只有巡检的时候发送
		send_atmospheric(); 			// 发送气压值
		// 发送当前值
		ai_stated = get_AiState()<<7;
		//printf("ai_stated:%d\r\n",ai_stated);		
		message = (uint8_t) (Pressure+64);
		message = message & 0X7F;
		message = message | ai_stated;
		USART_SendData(UART4,message);
	}
}
