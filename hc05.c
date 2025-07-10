#include "hc05.h"
#include "led.h"
#include "usart.h"
#include "move.h"
#include "MS5837.h"
#include "ai.h"

/*
mode�� 				1����Ѳ�� / 0ң�ؿ���
pid_speed�� 		0����ģʽ / 1����ģʽ 
*/
uint8_t mode=1;
uint8_t err_mode;
uint8_t send_atm = 0;				// ���ʹ���ѹֵ�����Ϊ1����з��� 
int fb_val,lr_val,turn_val,ug_val; 	// Ѳ��ģʽ, PID��λ��ǰ���ٶȣ� ����ƽ���ٶȣ�ת���ٶȣ���ǳ�ٶȡ�
extern uint32_t atmospheric; 		// ����ѹֵ
extern int32_t Pressure;

// ���ݴ��� ȷ�����ص���������ȷ�� ����ش�������ȷ���丳ֵ��hc_order ����hc_order����ԭ����ֵ
void ControlData_deal(uint8_t data){
	
	uint8_t highNibble,lowNibble; //���8λ����
	// �����ݽ�����ȡ
	highNibble  = (data & 0xE0) >> 5;
	lowNibble = data & 0x1F;
	if(highNibble == 0X00) fb_val = (int)lowNibble-8; 		// ǰ��ֵ
	if(highNibble == 0X01) turn_val = (int)lowNibble-7; 	// ת��ֵ
	if(highNibble == 0X02) ug_val = (int)lowNibble-8; 		// ��Ǳֵ
	if(highNibble == 0X03) lr_val = (int)lowNibble-8; 		// ƽ��ֵ
	if(highNibble == 0X04) {
		err_mode = (data & 0X10)>>4; 						// PIDdangwe
		mode = data & 0X01;									// PIDѲ��ֵ
	}
	if(highNibble == 0X07)send_atm = 1;						// ���ʹ���ѹ��־λ
}


// �������п��㺯��
uint8_t fish_Control(void){
	static int r1,r2,r3,l1,l2,l3;	// ���ת��
	
	if (mode == 1) return 0;		// ���ΪѲ��ģʽ�����˳�
	
	// �Ի������˶����н���
	r1 = (int)((fb_val-turn_val-lr_val)*600/8);
	r3 = (int)((fb_val-turn_val+lr_val)*600/8);
	l1 = (int)((fb_val+turn_val+lr_val)*600/8);
	l3 = (int)((fb_val+turn_val-lr_val)*600/8);
	r2 = (int)(ug_val*200/8);
	l2 = (int)(ug_val*200/8);
	
	// �������ֵ  
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

void send_atmospheric(void){ // ���ʹ���ѹֵ����
	static int message; 
	
	if (send_atm==1){
		// send_atmospheric���н��������㷢������;
		message = atmospheric - 900;
		
		if(message<256 && message>=0){	
			
			USART_SendData(UART4,(uint8_t)message);	
		}
		else erro();	// ���ڷ�Χ�ڻᷢ��ʧ�ܣ����б�����
		
		send_atm=0;		// ����֮����ֵ����Ϊ0
	}
}

void send_state(void){ 		// ���͵�ǰ��ѹֵ�Լ����״̬
	static uint8_t message,ai_stated;
	if(mode==1){						//ֻ��Ѳ���ʱ����
		send_atmospheric(); 			// ������ѹֵ
		// ���͵�ǰֵ
		ai_stated = get_AiState()<<7;
		//printf("ai_stated:%d\r\n",ai_stated);		
		message = (uint8_t) (Pressure+64);
		message = message & 0X7F;
		message = message | ai_stated;
		USART_SendData(UART4,message);
	}
}
