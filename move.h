#ifndef _MOVE_H
#define _MOVE_H
#include "sys.h"

typedef struct{
    float P;
    float I;
    float D;

    float PrevError;
    float SumError;
	float PID;

}Pid_t;

void Pid_Init(void);
void depth_balance(void);
void tracting(void);


void r1_motor(int pwm);
void r2_motor(int pwm);
void r3_motor(int pwm);
void l1_motor(int pwm);
void l2_motor(int pwm);
void l3_motor(int pwm);

#endif


