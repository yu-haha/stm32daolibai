#ifndef __pid_H
#define __pid_H
#include <sys.h>


float PID_balance(float Angle)     ;
int balance(float Angle);	

int myabs(int a);
void Get_Angle(u8 way);
void Xianfu_Pwm(void);
void Set_Pwm(int pwma);
int velocity(int encoder_left,int movement);
int jishuan(void);
u8 TurnOff(float angle, int voltage);
int turn(int encoder_zhuanxiang,float gyro);
int  run(void);







	
#endif
