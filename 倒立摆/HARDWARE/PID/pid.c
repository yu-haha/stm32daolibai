#include "pid.h"
#include "encoder.h"
#include "led.h"
#include "timer.h"
#include "adc.h"
#include "delay.h"
#include "ch451.h"
float Balance_Kp=15,Balance_Kd=80,Velocity_Kp=130,Velocity_Ki=1.5;
//float Balance_Kp=14,Balance_Kd=85,Velocity_Kp=125,Velocity_Ki=1.5;
//float Balance_Kp=100,Balance_Kd=295,Velocity_Kp=33,Velocity_Ki=50;
u8 Flag_qb;
extern float pwma;
extern int a,b,key;
extern int a1;
int c1,c2;
 int Balance_Pwm,pwm,Velocity_Pwm;
 
 
int run(void)
{
	TIM4_IRQHandler();
	if(key==1)
	{
		while(b<1600&&b>150)
	{
     TIM_Cmd(TIM4,DISABLE); 	
	if((b<2300&&b>2000)||b<300)	
	{   AIN2=1,			AIN1=0;
	    TIM_SetCompare1(TIM14,1000);
			delay_ms(500);}
	else{		
			AIN2=0,			AIN1=1;
		  TIM_SetCompare1(TIM14,800);
	}
	  	b=Get_Adc_Average(ADC_Channel_5,5);
	}
   TIM_Cmd(TIM4,ENABLE);
	if((b>2300&&b<2800)||(b<4095&&b>3600))
	{
	  Balance_Pwm =balance(b); 	//===平衡PID控制
		if(Balance_Pwm>0)			AIN2=0,			AIN1=1;
			else 	          AIN2=1,			AIN1=0;
		
		c1=-Balance_Pwm;
		TIM_SetCompare1(TIM14,500);
	}
 }

	if(b<3600&&b>2800)
	{
		TIM4_IRQHandler();
	  Balance_Pwm =balance(b); 	//===平衡PID控制	
    Velocity_Pwm=velocity(a,c2);                  //===速度环PID控制	 记住，速度反馈是正反馈，就是小车快的时候要慢下来就需要再跑快一点  
 	  pwm =-Balance_Pwm+Velocity_Pwm;                            //===计算左轮电机最终PWM
	  Set_Pwm(pwm);  
	  Xianfu_Pwm();	
    TIM_SetCompare1(TIM14,pwma);		
	}
	if(b>150&&b<1600) AIN2=0,			AIN1=0;
	

//else AIN2=0,			AIN1=0;
//	if(key==2)
//	{
//		TIM4_IRQHandler();
//	  Balance_Pwm =balance(b); 	//===平衡PID控制	
//    Velocity_Pwm=velocity(a);                  //===速度环PID控制	 记住，速度反馈是正反馈，就是小车快的时候要慢下来就需要再跑快一点  
// 	  pwm =-Balance_Pwm+Velocity_Pwm;                            //===计算左轮电机最终PWM
//	  Set_Pwm(pwm);  
//	  Xianfu_Pwm();	
//	}
	
	
  if(key==3)
	{
		AIN2=0,			AIN1=0;
	}
	if(key==2)c2=4;
	if(key==6)c2=-4;
	if(key==10)c2=0;
	return 0;
}


//int jishuan(void)
//{
//	
//		TIM4_IRQHandler();
//	  Balance_Pwm =balance(b); 	//===平衡PID控制	
//    Velocity_Pwm=velocity(a);                  //===速度环PID控制	 记住，速度反馈是正反馈，就是小车快的时候要慢下来就需要再跑快一点  
// 	  pwm =-Balance_Pwm+Velocity_Pwm;                            //===计算左轮电机最终PWM
//	  Set_Pwm(pwm);  
//	  Xianfu_Pwm();
//	  return 0;		
//	
//}			
//				
				
				
int balance(float Angle)
{  
   float Bias;                       
	 static float Last_Bias,D_Bias;    
	 int balance;                      
	 Bias=Angle-3270;              //求出平衡的角度中值 和机械相关
	 D_Bias=Bias-Last_Bias;            //求出偏差的微分 进行微分控制
	 balance=-Balance_Kp*Bias-D_Bias*Balance_Kd;   //===计算倾角控制的电机PWM  PD控制
   Last_Bias=Bias;                   //保持上一次的偏差
	 return balance;
}




//int balance(float pitch,float Angle)
//{  
//   float Bias,differential,last_Bias;
//	 int balance;
//	 Bias=pitch-Angle;       //===求出平衡的角度中值 和机械相关
//	 differential=Bias-last_Bias;//d
//	 balance=Balance_Kp*Bias+differential*Balance_Kd;   //===计算平衡控制的电机PWM  PD控制   kp是P系数 kd是D系数 
// 	 last_Bias=Bias;
//	 return balance;
//}



//速度环

int velocity(int encoder_left,int movement)
{  
    static float Velocity ,Encoder_Least,Encoder;
	  static float Encoder_Integral;
  	

   //=============速度PI控制器=======================//	
		Encoder_Least =(encoder_left)-0;                    //===获取最新速度偏差==测量速度（左右编码器之和）-目标速度（此处为零） 
		Encoder *= 0.8f;		                                                //===一阶低通滤波器       
		Encoder += Encoder_Least*0.2f;	                                    //===一阶低通滤波器    
		Encoder_Integral +=Encoder-movement;                                       //===积分出位移 积分时间：10ms
		if(Encoder_Integral>500)  	Encoder_Integral=500;             //===积分限幅
		if(Encoder_Integral<-500)	 Encoder_Integral=-500;              //===积分限幅	
		Velocity=Encoder*Velocity_Kp+Encoder_Integral*Velocity_Ki;                          //===速度控制	
	  return Velocity;
}

////位置环

//int velocity(int Encoder)
//{  
//   static float Position_PWM,Last_Position,Position_Bias,Position_Differential;
//	 static float Position_Least;
//  	Position_Least =Encoder;             //===
//    Position_Bias *=0.8f;		   
//    Position_Bias += Position_Least*0.2f;	             //===一阶低通滤波器  
//	  Position_Differential=Position_Bias-Last_Position;
//	  Last_Position=Position_Bias;
//		Position_PWM=Position_Bias*Velocity_Kp+Position_Differential*Velocity_Ki; //===速度控制	
//	  return Position_PWM;
//}



//void Run()
//{ 		   
//	 int  t1 ,t2;
//	if(key==1)t1=1;                                 //按键触发进入起摆
//	{
//		while(t1==1)
//		{
//			t2=100;
//		  TIM_SetCompare1(TIM14,t2 );
//			delay_ms(10);
//			t2=-t2;
//			if(a>3000|a<3600)t1=2;
//		}
//	}
//}

void Set_Pwm(int moto1)
{
    	if(moto1<0)			AIN2=0,			AIN1=1;
			else 	          AIN2=1,			AIN1=0;
			pwma=myabs(moto1);
		
}

void Xianfu_Pwm(void)
{	
	  int Amplitude=900;    //===PWM满幅是7200 限制在6900
    if(pwma<-Amplitude) pwma=-Amplitude;	
		if(pwma>Amplitude)  pwma=Amplitude;	
	  if(b<2900|b>3600)  pwma=0;
}


int myabs(int a)
{ 		   
	  int temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}









//void Run(u8 Way)
//{ 		   
//	  static float Count_FZ,Count_Next,Target_Position=10380;
//	  static u8 Flag_Back;
//  	static float Count_Big_Angle=0.046542;
//	  static int Position_Max;
//	if(Way==1)                                 //按键触发进入起摆
//	{
//	if(Flag_qb==1)  //第一步，摆杆自由摆动，振幅越来越大
//	{
//		Ratio=1; //正常pid
//		Count_qb+=Count_Big_Angle;   //自变量
//		Count_Big_Angle-=0.0000027;      //振幅越大，摆动周期减小
//		Count_FZ+=0.025;   //振幅越来越大
//		Target_Position=0.6*Count_FZ*sin(Count_qb)+10000;  //运动公式	 
//		Encoder=Read_Encoder(2);             	       //===更新编码器
//		Moto_qb=Position_PID(Encoder,Target_Position);  //位置闭环控制
//		if(Moto_qb>7200)Moto_qb=7200;//控制位置闭环控制过程的速度
//		if(Moto_qb<-7200)Moto_qb=-7200;//控制位置闭环控制过程的速度
//		//2100
//		if(Angle_Balance>(Angle_Max+850)&&Angle_Balance<2320&&D_Angle_Balance<=-1)   //振幅大于阈值时，进入下一个
//		{
//			Flag_qb++;
//			Count_qb=0;
//			TIM2->CNT=10000;        //复位一下计数寄存器
//			Count_FZ=0;
//		}
//	Set_Pwm(Moto_qb); //赋值给pwm 
//	}
//	 if(Flag_qb==2)            //第2步，通过位置控制，利用惯性，自动起摆
//	 {	 
//		 Target_Position=10600;                        //设定目标值
//		 Encoder=Read_Encoder(2);             	       //===更新编码器
//	 Moto_qb=Position_PID(Encoder,Target_Position);//===位置pid
//		 if(Moto_qb>7200) Moto_qb=7200;
//		 if(Moto_qb<-7200)Moto_qb=-7200;
//	 Set_Pwm(Moto_qb);  //赋值pwm
//		if(Angle_Balance<(ZHONGZHI+200)&&Angle_Balance>(ZHONGZHI-200))  //到底接近平衡位置 开启平衡系统
//		{	
//			State=1;   //倒立摆状态置1
//			Way_Turn=0;//自动起摆标志位清零
//			Flag_qb=0; //自动起摆步骤清零
//			Angle_Max=0;
//			Balance_KP += 30;
//			Balance_KD += 200;
//		}
//	 }
//	 if(Flag_qb==3)            //停
//	 {
//		AIN1 = 1;AIN2 = 1;
//	 }
//	}
//	if(Way==2) //手动起摆程序
//	{
//	if(Angle_Balance<(ZHONGZHI+200)&&Angle_Balance>(ZHONGZHI-200))  //到底接近平衡位置 开启平衡系统
//		{	
//			State=1;   //倒立摆状态置1
//			Way_Turn=0;//自动起摆标志位清零
//		}
//	}
//}
//int Position_PID (int Encoder,int Target)
//{ 	
//	 static float Bias,Pwm,Integral_bias,Last_Bias;
//	 Bias=Encoder-Target;                                   //计算偏差
//	 Integral_bias+=Bias;	                                  //积分
// //	  Pwm=70*Bias*Ratio+0.00*Integral_bias*Ratio+200*(Bias-Last_Bias)*Ratio;   //位置pid
//  Pwm=90*Bias*Ratio+0.00*Integral_bias*Ratio+250*(Bias-Last_Bias)*Ratio;   //???PID???
//	 Last_Bias=Bias;                                         
//	 return Pwm;                                       
//}











