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
	  Balance_Pwm =balance(b); 	//===ƽ��PID����
		if(Balance_Pwm>0)			AIN2=0,			AIN1=1;
			else 	          AIN2=1,			AIN1=0;
		
		c1=-Balance_Pwm;
		TIM_SetCompare1(TIM14,500);
	}
 }

	if(b<3600&&b>2800)
	{
		TIM4_IRQHandler();
	  Balance_Pwm =balance(b); 	//===ƽ��PID����	
    Velocity_Pwm=velocity(a,c2);                  //===�ٶȻ�PID����	 ��ס���ٶȷ�����������������С�����ʱ��Ҫ����������Ҫ���ܿ�һ��  
 	  pwm =-Balance_Pwm+Velocity_Pwm;                            //===�������ֵ������PWM
	  Set_Pwm(pwm);  
	  Xianfu_Pwm();	
    TIM_SetCompare1(TIM14,pwma);		
	}
	if(b>150&&b<1600) AIN2=0,			AIN1=0;
	

//else AIN2=0,			AIN1=0;
//	if(key==2)
//	{
//		TIM4_IRQHandler();
//	  Balance_Pwm =balance(b); 	//===ƽ��PID����	
//    Velocity_Pwm=velocity(a);                  //===�ٶȻ�PID����	 ��ס���ٶȷ�����������������С�����ʱ��Ҫ����������Ҫ���ܿ�һ��  
// 	  pwm =-Balance_Pwm+Velocity_Pwm;                            //===�������ֵ������PWM
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
//	  Balance_Pwm =balance(b); 	//===ƽ��PID����	
//    Velocity_Pwm=velocity(a);                  //===�ٶȻ�PID����	 ��ס���ٶȷ�����������������С�����ʱ��Ҫ����������Ҫ���ܿ�һ��  
// 	  pwm =-Balance_Pwm+Velocity_Pwm;                            //===�������ֵ������PWM
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
	 Bias=Angle-3270;              //���ƽ��ĽǶ���ֵ �ͻ�е���
	 D_Bias=Bias-Last_Bias;            //���ƫ���΢�� ����΢�ֿ���
	 balance=-Balance_Kp*Bias-D_Bias*Balance_Kd;   //===������ǿ��Ƶĵ��PWM  PD����
   Last_Bias=Bias;                   //������һ�ε�ƫ��
	 return balance;
}




//int balance(float pitch,float Angle)
//{  
//   float Bias,differential,last_Bias;
//	 int balance;
//	 Bias=pitch-Angle;       //===���ƽ��ĽǶ���ֵ �ͻ�е���
//	 differential=Bias-last_Bias;//d
//	 balance=Balance_Kp*Bias+differential*Balance_Kd;   //===����ƽ����Ƶĵ��PWM  PD����   kp��Pϵ�� kd��Dϵ�� 
// 	 last_Bias=Bias;
//	 return balance;
//}



//�ٶȻ�

int velocity(int encoder_left,int movement)
{  
    static float Velocity ,Encoder_Least,Encoder;
	  static float Encoder_Integral;
  	

   //=============�ٶ�PI������=======================//	
		Encoder_Least =(encoder_left)-0;                    //===��ȡ�����ٶ�ƫ��==�����ٶȣ����ұ�����֮�ͣ�-Ŀ���ٶȣ��˴�Ϊ�㣩 
		Encoder *= 0.8f;		                                                //===һ�׵�ͨ�˲���       
		Encoder += Encoder_Least*0.2f;	                                    //===һ�׵�ͨ�˲���    
		Encoder_Integral +=Encoder-movement;                                       //===���ֳ�λ�� ����ʱ�䣺10ms
		if(Encoder_Integral>500)  	Encoder_Integral=500;             //===�����޷�
		if(Encoder_Integral<-500)	 Encoder_Integral=-500;              //===�����޷�	
		Velocity=Encoder*Velocity_Kp+Encoder_Integral*Velocity_Ki;                          //===�ٶȿ���	
	  return Velocity;
}

////λ�û�

//int velocity(int Encoder)
//{  
//   static float Position_PWM,Last_Position,Position_Bias,Position_Differential;
//	 static float Position_Least;
//  	Position_Least =Encoder;             //===
//    Position_Bias *=0.8f;		   
//    Position_Bias += Position_Least*0.2f;	             //===һ�׵�ͨ�˲���  
//	  Position_Differential=Position_Bias-Last_Position;
//	  Last_Position=Position_Bias;
//		Position_PWM=Position_Bias*Velocity_Kp+Position_Differential*Velocity_Ki; //===�ٶȿ���	
//	  return Position_PWM;
//}



//void Run()
//{ 		   
//	 int  t1 ,t2;
//	if(key==1)t1=1;                                 //���������������
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
	  int Amplitude=900;    //===PWM������7200 ������6900
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
//	if(Way==1)                                 //���������������
//	{
//	if(Flag_qb==1)  //��һ�����ڸ����ɰڶ������Խ��Խ��
//	{
//		Ratio=1; //����pid
//		Count_qb+=Count_Big_Angle;   //�Ա���
//		Count_Big_Angle-=0.0000027;      //���Խ�󣬰ڶ����ڼ�С
//		Count_FZ+=0.025;   //���Խ��Խ��
//		Target_Position=0.6*Count_FZ*sin(Count_qb)+10000;  //�˶���ʽ	 
//		Encoder=Read_Encoder(2);             	       //===���±�����
//		Moto_qb=Position_PID(Encoder,Target_Position);  //λ�ñջ�����
//		if(Moto_qb>7200)Moto_qb=7200;//����λ�ñջ����ƹ��̵��ٶ�
//		if(Moto_qb<-7200)Moto_qb=-7200;//����λ�ñջ����ƹ��̵��ٶ�
//		//2100
//		if(Angle_Balance>(Angle_Max+850)&&Angle_Balance<2320&&D_Angle_Balance<=-1)   //���������ֵʱ��������һ��
//		{
//			Flag_qb++;
//			Count_qb=0;
//			TIM2->CNT=10000;        //��λһ�¼����Ĵ���
//			Count_FZ=0;
//		}
//	Set_Pwm(Moto_qb); //��ֵ��pwm 
//	}
//	 if(Flag_qb==2)            //��2����ͨ��λ�ÿ��ƣ����ù��ԣ��Զ����
//	 {	 
//		 Target_Position=10600;                        //�趨Ŀ��ֵ
//		 Encoder=Read_Encoder(2);             	       //===���±�����
//	 Moto_qb=Position_PID(Encoder,Target_Position);//===λ��pid
//		 if(Moto_qb>7200) Moto_qb=7200;
//		 if(Moto_qb<-7200)Moto_qb=-7200;
//	 Set_Pwm(Moto_qb);  //��ֵpwm
//		if(Angle_Balance<(ZHONGZHI+200)&&Angle_Balance>(ZHONGZHI-200))  //���׽ӽ�ƽ��λ�� ����ƽ��ϵͳ
//		{	
//			State=1;   //������״̬��1
//			Way_Turn=0;//�Զ���ڱ�־λ����
//			Flag_qb=0; //�Զ���ڲ�������
//			Angle_Max=0;
//			Balance_KP += 30;
//			Balance_KD += 200;
//		}
//	 }
//	 if(Flag_qb==3)            //ͣ
//	 {
//		AIN1 = 1;AIN2 = 1;
//	 }
//	}
//	if(Way==2) //�ֶ���ڳ���
//	{
//	if(Angle_Balance<(ZHONGZHI+200)&&Angle_Balance>(ZHONGZHI-200))  //���׽ӽ�ƽ��λ�� ����ƽ��ϵͳ
//		{	
//			State=1;   //������״̬��1
//			Way_Turn=0;//�Զ���ڱ�־λ����
//		}
//	}
//}
//int Position_PID (int Encoder,int Target)
//{ 	
//	 static float Bias,Pwm,Integral_bias,Last_Bias;
//	 Bias=Encoder-Target;                                   //����ƫ��
//	 Integral_bias+=Bias;	                                  //����
// //	  Pwm=70*Bias*Ratio+0.00*Integral_bias*Ratio+200*(Bias-Last_Bias)*Ratio;   //λ��pid
//  Pwm=90*Bias*Ratio+0.00*Integral_bias*Ratio+250*(Bias-Last_Bias)*Ratio;   //???PID???
//	 Last_Bias=Bias;                                         
//	 return Pwm;                                       
//}











