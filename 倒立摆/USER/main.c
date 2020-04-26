#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "oled.h"
#include "ch451.h"
#include "adc.h"
#include "pwm.h"
#include "timer.h"
#include "encoder.h"
#include "led.h" 
#include "pid.h"
#include "math.h"

int a,b,c,b1;
int key;
 float pwma;
 
 

 
int main(void)
{ 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//����ϵͳ�ж����ȼ�����2
	delay_init(168);//��ʼ����ʱ����
	uart_init(115200);//��ʼ�����ڲ�����Ϊ115200		
 	OLED_Init();//��ʼ��OLED
  Adc_Init();              //��ʼ��adc
	LED_Init();                 //led��ʼ��
  ch451_init();               //���̳�ʼ��
  TIM14_PWM_Init(1000-1,84-1);//84M/84=1Mhz�ļ���Ƶ��,��װ��ֵ1000������PWMƵ��Ϊ 1M/1000=1Khz.  
  TIM4_Int_Init(100-1,8400-1);	//��ʱ��ʱ��84M����Ƶϵ��8400������84M/8400=10Khz�ļ���Ƶ�ʣ�����5000��Ϊ500ms 	
  Encoder_Init_TIM2();        //��������ʱ����ʼ��
 
	while(1) 
	{

    key=ch451_read();
    run();
	//  jishuan();
//	  TIM_SetCompare1(TIM14,pwma );
  	OLED_ShowDecimals(0,0,"",24,a);
	  OLED_ShowNum(0,24,b,5,24);
		//OLED_ShowDecimals(0,24,"",24,b);
    OLED_ShowDecimals(48,0,"",24,key);
		OLED_Refresh_Gram();//������ʾ��OLED
	}
}

