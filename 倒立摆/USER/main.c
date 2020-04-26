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
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	delay_init(168);//初始化延时函数
	uart_init(115200);//初始化串口波特率为115200		
 	OLED_Init();//初始化OLED
  Adc_Init();              //初始化adc
	LED_Init();                 //led初始化
  ch451_init();               //键盘初始化
  TIM14_PWM_Init(1000-1,84-1);//84M/84=1Mhz的计数频率,重装载值1000，所以PWM频率为 1M/1000=1Khz.  
  TIM4_Int_Init(100-1,8400-1);	//定时器时钟84M，分频系数8400，所以84M/8400=10Khz的计数频率，计数5000次为500ms 	
  Encoder_Init_TIM2();        //编码器定时器初始化
 
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
		OLED_Refresh_Gram();//更新显示到OLED
	}
}

