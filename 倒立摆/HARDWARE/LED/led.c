#include "led.h" 

void LED_Init(void)
{    	 
  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);//使能GPIOF时钟

  //GPIOF9,F10初始化设置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOE, &GPIO_InitStructure);//初始化
	
	GPIO_SetBits(GPIOE,GPIO_Pin_12 | GPIO_Pin_13);//GPIOF9,F10设置高，灯灭
	
	


}

void AIN(u8 a)
{
	if(a==1)
	{AIN1 = 1;
	AIN2 = 0;}
	
		if(a==0)
	{AIN1 = 0;
	AIN2 = 0;}
	
		if(a==2)
	{AIN1 = 0;
	AIN2 = 1;}
}














