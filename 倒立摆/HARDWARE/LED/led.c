#include "led.h" 

void LED_Init(void)
{    	 
  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);//ʹ��GPIOFʱ��

  //GPIOF9,F10��ʼ������
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOE, &GPIO_InitStructure);//��ʼ��
	
	GPIO_SetBits(GPIOE,GPIO_Pin_12 | GPIO_Pin_13);//GPIOF9,F10���øߣ�����
	
	


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














