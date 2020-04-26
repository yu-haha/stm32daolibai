#ifndef __CH451_H
#define __CH451_H
#include "sys.h"

typedef struct
{
	u16    Q_Start;
	u16    Mode;         //问题选择标志位
	u16    POS_Sec;
	u8     save_flag;    //保存标志位
	u8     flash_start;
  u8     change_mode;    //改变服务函数
}Question;
extern  Question  My;



#define  dclk_0  GPIO_ResetBits(GPIOB,GPIO_Pin_6)
#define  dclk_1  GPIO_SetBits(GPIOB,GPIO_Pin_6)
#define  din_0   GPIO_ResetBits(GPIOB,GPIO_Pin_7)
#define  din_1   GPIO_SetBits(GPIOB,GPIO_Pin_7)
#define  load_0  GPIO_ResetBits(GPIOB,GPIO_Pin_8)
#define  load_1  GPIO_SetBits(GPIOB,GPIO_Pin_8)
#define  dout    GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_9)

void  ch451_write(u16 command);
void  ch451_init(void);
u16   ch451_read(void);
u8 get_val(u8 a);
u16 get_value(u8 mode);
void CH451_Get_val(void);



#endif



