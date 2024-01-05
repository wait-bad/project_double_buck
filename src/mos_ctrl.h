#ifndef __mos_ctrl_h__
#define __mos_ctrl_h__

#include "stm32f10x.h"
#include "dealy.h"

#define g8_pin GPIO_Pin_12
#define g7_pin GPIO_Pin_11
#define g6_pin GPIO_Pin_10
#define g5_pin GPIO_Pin_9
#define g4_pin GPIO_Pin_8

#define g3_pin GPIO_Pin_15
#define g2_pin GPIO_Pin_12
#define g1_pin GPIO_Pin_13

#define g8_set(x) 		GPIO_WriteBit(GPIOA, g8_pin,  (BitAction)(x))//cs
#define g7_set(x) 		GPIO_WriteBit(GPIOA, g7_pin,  (BitAction)(x))//cs
#define g6_set(x) 		GPIO_WriteBit(GPIOA, g6_pin,  (BitAction)(x))//cs
#define g5_set(x) 		GPIO_WriteBit(GPIOA, g5_pin,  (BitAction)(x))//cs
#define g4_set(x) 		GPIO_WriteBit(GPIOA, g4_pin,  (BitAction)(x))//cs

#define g3_set(x) 		GPIO_WriteBit(GPIOB, g3_pin,  (BitAction)(x))//cs
#define g2_set(x) 		GPIO_WriteBit(GPIOB, g2_pin,  (BitAction)(x))//cs
#define g1_set(x) 		GPIO_WriteBit(GPIOB, g1_pin,  (BitAction)(x))//cs

void mos_all_set_off(void);
void mos_all_set_on(void);
void mos_gate(uint8_t number,uint8_t stage );

void MOS_gpio_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    //RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_InitStructure.GPIO_Pin = g3_pin|g2_pin|g1_pin;
    GPIO_Init(GPIOB,&GPIO_InitStructure);

	mos_all_set_off();
}

void mos_all_set_off()
{
	g1_set(0);
	g2_set(0);
	g3_set(0);

	g4_set(0);
	g5_set(0);
	g6_set(0);
	g7_set(0);
	g8_set(0);
}

void mos_all_set_on()
{
	g1_set(1);
	g2_set(1);
	g3_set(1);

	g4_set(1);
	g5_set(1);
	g6_set(1);
	g7_set(1);
	g8_set(1);
}

void mos_gate(uint8_t number,uint8_t stage )
{
	switch (number)
	{
	case 1:
		if (stage == 0){g1_set(0);}
		else{g1_set(1);}
	break;

	case 2:
		if (stage == 0){g2_set(0);}
		else{g2_set(1);}
	break;

	case 3:
		if (stage == 0){g3_set(0);}
		else{g3_set(1);}
	break;

	case 4:
		if (stage == 0){g4_set(0);}
		else{g4_set(1);}
	break;

	case 5:
		if (stage == 0){g5_set(0);}
		else{g5_set(1);}
	break;

	case 6:
		if (stage == 0){g6_set(0);}
		else{g6_set(1);}
	break;

	case 7:
		if (stage == 0){g7_set(0);}
		else{g7_set(1);}
	break;

	case 8:
		if (stage == 0){g8_set(0);}
		else{g8_set(1);}
	break;
	
	default:
		break;
	}
}

void mos_set_gate(uint8_t stage)
{
	uint8_t i;
	for ( i = 1; i < 9; i++)
	{
		mos_gate(i,stage&0x80>>(i-1));
	}
}



#endif