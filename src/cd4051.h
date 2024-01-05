#ifndef __cd4051_h__
#define __cd4051_h__

#include "stm32f10x.h"
#include "dealy.h"

#define cd4051A_pin GPIO_Pin_5
#define cd4051A_port GPIOB
#define cd4051B_pin GPIO_Pin_6
#define cd4051B_port GPIOB
#define cd4051C_pin GPIO_Pin_7
#define cd4051C_port GPIOB

#define cd4051A(x) 		GPIO_WriteBit(GPIOB, cd4051A_pin,  (BitAction)(x))//cs
#define cd4051B(x) 		GPIO_WriteBit(GPIOB, cd4051B_pin,  (BitAction)(x))//cs
#define cd4051C(x) 		GPIO_WriteBit(GPIOB, cd4051C_pin,  (BitAction)(x))//cs


void MOS_gpio_Init(void)
{

    //RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = cd4051A_pin|cd4051B_pin|cd4051C_pin;
    GPIO_Init(GPIOB,&GPIO_InitStructure);
}

void channel_choose(uint8_t number){

    switch (number)
    {
    case 0:
        CD4051A(0);
        CD4051B(0);
        CD4051C(0);
        break;

    case 1:
        CD4051A(1);
        CD4051B(0);
        CD4051C(0);
        break;
    case 2:
        CD4051A(0);
        CD4051B(1);
        CD4051C(0);
        break;

    case 3:
        CD4051A(1);
        CD4051B(1);
        CD4051C(0);
        break;
    case 4:
        CD4051A(0);
        CD4051B(0);
        CD4051C(1);
        break;
    case 5:
        CD4051A(1);
        CD4051B(0);
        CD4051C(1);
        break;
    case 6:
        CD4051A(0);
        CD4051B(1);
        CD4051C(1);
        break;

    case 7:
        CD4051A(1);
        CD4051B(1);
        CD4051C(1);
        break;        

    default:
        break;
    }
}

void batter_current_capture(uint8_t number)
{

    switch (number)
    {
    case 1:
        channel_choose(5);
        break;

    case 2:
        channel_choose(7);
        break;

    case 3:
        channel_choose(6);
        break;

    case 4:
        channel_choose(4);
        break;

/*************************************/
    case 5:
        channel_choose(2);
        break;

    case 6:
        channel_choose(1);
        break;

    case 7:
        channel_choose(0);
        break;

    case 8:
        channel_choose(3);
        break;

    default:
        break;
    }
}