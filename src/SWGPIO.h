#ifndef __sw1_gpioa1_h__
#define __sw1_gpioa1_h__

#include "stm32f10x.h"
#include "dealy.h"

#define sw1_pin GPIO_Pin_15
#define sw1_port GPIOC
#define sw2_pin GPIO_Pin_14
#define sw2_port GPIOC
#define sw3_pin GPIO_Pin_13
#define sw3_port GPIOC
#define sw4_pin GPIO_Pin_10
#define sw4_port GPIOB


void sw_gpio_Init(void)
{
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = sw1_pin|sw2_pin|sw3_pin;
  GPIO_Init(GPIOC,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin =sw4_pin;
  GPIO_Init(GPIOB,&GPIO_InitStructure);
}

void delay_ms(uint16_t time)
{
    Delay_ms(time);
}

uint8_t key_get()
{
  uint16_t time=0 ;
  int8_t flag = 0;
  delay_ms(10);
  if (GPIO_ReadInputDataBit(sw1_port,sw1_pin) == 0)
  {
    delay_ms(10);
    while (time < 100)
    {
      delay_ms(10);
      time++;
      if (GPIO_ReadInputDataBit(sw1_port,sw1_pin) != 0) {break;}
    }
    while (GPIO_ReadInputDataBit(sw1_port,sw1_pin) == 0) { };
    delay_ms(10);
    if (time > 90)
      {
        flag = 5;//?¡è¡ã?sw1 ??5
        time = 0;
      }
      else
      {
        flag = 1;//??¡ã?sw1 ??1
        time = 0;
      };
    }

  if (GPIO_ReadInputDataBit(sw2_port,sw2_pin) == 0)
  {
    delay_ms(10);
    while (time < 100)
    {
      delay_ms(10);
      time++;
      if (GPIO_ReadInputDataBit(sw2_port,sw2_pin) != 0) {break;}
    }
    while (GPIO_ReadInputDataBit(sw2_port,sw2_pin) == 0) { };
      delay_ms(10);
      if (time > 90)
      {
        flag = 6;//?¡è¡ã?sw2 ??6
        time = 0;
      }
      else
      {
        flag = 2;//??¡ã?sw2 ??2
        time = 0;
      };
  }
    
  if (GPIO_ReadInputDataBit(sw3_port,sw3_pin) == 0)
  {
    delay_ms(10);
    while (time < 100)
    {
      delay_ms(10);
      time++;
      if (GPIO_ReadInputDataBit(sw3_port,sw3_pin) != 0) {break;}
    }
    while (GPIO_ReadInputDataBit(sw3_port,sw3_pin) == 0) { };
      delay_ms(10);
      if (time > 90)
      {
        flag = 7;//?¡è¡ã?sw3 ??7
        time = 0;
      }
      else
      {
        flag = 3;//??¡ã?sw3 ??3
        time = 0;
      };
    }


  if (GPIO_ReadInputDataBit(sw4_port,sw4_pin) == 0)
  {
    delay_ms(10);
    while (time < 100)
    {
      delay_ms(10);
      time++;
      if (GPIO_ReadInputDataBit(sw4_port,sw4_pin) != 0) {break;}
    }
    while (GPIO_ReadInputDataBit(sw4_port,sw4_pin) == 0) { };
      delay_ms(10);
      if (time > 90)
      {
        flag = 8;//?¡è¡ã?sw4 ??8
        time = 0;
      }
      else
      {
        flag = 4;//??¡ã?sw4 ??4
        time = 0;
      };
  }
  return flag;
}
#endif // !sw1_gpioa1_h__