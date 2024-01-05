#ifndef __exit_h_
#define __exit_h_

#include "stm32f10x.h"

void key_exit1_init()
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
    GPIO_InitTypeDef GPIO_initstructure;
    GPIO_initstructure.GPIO_Mode=GPIO_Mode_IPU;
    GPIO_initstructure.GPIO_Pin=GPIO_Pin_5;
    GPIO_initstructure.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_Init(GPIOA,&GPIO_initstructure);

    //打开afio的时钟 
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
    //将gpio作为外部中断引脚
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource5);

    EXTI_InitTypeDef EXTI_InitStructure;
    //触发线 选择exti线一
    EXTI_InitStructure.EXTI_Line=EXTI_Line5;
    //使能触发线
    EXTI_InitStructure.EXTI_LineCmd=ENABLE;
    //触发模式选择中断
    EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt;
    //触发边沿为下降沿
    EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Falling;

    EXTI_Init(&EXTI_InitStructure);
    //完成中断优先级分组 抢占优先级两位 响应优先级两位
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
   

    NVIC_InitTypeDef NVIC_Initstructure;
    //指定启用的irq通道
    NVIC_Initstructure.NVIC_IRQChannel=EXTI9_5_IRQn;
    //使能irq通道
    NVIC_Initstructure.NVIC_IRQChannelCmd=ENABLE;
    //抢着优先级2
    NVIC_Initstructure.NVIC_IRQChannelPreemptionPriority=2;
    //响应优先级2
    NVIC_Initstructure.NVIC_IRQChannelSubPriority=2;
    //初始化nvic
    NVIC_Init(&NVIC_Initstructure);  
}

void key_exti0_init()  
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
    GPIO_InitTypeDef GPIO_Initstructure;
    GPIO_Initstructure.GPIO_Mode=GPIO_Mode_IPU;
    GPIO_Initstructure.GPIO_Pin=GPIO_Pin_7;
    GPIO_Initstructure.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_Init(GPIOA,&GPIO_Initstructure);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
    //选择中断线
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource7);

    EXTI_InitTypeDef EXTI_Initsturcture;
    EXTI_Initsturcture.EXTI_Line=EXTI_Line7;
    EXTI_Initsturcture.EXTI_LineCmd=ENABLE;
    EXTI_Initsturcture.EXTI_Mode=EXTI_Mode_Interrupt;
    EXTI_Initsturcture.EXTI_Trigger=EXTI_Trigger_Falling;
    EXTI_Init(&EXTI_Initsturcture);


    NVIC_InitTypeDef NVIC_Initstructure;
    NVIC_Initstructure.NVIC_IRQChannel=EXTI9_5_IRQn;
    NVIC_Initstructure.NVIC_IRQChannelCmd=ENABLE;
    NVIC_Initstructure.NVIC_IRQChannelPreemptionPriority=1;
    NVIC_Initstructure.NVIC_IRQChannelSubPriority=1;
    NVIC_Init(&NVIC_Initstructure);
}

void gpio_6_down()
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;

	GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOA,GPIO_Pin_6);
}

//拉低A2做地 消抖使用
void gpio_A2_down()
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;

	GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOA,GPIO_Pin_2);
}

void gpio_C14_down()
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;

	GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOC,GPIO_Pin_14);
}

//校准中断
void sw1_exti_init()
    {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
    GPIO_InitTypeDef GPIO_initstructure;
    GPIO_initstructure.GPIO_Mode=GPIO_Mode_IPU;
    GPIO_initstructure.GPIO_Pin=GPIO_Pin_0;
    GPIO_initstructure.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_Init(GPIOA,&GPIO_initstructure);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource0);

    EXTI_InitTypeDef EXTI_InitStructure;
    EXTI_InitStructure.EXTI_Line=EXTI_Line0;
    
    EXTI_InitStructure.EXTI_LineCmd=ENABLE;
    EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Falling;
    EXTI_Init(&EXTI_InitStructure);
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
   

    NVIC_InitTypeDef NVIC_Initstructure;
    NVIC_Initstructure.NVIC_IRQChannel=EXTI0_IRQn;
    NVIC_Initstructure.NVIC_IRQChannelCmd=ENABLE;
    NVIC_Initstructure.NVIC_IRQChannelPreemptionPriority=2;
    NVIC_Initstructure.NVIC_IRQChannelSubPriority=2;
    NVIC_Init(&NVIC_Initstructure);  
}


void sw2_exti_init()
    {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
    GPIO_InitTypeDef GPIO_initstructure;
    GPIO_initstructure.GPIO_Mode=GPIO_Mode_IPU;
    GPIO_initstructure.GPIO_Pin=GPIO_Pin_1;
    GPIO_initstructure.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_Init(GPIOA,&GPIO_initstructure);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource1);

    EXTI_InitTypeDef EXTI_InitStructure;
    EXTI_InitStructure.EXTI_Line=EXTI_Line1;
    
    EXTI_InitStructure.EXTI_LineCmd=ENABLE;
    EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Falling;
    EXTI_Init(&EXTI_InitStructure);
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    NVIC_InitTypeDef NVIC_Initstructure;
    NVIC_Initstructure.NVIC_IRQChannel=EXTI1_IRQn;
    NVIC_Initstructure.NVIC_IRQChannelCmd=ENABLE;
    NVIC_Initstructure.NVIC_IRQChannelPreemptionPriority=3;
    NVIC_Initstructure.NVIC_IRQChannelSubPriority=3;
    NVIC_Init(&NVIC_Initstructure);  
}

#endif // !__exit_h_
