/*
 * ************************************************
 *
 * STM32 double_buck_project
 *
 * system clock 12M*9  108M
 * pwm_frequency 108M / 610 = 177.049k
 * 
 * CPU:     STM32F103C6
 * ADC_PIN:     PA1 PA2 PA3 PA4
 * PWM_PIN:     PA7-PA8    PB0-PA9
 * 
 * ************************************************
 */

/******header file********/
#include "stm32f10x.h"
#include "pwm.h"
#include "dealy.h"
#include "12864.h"
#include "exti.h"
#include "stm32f10x_flash.c"
#include "adc.h"
#include "aizi.h"
#include "mos_ctrl.h"

#define OPTPARSE_API static
#include "optparse.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
/******software infrastructure*******/

// ctrl_parameter ;
float set_voltage_1       = 0;
float set_voltage_2       = 0;
float set_current_1       = 0;
float set_current_2       = 0;
double voltage_cailbration = 0.008872;
double current_cailbration = 0.0013748;
int32_t constant         = 10000;//constant = 0.0001;
uint16_t set_Reload_value = 609; //  freq = 177k
uint16_t set_duty_max_value = 609;
uint16_t set_duty_min_value = 1;

uint16_t led__ = 0;


// un_ctrl_parameter


uint32_t set_duty_voltage_1 = 0;
uint32_t set_duty_voltage_2 = 0;
uint32_t set_duty_currten_1 = 0;
uint32_t set_duty_currten_2 = 0;
int32_t currten_duty_1 = 0;
int32_t currten_duty_2 = 0;
int32_t voltage_duty_1 = 0;
int32_t voltage_duty_2 = 0;
uint32_t set_duty_max  = 0;
uint32_t set_duty_min  = 0;

// flag
uint16_t tim_add_counter = 0;
uint16_t refresh_flag    = 0;

/******************************************/

void RCCCLOCK_Init(void)
{
    __IO uint32_t HSIstatue = 0;
    // 1 复位时钟
    RCC_DeInit();

    // 2 HSI使能并等待其就绪
    RCC_HSICmd(ENABLE);
    HSIstatue = RCC->CR & RCC_CR_HSIRDY;

    if (HSIstatue == 2) {
        // 3 HSI使能预取值，配置等待周期
        FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
        FLASH_SetLatency(FLASH_Latency_2);

        // 4 配置时钟来源和倍�?�系�?
        RCC_PLLConfig(RCC_PLLSource_HSI_Div2, RCC_PLLMul_16); //  8/2*16=64M

        // 5 使能PLL并等待其稳定
        RCC_PLLCmd(ENABLE);
        while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
            ;

        // 6 选择系统时钟
        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
        while (RCC_GetSYSCLKSource() != 0x08)
            ; // PLLCLK

        // 7 设置HCLK,PCLK2,PCLK2时钟
        RCC_HCLKConfig(RCC_SYSCLK_Div1);
        RCC_PCLK1Config(RCC_HCLK_Div2);
        RCC_PCLK2Config(RCC_HCLK_Div1);
    } else {
        //
    }
}

void Timer_Init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    TIM_InternalClockConfig(TIM2);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision     = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode       = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period            = set_Reload_value;
    TIM_TimeBaseInitStructure.TIM_Prescaler         = 0;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);

    TIM_ClearFlag(TIM2, TIM_FLAG_Update);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel                   = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1;
    NVIC_Init(&NVIC_InitStructure);
    TIM_Cmd(TIM2, ENABLE);
}

void show_decimals(char dir, char x, unsigned char y, unsigned long datab);

void TIM1_Configuration(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    // 使能TIM1时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    // 初�?�化TIM1
    TIM_TimeBaseStructure.TIM_Prescaler         = 0;
    TIM_TimeBaseStructure.TIM_CounterMode       = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period            = set_Reload_value; // 360kHz输出频率
    TIM_TimeBaseStructure.TIM_ClockDivision     = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    // 配置TIM1通道1为PWM模式
    TIM_OCInitStructure.TIM_OCMode       = TIM_OCMode_PWM2;
    TIM_OCInitStructure.TIM_OutputState  = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
    TIM_OCInitStructure.TIM_Pulse        = 1; // 初�?�占空比�?50%
    TIM_OCInitStructure.TIM_OCPolarity   = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCNPolarity  = TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState  = TIM_OCIdleState_Reset;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);

    // 配置TIM1通道1为PWM模式
    TIM_OCInitStructure.TIM_OCMode       = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState  = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
    TIM_OCInitStructure.TIM_Pulse        = 1; // 初�?�占空比�?50%
    TIM_OCInitStructure.TIM_OCPolarity   = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCNPolarity  = TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState  = TIM_OCIdleState_Reset;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
    TIM_OC2Init(TIM1, &TIM_OCInitStructure);

    // 配置TIM1通道1N为互�?PWM模式，并�?用�?�用映射
    TIM_BDTRInitTypeDef TIM_BDTRInitStructure;
    TIM_BDTRInitStructure.TIM_OSSRState       = TIM_OSSRState_Enable;
    TIM_BDTRInitStructure.TIM_OSSIState       = TIM_OSSIState_Enable;
    TIM_BDTRInitStructure.TIM_LOCKLevel       = TIM_LOCKLevel_OFF;
    TIM_BDTRInitStructure.TIM_DeadTime        = 0; // 死区时间设置
    TIM_BDTRInitStructure.TIM_Break           = TIM_Break_Disable;
    TIM_BDTRInitStructure.TIM_BreakPolarity   = TIM_BreakPolarity_High;
    TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
    TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);

    // 使能TIM1通道1输出
    TIM_CtrlPWMOutputs(TIM1, ENABLE);

    // 使能TIM1更新�?�?
    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);

    // 配置�?�?优先�?
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel                   = TIM1_UP_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; // �?以根�?需要�?�置
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0; // �?以根�?需要�?�置
    NVIC_Init(&NVIC_InitStructure);

    // 使能TIM1
    TIM_Cmd(TIM1, ENABLE);
}

void GPIO_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    // 使能GPIOA时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

    // 配置PA7为�?�用推挽输出，重映射到TIM1_CH1N
    // 使能AFIO时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    // 配置PA7为�?�用推挽输出，重映射到TIM1_CH1N
    GPIO_PinRemapConfig(GPIO_PartialRemap_TIM1, ENABLE);
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 配置PB0为�?�用推挽输出，重映射到TIM1_CH2N
    GPIO_PinRemapConfig(GPIO_PartialRemap_TIM1, ENABLE);
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // 配置PA8为�?�用推挽输出
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    // 配置PA8为�?�用推挽输出
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void count_un_ctrl_parameter();
/**
 * @brief init all system hardware
 *
 * @return uint8_t 1  Flag configuration complete
 */
uint8_t system_all_init()
{
    lcd_gpio_Init();
    lcd12864_init();
    Lcd12864_ClearScreen(0x00);

    AD_Init();
    TIM1_Configuration();
    GPIO_Configuration();
    //Timer_Init();
    MOS_gpio_Init();
    return 1;
}

/**
 * @brief
 * system frequency = 12*9  108M
 * adc_frequentcy   = 108M /76 / 4 =
 * @return int
 */
int main()
{
    system_all_init();
    set_current_1 = 5.5;
    set_current_2 = 2;
    set_voltage_1 = 1.25;

    set_voltage_2 = 8.45;
    set_duty_max_value = 510 ;
    count_un_ctrl_parameter();
    while (9) 
    {
      Delay_ms(10);
    }
}

void count_un_ctrl_parameter()
{
  set_duty_currten_1 = set_current_1 / current_cailbration;
  set_duty_currten_2 = set_current_2 / current_cailbration;
  set_duty_voltage_1 = set_voltage_1 / voltage_cailbration;
  set_duty_voltage_2 = set_voltage_2 / voltage_cailbration;
  set_duty_max       = set_duty_max_value*constant ; 
  set_duty_min       = set_duty_min_value*constant+5000;
}


/**
 * @brief 将四位数�?换成千进制小�?
 *
 * @param dir
 * @param x
 * @param y
 * @param datab �?
 */
void show_decimals(char dir, char x, unsigned char y, unsigned long datab)
{
    lcd12864_show_nmuber(dir, x, y, 2, datab / 1000);
    lcd12864_show_n(dir, x, y + 8 + 8, 12);
    lcd12864_show_nmuber(dir, x, y + 16 + 8, 3, datab % 1000);
};
/*
void TIM2_IRQHandler(void)
{
    int16_t dif_value = 0;
    uint16_t duty;
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET) {
      //AD_Value[4];// order   currten_2  voltage_2  current_1  voltage_1
      currten_duty_2 += (set_duty_currten_2 - AD_Value[0]);

      voltage_duty_2 += (set_duty_voltage_2 - AD_Value[1]);

      dif_value = set_duty_currten_1 - AD_Value[2];
      currten_duty_1 += dif_value ;
      dif_value = set_duty_voltage_1 - AD_Value[3];
      voltage_duty_1 += dif_value ; 



      if (voltage_duty_2 > set_duty_max)
      {
        voltage_duty_2 = set_duty_max;
      }
      else if (voltage_duty_2 < set_duty_min)
      {
        voltage_duty_2 = set_duty_min;
      }
      
      if (currten_duty_2 >voltage_duty_2)
      {
        currten_duty_2 = voltage_duty_2;
      }
      else if (currten_duty_2 < set_duty_min)
      {
        currten_duty_2 = set_duty_min;
      }



      if (voltage_duty_1 > set_duty_max)
      {
        voltage_duty_1 = set_duty_max;
      }
      else if (voltage_duty_1 < set_duty_min)
      {
        voltage_duty_1 = set_duty_min;
      }

      if (currten_duty_1 > voltage_duty_1)
      {
        currten_duty_1 = voltage_duty_1;
      }
      else if(currten_duty_1 < set_duty_min)
      {
        currten_duty_1 = set_duty_min;
      }


      TIM_SetCompare1(TIM1, currten_duty_2 / constant);
      TIM_SetCompare2(TIM1, currten_duty_1 / constant);

      TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    }
}*/


void TIM1_UP_IRQHandler(void)
{
    int16_t dif_value = 0;
    uint16_t duty;
    int32_t last_time_duty;
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
        // 清除�?�?标志
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
    g2_set(0);
    {
        // 在这里添加你的中�?处理代码
        // ...
      /*AD_Value[4];// order   currten_2  voltage_2  current_1  voltage_1*/
      currten_duty_2 += (set_duty_currten_2 - AD_Value[0]);
      //currten_duty_2 += dif_value ;
      voltage_duty_2 += (set_duty_voltage_2 - AD_Value[1]);
      //voltage_duty_2 += dif_value ; 

      /*dif_value = set_duty_currten_1 - AD_Value[2];
      currten_duty_1 += dif_value ;
      dif_value = set_duty_voltage_1 - AD_Value[3];
      voltage_duty_1 += dif_value ; */



      if (voltage_duty_2 > set_duty_max)
      {
        voltage_duty_2 = set_duty_max;
      }
      else if (voltage_duty_2 < set_duty_min)
      {
        voltage_duty_2 = set_duty_min;
      }
      
      if (currten_duty_2 >voltage_duty_2)
      {
        currten_duty_2 = voltage_duty_2;
      }
      else if (currten_duty_2 < set_duty_min)
      {
        currten_duty_2 = set_duty_min;
      }



      /*if (voltage_duty_1 > set_duty_max)
      {
        voltage_duty_1 = set_duty_max;
      }
      else if (voltage_duty_1 < set_duty_min)
      {
        voltage_duty_1 = set_duty_min;
      }

      if (currten_duty_1 > voltage_duty_1)
      {
        currten_duty_1 = voltage_duty_1;
      }
      else if(currten_duty_1 < set_duty_min)
      {
        currten_duty_1 = set_duty_min;
      }*/


      if (currten_duty_2!=last_time_duty)
      { 
      }
        TIM_SetCompare1(TIM1, currten_duty_2 / constant);

      //TIM_SetCompare2(TIM1, currten_duty_1 / constant);


    }
}