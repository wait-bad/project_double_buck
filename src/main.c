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
#include "Serial.h"
//#include "Serial.c"


#define OPTPARSE_API static
#include "optparse.h"
#include <string.h>
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
int32_t constant         = 8192;//constant = 0.0001;
uint16_t set_Reload_value = 609; //  freq = 177k
uint16_t set_duty_max_value = 609;
uint16_t set_duty_min_value = 1;
uint16_t voltage_limit     = 23;//v
uint16_t current_limit      = 5;//a


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

uint8_t commmand_first_5[5];

// flag
uint16_t tim_add_counter = 0;
uint16_t refresh_flag    = 0;
//uint16_t Serial_RxFlag   = 0;
//

/************************/
uint16_t  test__ = 0;
uint8_t    test_flag = 0;
uint8_t RxData;
uint8_t* array;

/******************************************/
       //function
void RCCCLOCK_Init(void);
uint8_t system_all_init();
void Timer_Init(void);
void TIM1_Configuration(void);
void GPIO_Configuration(void);
void count_un_ctrl_parameter();
void show_decimals(char dir, char x, unsigned char y, unsigned long datab);
void set_duty_max__(uint16_t duty_max);

void set_set_current_1(float current);
void set_set_current_2(float current);
void set_set_voltage_1(float voltage);
void set_set_voltage_2(float voltage);



void control_output_b();
void control_output_a();
void uart_NVIC_Configuration(void);
uint16_t uart_count_parameters(uint8_t flag , uint8_t* array);
/*******************************************/

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
    TIM_BDTRInitStructure.TIM_DeadTime        = 1; // 死区时间设置
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


// Initialize USART1
void uart_init(u32 bound) {
    //GPIO端口设置
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);	//使能USART1，GPIOA时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_USART1, ENABLE);
//	//USART1_TX   GPIOA.9
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
//  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.9
//
//  //USART1_RX	  GPIOA.10初始化
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
//  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.10
    /* TX PB6 */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /*  RX PB7 */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    //Usart1 NVIC 配置
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1 ; //抢占优先级3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//子优先级3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
    NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器

    //USART 初始化设置

    USART_InitStructure.USART_BaudRate = bound;//串口波特率
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
    USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

    USART_Init(USART1, &USART_InitStructure); //初始化串口1
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启串口接受中断
    USART_Cmd(USART1, ENABLE);                    //使能串口1

}

// Configure NVIC for USART1
void uart_NVIC_Configuration(void) {
    NVIC_InitTypeDef NVIC_InitStructure;

    // Enable the USART1 interrupt
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // Enable the USART1 global interrupt
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
}


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
    //uart_NVIC_Configuration();
    //USART1_Init();
    uart_init(115200);
    //Serial_Init();
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
    uint8_t i;
    system_all_init();
    set_duty_max__(500);
    set_current_1 = 5.5;
    set_current_2 = 2;
    set_voltage_1 = 5.25;
    set_voltage_2 = 8.45;

    count_un_ctrl_parameter();
    printf("HELLO_WORLD");
    while (9) 
    {
        Delay_ms(100);
        //Serial_SendString("hello world\n");
        uart_count_parameters(Serial_GetRxFlag(),receive_data_send_back());
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

void control_output_b()
{
  /*AD_Value[4];// order   currten_2  voltage_2  current_1  voltage_1*/
  currten_duty_2 += (set_duty_currten_2 - AD_Value[0]);
  voltage_duty_2 += (set_duty_voltage_2 - AD_Value[1]);

  // 限幅电压和电流
  voltage_duty_2 = (voltage_duty_2 > set_duty_max) ? set_duty_max : ((voltage_duty_2 < set_duty_min) ? set_duty_min : voltage_duty_2);
  currten_duty_2 = (currten_duty_2 > voltage_duty_2) ? voltage_duty_2 : ((currten_duty_2 < set_duty_min) ? set_duty_min : currten_duty_2);

  TIM_SetCompare1(TIM1, currten_duty_2>>13);

}

void control_output_a()
{
  /*AD_Value[4];// order   currten_2  voltage_2  current_1  voltage_1*/
  currten_duty_1 += set_duty_currten_1 - AD_Value[2];
  voltage_duty_1 += set_duty_voltage_1 - AD_Value[3];

  // 限幅电压和电流
  voltage_duty_1 = (voltage_duty_1 > set_duty_max) ? set_duty_max : ((voltage_duty_1 < set_duty_min) ? set_duty_min : voltage_duty_1);
  currten_duty_1 = (currten_duty_1 > voltage_duty_1) ? voltage_duty_1 : ((currten_duty_1 < set_duty_min) ? set_duty_min : currten_duty_1);

  TIM_SetCompare2(TIM1, currten_duty_1>>13);
}

void set_duty_max__(uint16_t duty_max)
{
  set_duty_max = duty_max;
}


uint16_t uart_count_parameters(uint8_t flag, uint8_t* array)
{
    char array_command[7];
    char array_settings[6];
    float numebr;
    if (flag == 1)
    {
        // 将array的前6个字符复制到array_command中
        strncpy(array_command, (char*)array,6);
        array_command[6] = '\0';  // 添加字符串结尾符

        // 将array的后5个字符复制到array_settings中
        strncpy(array_settings, (char*)(array + 6), 5);
        array_settings[5] = '\0';  // 添加字符串结尾符
        numebr = atoi(array_settings);

        // 根据命令类型执行相应的操作
        if (strcmp(array_command, "set_1v") == 0) {
            // 执行设置1V的操作
            printf("Setting 1V with parameters: %0.2fV\n", numebr/100);
        } else if (strcmp(array_command, "set_2v") == 0) {
            // 执行设置2V的操作
            printf("Setting 2V with parameters: %0.2fV\n", numebr/100);
        } else if (strcmp(array_command, "set_1a") == 0) {
            // 执行设置1A的操作
            printf("Setting 1A with parameters: %0.3fA\n", numebr/1000);
        } else if (strcmp(array_command, "set_2a") == 0) {
            // 执行设置2A的操作
            printf("Setting 2A with parameters: %0.3fA\n", numebr/1000);
        } else if (strcmp(array_command, "hello_") == 0) {
            // 执行设置2A的操作
            printf("hello my friend,wish you have a good day.\n");
            printf("    *****   \n");
            printf("  *       *  \n");
            printf(" *  O   O  * \n");
            printf(" *    ^    * \n");
            printf("  *  \\__/  *  \n");
            printf("   *     *   \n");
            printf("    *****    \n");
        } else if (strcmp(array_command, "limit_") == 0) {
            // 执行设置2A的操作
            printf("voltage limit = %d\nV" , voltage_limit);
            printf("current limit = %d\nA" , current_limit);
        }
        else {
            // 未知命令
            printf("Unknown command: %s\n", array_command);
        }
        Serial_setflag_0();
    }
    return 0; // 这里需要根据实际情况返回一个值
}

void set_set_current_1(float current)
{
    if (current<= 0){
        current = 0;
    }
    else if(current>current_limit){
        current = current_limit;
    }
    set_current_1 = current ;
}
void set_set_current_2(float current)
{
    if (current<= 0){
        current = 0;
    }
    else if(current>current_limit){
        current = current_limit;
    }
    set_current_2 = current ;
}

void set_set_voltage_1(float voltage)
{
    if (voltage <= 0){
        voltage = 0;
    }
    else if (voltage> voltage_limit)
    {
        voltage = voltage_limit;
    }
    set_voltage_1 = voltage;
}

void set_set_voltage_2(float voltage)
{
    if (voltage <= 0){
        voltage = 0;
    }
    else if (voltage> voltage_limit)
    {
        voltage = voltage_limit;
    }
    set_voltage_2 = voltage;
}

void TIM1_UP_IRQHandler(void)
{
    int16_t dif_value = 0;
    uint16_t duty;
    int32_t last_time_duty;
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
    {
      // 清除中断标志
      TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
      g2_set(0);

      control_output_b();
      control_output_a();
    }
}

