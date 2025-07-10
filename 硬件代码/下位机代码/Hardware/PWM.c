#include "stm32f10x.h"
#include "Delay.h"

void PWM_Init(void)
{
    /* 开启时钟 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);  // 开启TIM3时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); // 开启GPIOB时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);  // 开启AFIO时钟（重映射需要）

    /* GPIO初始化：PB0和PB1复用推挽输出 */
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;        // 复用推挽输出
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1; // PB0和PB1
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* 配置TIM3时基单元 */
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;     // 时钟分频
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; // 向上计数
    TIM_TimeBaseInitStructure.TIM_Period = 20000 - 1;              // 自动重装载值ARR
    TIM_TimeBaseInitStructure.TIM_Prescaler = 72 - 1;              // 预分频器PSC
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure); // 初始化TIM3

    /* 配置PWM通道3（PB0）和通道4（PB1） */
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCStructInit(&TIM_OCInitStructure);               // 初始化结构体默认值
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;     // PWM模式1
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; // 高电平有效
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; // 使能输出

    // 初始化通道3（PB0）
    TIM_OCInitStructure.TIM_Pulse = 0;                    // 初始占空比0
    TIM_OC3Init(TIM3, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);     // 使能预装载

    // 初始化通道4（PB1）
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OC4Init(TIM3, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);

    /* 启动TIM3 */
    TIM_Cmd(TIM3, ENABLE);
    TIM_CtrlPWMOutputs(TIM3, ENABLE); // 高级定时器需要，普通定时器可省略
}

/* 设置通道3（PB0）的脉宽 */
void PWM_SetPulseWidth_CH3(uint16_t pulse_us)
{
    if (pulse_us < 800) pulse_us = 800;
    else if (pulse_us > 2200) pulse_us = 2200;
    TIM_SetCompare3(TIM3, pulse_us);
}

/* 设置通道4（PB1）的脉宽 */
void PWM_SetPulseWidth_CH4(uint16_t pulse_us)
{
    if (pulse_us < 800) pulse_us = 800;
    else if (pulse_us > 2200) pulse_us = 2200;
    TIM_SetCompare4(TIM3, pulse_us);
}

/* 急停函数（双通道回归中位） */
void Motor_Stop(void)
{
    PWM_SetPulseWidth_CH3(1500);
    PWM_SetPulseWidth_CH4(1500);
    Delay_ms(100);
}