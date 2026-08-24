/**
 ******************************************************************************
 * @file    beep.c
 * @brief   无源蜂鸣器驱动实现（TIM4_CH1 / PD12 可变频率，低电平触发极性适配）
 * @note    换脚原因：PB0 未从 2×22 排针引出；按开发板原理图核对 P2 排针
 *          占用后选定 PD12（空闲脚，TIM4_CH1 = AF2；TIM4 无其他用途，独占）。
 *
 * 时钟推导：APB1 预分频=4 → 定时器时钟 = 42MHz×2 = 84MHz；
 *           PSC=83 → 计数时钟 1MHz；ARR = 1MHz/f - 1。
 *           2kHz→ARR499；2.5kHz→ARR399；3kHz→ARR332(≈3003Hz)；4kHz→ARR249。
 * 极性：CH1 配 PWM 模式2（CNT<CCR 输出低，CNT≥CCR 输出高）：
 *           CCR=0     → 恒高 = 静默（低电平触发模块未触发）；
 *           CCR=ARR/2 → 50% 方波 = 发声。
 * 本文件手动配置 TIM4（CubeMX 未启用 TIM4），全部位于用户代码层，
 * 不依赖 .ioc 重生成。
 ******************************************************************************
 */
#include "beep.h"

#define BEEP_PSC        83u
#define BEEP_CNT_CLK_HZ 1000000u

static TIM_HandleTypeDef s_htim4;   /* 蜂鸣器专用 TIM4 句柄（手动配置） */

/**
 * @brief  蜂鸣器初始化：GPIO(PD12, AF2) + TIM4(PWM 模式2) + 静默启动
 */
void Beep_Init(void)
{
    /* ---------- 时钟 ---------- */
    BEEP_TIM_CLK_EN();
    BEEP_GPIO_CLK_EN();

    /* ---------- GPIO：PD12 = TIM4_CH1 ---------- */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = BEEP_Pin;
    gpio.Mode  = GPIO_MODE_AF_PP;      /* 推挽输出，模块侧有上拉/驱动管 */
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = BEEP_AF;
    HAL_GPIO_Init(BEEP_GPIO_Port, &gpio);

    /* ---------- TIM4 基础：1MHz 计数 ---------- */
    s_htim4.Instance               = BEEP_TIM;
    s_htim4.Init.Prescaler         = BEEP_PSC;
    s_htim4.Init.CounterMode       = TIM_COUNTERMODE_UP;
    s_htim4.Init.Period            = 499u;   /* 上电默认 2kHz 档 */
    s_htim4.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    s_htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&s_htim4) != HAL_OK) {
        return;   /* 失败则保持静默，不让蜂鸣器问题拖垮系统 */
    }

    /* ---------- CH1：PWM 模式2 + 预装载，CCR=0(静默常高) ---------- */
    TIM_OC_InitTypeDef oc = {0};
    oc.OCMode     = TIM_OCMODE_PWM2;       /* 极性关键：CCR=0 时输出恒高 */
    oc.Pulse      = 0u;                    /* 上电静默 */
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&s_htim4, &oc, BEEP_TIM_CH);

    HAL_TIM_PWM_Start(&s_htim4, BEEP_TIM_CH);
}

/**
 * @brief  以指定频率发声（50% 占空比，影子寄存器平滑切换）
 */
void Beep_SetFreq(uint32_t freq_hz)
{
    if (freq_hz == 0u) {
        Beep_Off();
        return;
    }
    uint32_t arr = (BEEP_CNT_CLK_HZ / freq_hz) - 1u;
    if (arr < 99u)     arr = 99u;
    if (arr > 0xFFFFu) arr = 0xFFFFu;

    __HAL_TIM_SET_AUTORELOAD(&s_htim4, arr);
    __HAL_TIM_SET_COMPARE(&s_htim4, BEEP_TIM_CH, arr / 2u);  /* 50% 方波=发声 */
}

/**
 * @brief  静默：CCR=0 → PWM 模式2 输出恒高 = 模块未触发
 */
void Beep_Off(void)
{
    __HAL_TIM_SET_COMPARE(&s_htim4, BEEP_TIM_CH, 0u);
}

/**
 * @brief  按预警等级播放/静默蜂鸣器
 * @param  level 发声等级 BEEP_LVL1~BEEP_LVL3
 * @param  on    1=发声  0=静默
 */
void Beep_Play(Beep_Level_t level, uint8_t on)
{
    if (!on)
    {
        Beep_Off();
        return;
    }

    switch (level)
    {
        case BEEP_LVL1:  Beep_SetFreq(BEEP_FREQ_LEVEL1_HZ);   break;  /* 2kHz  一级预警 */
        case BEEP_LVL2:  Beep_SetFreq(BEEP_FREQ_LEVEL2_HZ);   break;  /* 3kHz  一级警报 */
        case BEEP_LVL25: Beep_SetFreq(BEEP_FREQ_LEVEL25_HZ);  break;  /* 2.5kHz 滴答 */
        case BEEP_LVL3:  Beep_SetFreq(BEEP_FREQ_LEVEL3_HZ);   break;  /* 4kHz  二级警报 */
        default:         Beep_Off();                          break;
    }
}
