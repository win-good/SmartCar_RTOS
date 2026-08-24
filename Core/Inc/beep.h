/**
 ******************************************************************************
 * @file    beep.h
 * @brief   无源蜂鸣器驱动接口（TIM4_CH1，PD12，可变发声频率）
 * @note    硬件：无源蜂鸣器模块，【低电平触发】（板载反相驱动管）。
 * 引脚历史：初版用 PB0(TIM8_CH2N)，但底层驱动板 2×22 排针【未引出 PB0】，
 *          按原理图核对排针占用后改接 PD12（TIM4_CH1，排针空闲脚）。
 *          PD12 无其他外设占用；TIM4 全定时器独占给蜂鸣器，频率随意配。
 * 极性约定（低电平触发模块）：
 *          静默 = PD12 常高（CH1 配 PWM 模式2 + CCR=0 实现，模块未触发）；
 *          发声 = 50% 方波（CCR=ARR/2，模块内部反相后仍是方波，无源正常响）。
 * 分档发声频率（对应三级预警"蜂鸣器频率由低到高"）：
 *          一级预警 2kHz（低频）、一级警报 3kHz、2.5档 2.5kHz 滴答音、
 *          二级警报 4kHz 长鸣。无源蜂鸣器谐振多在 2~4kHz，响亮可闻。
 ******************************************************************************
 */
#ifndef __BEEP_H
#define __BEEP_H

#include "main.h"

/* 蜂鸣器硬件引脚：PD12 = TIM4_CH1（排针已引出且空闲） */
#define BEEP_TIM        TIM4
#define BEEP_TIM_CH     TIM_CHANNEL_1
#define BEEP_TIM_CLK_EN __HAL_RCC_TIM4_CLK_ENABLE
#define BEEP_GPIO_Port  GPIOD
#define BEEP_Pin        GPIO_PIN_12
#define BEEP_GPIO_CLK_EN __HAL_RCC_GPIOD_CLK_ENABLE
#define BEEP_AF         GPIO_AF2_TIM4   /* PD12 的 TIM4_CH1 复用 = AF2 */

/* 各级发声频率（Hz），由低到高 */
#define BEEP_FREQ_LEVEL1_HZ   2000u   /* 一级预警：低频间歇 */
#define BEEP_FREQ_LEVEL2_HZ   3000u   /* 一级警报：中频间歇 */
#define BEEP_FREQ_LEVEL25_HZ  2500u   /* 2.5档：滴答变调（2.5k 短音交替） */
#define BEEP_FREQ_LEVEL3_HZ   4000u   /* 二级警报：高频长鸣 */

/**
 * @brief  蜂鸣器初始化：PD12 配为 AF2(TIM4_CH1)，PWM 模式2+CCR=0（静默=常高），
 *         启动 CH1 输出。在 MX_FREERTOS_Init() 中任务创建前调用。
 */
void Beep_Init(void);

/**
 * @brief  以指定频率发声（50% 占空比方波）
 * @param  freq_hz 目标频率，内部按 84MHz(APB1 定时器时钟) 重算 PSC/ARR
 */
void Beep_SetFreq(uint32_t freq_hz);

/**
 * @brief  静默：CCR=0 → PWM 模式2 下 PD12 常高 = 低电平触发模块未触发
 */
void Beep_Off(void);

#endif /* __BEEP_H */
