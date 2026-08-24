/**
 ******************************************************************************
 * @file    hcsr04.h
 * @brief   HC-SR04 四路超声波驱动（TIM5 输入捕获，非阻塞）
 * @note    引脚分配（对齐《毕设引脚分配表 V2》）：
 *            - 触发 Trig：PE2=前 / PE3=左 / PE4=右 / PE5=后（CubeMX Label HCSR04Q/Z/Y/H）
 *            - 回波 Echo：PA0~PA3 → TIM5_CH1~CH4
 *          驱动使用输入捕获中断测量回波高电平脉宽，d(cm)=脉宽us/58，
 *          超量程/未完成返回 HCSR04_INVALID_CM(0xFFFF)。
 ******************************************************************************
 */
#ifndef __HCSR04_H__
#define __HCSR04_H__

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 传感器索引（前/左/右/后） */
typedef enum
{
  HCSR04_FRONT = 0,
  HCSR04_LEFT  = 1,
  HCSR04_RIGHT = 2,
  HCSR04_BACK  = 3,
  HCSR04_NUM   = 4
} HCSR04_Index_t;

/* 无效/超量程距离标记（与 App 层 SensorData_t 约定一致） */
#define HCSR04_INVALID_CM   0xFFFFu
/* 最大有效量程(cm)，超过视为无效（HC-SR04 标称 2~400cm） */
#define HCSR04_MAX_RANGE_CM 400u

void     HCSR04_Init(void);
void     HCSR04_Trigger(HCSR04_Index_t idx);
uint16_t HCSR04_GetDistanceCm(HCSR04_Index_t idx); /* 最近一次测量，无效返回 0xFFFF */
int32_t  HCSR04_GetPulseUs(HCSR04_Index_t idx);    /* 原始回波脉宽(us)，无效返回 -1 */

#ifdef __cplusplus
}
#endif

#endif /* __HCSR04_H__ */
