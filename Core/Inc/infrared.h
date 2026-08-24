/**
 ******************************************************************************
 * @file    infrared.h
 * @brief   TCRT5000/HW01 红外避障传感器驱动
 * @note    引脚分配：左前=PE6(HW01Z)、右前=PE7(HW01Y)，GPIO 输入。
 *          输出逻辑：模块遇障碍反射时输出低电平(0)，无障碍输出高电平(1)。
 ******************************************************************************
 */
#ifndef __INFRARED_H__
#define __INFRARED_H__

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  IR_LEFT = 0,
  IR_RIGHT = 1,
  IR_NUM = 2
} Infrared_Index_t;

/* 返回值：1=无障碍 0=有障碍 */
uint8_t Infrared_Read(Infrared_Index_t idx);

#ifdef __cplusplus
}
#endif

#endif /* __INFRARED_H__ */
