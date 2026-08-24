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

/* 红外模块输出极性适配宏：
 * 多数 HW01/TCRT5000 模块遇障碍反射时 DO 输出低电平（集电极开路），故 0=有障碍。
 * 若实车发现"物体贴住红外仍不触发/只触发低档"，说明该模块为有障碍输出高电平，
 * 把本宏改为 1 即可让决策层把高电平判为有障碍，无需改逻辑代码。 */
#ifndef IR_ACTIVE_LEVEL
#define IR_ACTIVE_LEVEL  0u   /* 0：遇障碍=低电平；1：遇障碍=高电平 */
#endif

/**
 * @brief  判定指定红外是否检测到障碍（已按 IR_ACTIVE_LEVEL 做极性归一化）
 * @retval 1=检测到障碍  0=无障碍
 */
uint8_t Infrared_Detected(Infrared_Index_t idx);

#ifdef __cplusplus
}
#endif

#endif /* __INFRARED_H__ */
