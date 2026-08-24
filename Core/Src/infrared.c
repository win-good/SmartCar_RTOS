/**
 ******************************************************************************
 * @file    infrared.c
 * @brief   TCRT5000/HW01 红外避障传感器驱动实现（纯 GPIO 读取）
 ******************************************************************************
 */
#include "infrared.h"

/**
 * @brief  读取指定红外避障模块电平
 * @param  idx IR_LEFT=左前(PE6/HW01Z) / IR_RIGHT=右前(PE7/HW01Y)
 * @retval 1=无障碍 0=有障碍（模块输出低有效）
 * @note   纯 GPIO 读取，无去抖；决策层以 20ms 周期采样，天然滤除瞬时抖动
 */
uint8_t Infrared_Read(Infrared_Index_t idx)
{
  if (idx == IR_LEFT)
  {
    return (uint8_t)HAL_GPIO_ReadPin(HW01Z_GPIO_Port, HW01Z_Pin);
  }
  return (uint8_t)HAL_GPIO_ReadPin(HW01Y_GPIO_Port, HW01Y_Pin);
}

/**
 * @brief  判定指定红外是否检测到障碍（按 IR_ACTIVE_LEVEL 做极性归一化）
 * @note   返回值语义始终为"是否检测到障碍"，与原始电平解耦，
 *         决策层/传感任务直接用它，避免因模块极性不同而误判等级。
 */
uint8_t Infrared_Detected(Infrared_Index_t idx)
{
  uint8_t lvl = Infrared_Read(idx);   /* 1=高 0=低 */
  if (IR_ACTIVE_LEVEL == 1u)
  {
    return lvl;                       /* 遇障碍=高电平 */
  }
  return (uint8_t)(lvl == 0u);        /* 遇障碍=低电平 */
}
