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
