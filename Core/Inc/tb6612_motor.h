#ifndef __TB6612_MOTOR_H__
/**
 ******************************************************************************
 * @file    tb6612_motor.h
 * @brief   TB6612FNG 双路电机驱动接口
 * @note    速度有两种口径：
 *            - SetSpeed/SetLeft/SetRight 及便捷动作：原始 TIM 计数 0~8400，负值反转
 *            - SetSpeedPercent：±100 百分比（应用层统一使用）
 ******************************************************************************
 */
#define __TB6612_MOTOR_H__

#include "main.h"

typedef enum
{
  TB6612_MOTOR_STOP = 0,
  TB6612_MOTOR_FORWARD,
  TB6612_MOTOR_BACKWARD,
  TB6612_MOTOR_BRAKE
} TB6612_MotorDirection;

HAL_StatusTypeDef TB6612_Motor_Init(void);
void TB6612_Motor_SetLeft(int16_t speed);
void TB6612_Motor_SetRight(int16_t speed);
void TB6612_Motor_SetSpeed(int16_t left_speed, int16_t right_speed);
void TB6612_Motor_SetSpeedPercent(int16_t left_pct, int16_t right_pct);
void TB6612_Motor_Stop(void);
void TB6612_Motor_Brake(void);
void TB6612_Motor_Forward(uint16_t speed);
void TB6612_Motor_Backward(uint16_t speed);
void TB6612_Motor_TurnLeft(uint16_t speed);
void TB6612_Motor_TurnRight(uint16_t speed);

#endif /* __TB6612_MOTOR_H__ */
