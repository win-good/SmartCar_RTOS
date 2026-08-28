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

/* ============================ 电机调试宏区（2026-08-28 抽出） ============================
 * 实车需要调的电机参数全部集中在这里：改数值→重新编译即可，不必翻其他代码。
 * 速度单位：占空比百分比 ±100（0=停，100=满速）。
 * 四只直流电机精度不高、启动阈值/摩擦阻力不一致，调试顺序建议：
 *   1) 先把 MOTOR_SPEED_CRUISE_PCT 调到车能平稳直走的最低值再+10 余量；
 *   2) 直行跑偏用 MOTOR_TRIM_L/R_PCT 配平（偏左=左轮慢→加大 L 或减小 R）；
 *   3) 倒车/转弯档位按手感微调。 */
#define MOTOR_SPEED_SLOW_PCT    35   /* 后退 / 一级预警减速直行 */
#define MOTOR_SPEED_CRUISE_PCT  50   /* 巡航直行速度 */
#define MOTOR_SPEED_TURN_PCT    40   /* 避障转向差速 */
#define MOTOR_SPEED_UTURN_PCT   45   /* 死胡同掉头速度 */
#define MOTOR_TRIM_L_PCT         0   /* 左轮配平补偿：正=左轮加速 */
#define MOTOR_TRIM_R_PCT         2   /* 右轮配平补偿：正=右轮加速 */

/* 速度指令为 0 时的停机方式：
 *   1 = 短路制动（TB6612 IN1=IN2=1，绕组短路锁轴：初始待命/停车时车体定住，
 *       手推不动、坡道不溜车）——推荐，解决"上电电机不静止"观感；
 *   0 = 滑行停止（IN 全 0，轮子可自由转动/手推滑行）。 */
#define MOTOR_STOP_USE_BRAKE     1

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
