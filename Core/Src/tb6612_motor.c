/**
 ******************************************************************************
 * @file    tb6612_motor.c
 * @brief   TB6612FNG 双路电机驱动实现
 * @note    硬件连接（对齐《毕设引脚分配表 V2》）：
 *            - 左电机方向：AIN1=PE9 / AIN2=PE10，速度 PWM = TIM3_CH1(PA6)
 *            - 右电机方向：BIN1=PE11 / BIN2=PE12，速度 PWM = TIM3_CH2(PA7)
 *            - 使能 STBY = PE13（高电平使能芯片）
 *          控制原理：IN1/IN2 决定转向（10=正转、01=反转、00=滑行停止、
 *          11+满占空比=短路制动），PWM 占空比决定转速。
 *          速度约定：应用层用 ±100 百分比，本文件内部换算为 TIM 比较值。
 ******************************************************************************
 */
#include "tb6612_motor.h"
#include "tim.h"

#define TB6612_PWM_MAX ((uint16_t)__HAL_TIM_GET_AUTORELOAD(&htim3))

/**
 * @brief  速度限幅：把 int16 速度收敛到 ±TB6612_PWM_MAX，返回其绝对值（占空比计数）
 * @note   负数越界时返回满量程而非截断为 0，保证"超速=全速"而不是"超速=停转"
 */
static uint16_t TB6612_ClampSpeed(int16_t speed)
{
  if (speed > (int16_t)TB6612_PWM_MAX)
  {
    return TB6612_PWM_MAX;
  }
  if (speed < -(int16_t)TB6612_PWM_MAX)
  {
    return TB6612_PWM_MAX;
  }
  return (uint16_t)(speed < 0 ? -speed : speed);
}

/**
 * @brief  单侧电机控制（方向 GPIO + 速度 PWM 一次写齐）
 * @param  speed 有符号速度：>0 正转(IN1=1,IN2=0)，<0 反转(IN1=0,IN2=1)，=0 滑行停止
 * @param  pwm_channel 该侧对应的 TIM3 PWM 通道
 * @note   先写方向再写占空比，避免方向未定就先给速度的瞬间误动作
 */
static void TB6612_SetSide(int16_t speed,
                           GPIO_TypeDef *in1_port, uint16_t in1_pin,
                           GPIO_TypeDef *in2_port, uint16_t in2_pin,
                           uint32_t pwm_channel)
{
  GPIO_PinState in1_state = GPIO_PIN_RESET;
  GPIO_PinState in2_state = GPIO_PIN_RESET;
  uint16_t duty = TB6612_ClampSpeed(speed);

  if (speed > 0)
  {
    in1_state = GPIO_PIN_SET;
  }
  else if (speed < 0)
  {
    in2_state = GPIO_PIN_SET;
  }

  HAL_GPIO_WritePin(in1_port, in1_pin, in1_state);
  HAL_GPIO_WritePin(in2_port, in2_pin, in2_state);
  __HAL_TIM_SET_COMPARE(&htim3, pwm_channel, duty);
}

/**
 * @brief  电机初始化：拉高 STBY 使能芯片 → 启动 TIM3 CH1/CH2 两路 PWM → 停机
 * @retval HAL_OK=成功；任一路 PWM 启动失败则提前返回错误码
 * @note   在 MX_FREERTOS_Init() 中、任务创建前调用，保证任务一开始就能安全写电机
 */
HAL_StatusTypeDef TB6612_Motor_Init(void)
{
  HAL_StatusTypeDef status;

  HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_SET);
  status = HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  if (status != HAL_OK)
  {
    return status;
  }
  status = HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  if (status != HAL_OK)
  {
    return status;
  }
  TB6612_Motor_Stop();
  return HAL_OK;
}

/* 左电机：AIN1/AIN2 + TIM3_CH1 */
void TB6612_Motor_SetLeft(int16_t speed)
{
  TB6612_SetSide(speed, AIN1_GPIO_Port, AIN1_Pin, AIN2_GPIO_Port, AIN2_Pin,
                 TIM_CHANNEL_1);
}

/* 右电机：BIN1/BIN2 + TIM3_CH2 */
void TB6612_Motor_SetRight(int16_t speed)
{
  TB6612_SetSide(speed, BIN1_GPIO_Port, BIN1_Pin, BIN2_GPIO_Port, BIN2_Pin,
                 TIM_CHANNEL_2);
}

/* 同时设置左右电机（有符号原始计数速度） */
void TB6612_Motor_SetSpeed(int16_t left_speed, int16_t right_speed)
{
  TB6612_Motor_SetLeft(left_speed);
  TB6612_Motor_SetRight(right_speed);
}

/**
 * @brief  按百分比设定左右电机速度（应用层统一用 ±100 表示）
 * @param  left_pct  左电机：-100（全速倒车）~ +100（全速前进）
 * @param  right_pct 右电机：同上
 * @note   内部换算为 TIM 计数（ARR 由 CubeMX 定，此处动态读取保证一致）
 */
void TB6612_Motor_SetSpeedPercent(int16_t left_pct, int16_t right_pct)
{
  int32_t l = ((int32_t)left_pct * (int32_t)TB6612_PWM_MAX) / 100;
  int32_t r = ((int32_t)right_pct * (int32_t)TB6612_PWM_MAX) / 100;
  TB6612_Motor_SetLeft((int16_t)l);
  TB6612_Motor_SetRight((int16_t)r);
}

/**
 * @brief  滑行停止：两路 IN 全 0 + 占空比 0（电机自由停转，惯性滑行）
 * @note   日常"停车/待命"用本函数；需要立刻停住用 TB6612_Motor_Brake()
 */
void TB6612_Motor_Stop(void)
{
  TB6612_Motor_SetSpeed(0, 0);
}

/**
 * @brief  短路制动：两路 IN 全 1 + 满占空比，电机绕组短路产生反电动势阻力
 * @note   用于三级预警紧急停车，制动距离明显短于 Stop()
 */
void TB6612_Motor_Brake(void)
{
  HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, TB6612_PWM_MAX);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, TB6612_PWM_MAX);
}

/* 便捷动作：双轮同速正转=直行前进（speed 为原始计数 0~ARR） */
void TB6612_Motor_Forward(uint16_t speed)
{
  TB6612_Motor_SetSpeed((int16_t)speed, (int16_t)speed);
}

/* 便捷动作：双轮同速反转=直行后退 */
void TB6612_Motor_Backward(uint16_t speed)
{
  TB6612_Motor_SetSpeed(-(int16_t)speed, -(int16_t)speed);
}

/* 便捷动作：左轮反转+右轮正转=原地左转 */
void TB6612_Motor_TurnLeft(uint16_t speed)
{
  TB6612_Motor_SetSpeed(-(int16_t)speed, (int16_t)speed);
}

/* 便捷动作：左轮正转+右轮反转=原地右转 */
void TB6612_Motor_TurnRight(uint16_t speed)
{
  TB6612_Motor_SetSpeed((int16_t)speed, -(int16_t)speed);
}
