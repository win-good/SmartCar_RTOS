/**
 ******************************************************************************
 * @file    mpu6050.h
 * @brief   MPU6050 六轴姿态驱动接口（I2C1：PB6=SCL / PB7=SDA，地址 0x68）
 * @note    硬件核对：开发板原理图中 I2C1(PB6/PB7) 仅被板载 EEPROM 占用，
 *          与 EEPROM(0xA0) 地址不冲突，可并联挂载 MPU6050(0xD0)。
 * 用途：直线行走航向保持——陀螺仪 Z 轴积分得累积偏航角，
 *          偏航超过 3%(≈3.6°/120°行程口径，代码取 3°) 时差速修正 TB6612。
 * 简化方案（不上 DMP）： gyro Z 积分 + 静止自动清零，满足直行纠偏需求。
 ******************************************************************************
 */
#ifndef __MPU6050_H
#define __MPU6050_H

#include "main.h"
#include <stdint.h>

/**
 * @brief  初始化：唤醒 MPU6050，陀螺仪 ±500°/s、加速度 ±2g
 * @retval 1=成功 0=失败（I2C 无应答）
 */
uint8_t MPU6050_Init(void);

/**
 * @brief  读取一次数据并更新内部航向积分
 * @param  dt_ms 距上次调用的间隔（ms）
 * @param  out_yaw_deg10  累积偏航角 ×10（右转为正）
 * @param  out_gz_dps10   Z 轴角速度 ×10（°/s ×10）
 * @retval 1=读取成功 0=失败（数据保持上次值）
 */
uint8_t MPU6050_Update(uint32_t dt_ms, int16_t *out_yaw_deg10, int16_t *out_gz_dps10);

/**
 * @brief  航向角清零（静止锚定/出发前调用）
 */
void MPU6050_ResetYaw(void);

#endif /* __MPU6050_H */
