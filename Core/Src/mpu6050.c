/**
 ******************************************************************************
 * @file    mpu6050.c
 * @brief   MPU6050 六轴姿态驱动实现（轮询 I2C1，任务上下文调用）
 * @note    寄存器要点：
 *          0x6B PWR_MGMT_1 写 0x00 唤醒；
 *          0x1B GYRO_CONFIG  0x08 → ±500°/s（灵敏度 65.5 LSB/(°/s)）；
 *          0x1C ACCEL_CONFIG 0x00 → ±2g；
 *          0x43~0x4A 角速度原始数据（GYRO_YOUT_H 起，大端）。
 * 航向积分：yaw += gz * dt；静止（|gz|<0.5°/s 且调用方判定停车）不清零，
 *          清零由上层在"出发前/停车时"显式调用 MPU6050_ResetYaw()。
 * 轮询模式：每次事务 ≤ 几 ms，20ms 周期任务可接受；不占用中断资源。
 ******************************************************************************
 */
#include "mpu6050.h"
#include "i2c.h"

#define MPU_ADDR_8BIT   0xD0u   /* 7bit 0x68 << 1 */
#define MPU_REG_PWR     0x6Bu
#define MPU_REG_GYRO_CF 0x1Bu
#define MPU_REG_ACC_CF  0x1Cu
#define MPU_REG_GYRO_YH 0x47u   /* 只用 Y 轴（安装方式：Y 轴垂直车体=转向轴） */
#define MPU_TIMEOUT_MS  10u

static int32_t s_yaw_deg10 = 0;   /* 累积偏航 ×10 */

static uint8_t MPU_WriteReg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return (HAL_I2C_Master_Transmit(&hi2c1, MPU_ADDR_8BIT, buf, 2,
                                    MPU_TIMEOUT_MS) == HAL_OK) ? 1u : 0u;
}

static uint8_t MPU_ReadRegs(uint8_t reg, uint8_t *buf, uint8_t len)
{
    return (HAL_I2C_Master_Transmit(&hi2c1, MPU_ADDR_8BIT, &reg, 1,
                                    MPU_TIMEOUT_MS) == HAL_OK &&
            HAL_I2C_Master_Receive(&hi2c1, MPU_ADDR_8BIT, buf, len,
                                   MPU_TIMEOUT_MS) == HAL_OK) ? 1u : 0u;
}

uint8_t MPU6050_Init(void)
{
    if (!MPU_WriteReg(MPU_REG_PWR, 0x00u))     return 0u;  /* 唤醒 */
    HAL_Delay(10);
    if (!MPU_WriteReg(MPU_REG_GYRO_CF, 0x08u)) return 0u;  /* ±500°/s */
    if (!MPU_WriteReg(MPU_REG_ACC_CF, 0x00u))  return 0u;  /* ±2g */
    s_yaw_deg10 = 0;
    return 1u;
}

uint8_t MPU6050_Update(uint32_t dt_ms, int16_t *out_yaw_deg10, int16_t *out_gz_dps10)
{
    uint8_t raw[2];
    if (!MPU_ReadRegs(MPU_REG_GYRO_YH, raw, 2)) {
        return 0u;
    }
    int16_t gyro_raw = (int16_t)((raw[0] << 8) | raw[1]);
    /* ±500°/s → 65.5 LSB/(°/s)；×10 放大避免小数：gz_dps10 = raw*10/65.5 ≈ raw*100/655 */
    int32_t gz_dps10 = ((int32_t)gyro_raw * 100) / 655;

    /* 积分：yaw_deg10 += gz(°/s)×10 × dt(s) = gz_dps10 × dt_ms / 100 */
    s_yaw_deg10 += (gz_dps10 * (int32_t)dt_ms) / 100;
    if (s_yaw_deg10 >  1800) s_yaw_deg10 =  1800;
    if (s_yaw_deg10 < -1800) s_yaw_deg10 = -1800;

    *out_yaw_deg10 = (int16_t)s_yaw_deg10;
    *out_gz_dps10  = (int16_t)gz_dps10;
    return 1u;
}

void MPU6050_ResetYaw(void)
{
    s_yaw_deg10 = 0;
}
