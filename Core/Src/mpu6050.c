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

/**
 * @brief  I2C1 总线自救：9 个 SCL 脉冲 + 手动 STOP + 外设重建
 * @note   与 OLED.c 的 OLED_I2C_BusRecovery 同一套路（2026-08-28 同步引入）：
 *         把 PB6/PB7 临时切普通开漏 GPIO，手动拨 9 个 SCL 脉冲让挂死的从机
 *         吐完剩余位、释放 SDA；再拉高 SDA 补一个 STOP 条件；最后
 *         DeInit/Init 重建 I2C1 外设，清掉 BUSY/ERR 状态。
 *         典型触发场景：按键复位瞬间 MPU6050 正在应答，SDA 残留低电平。
 */
void MPU6050_BusRecovery(void)
{
    GPIO_InitTypeDef gi = {0};

    /* 1) PB6/PB7 切普通开漏输出，CPU 手动接管总线 */
    gi.Mode  = GPIO_MODE_OUTPUT_OD;
    gi.Pull  = GPIO_PULLUP;
    gi.Speed = GPIO_SPEED_FREQ_LOW;
    gi.Pin   = GPIO_PIN_6;
    HAL_GPIO_Init(GPIOB, &gi);
    gi.Pin = GPIO_PIN_7;
    HAL_GPIO_Init(GPIOB, &gi);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);  /* SCL 空闲高 */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);  /* 尝试释放 SDA */

    /* 2) 9 个 SCL 脉冲：从机最多还欠 9 个时钟位，拨满即释放 SDA */
    for (uint8_t i = 0u; i < 9u; i++) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
        for (volatile uint16_t d = 0u; d < 50u; d++) { __NOP(); }
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
        for (volatile uint16_t d = 0u; d < 50u; d++) { __NOP(); }
    }

    /* 3) 手动 STOP：SCL 高期间 SDA 低→高 */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
    for (volatile uint16_t d = 0u; d < 50u; d++) { __NOP(); }
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    for (volatile uint16_t d = 0u; d < 50u; d++) { __NOP(); }
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
    for (volatile uint16_t d = 0u; d < 50u; d++) { __NOP(); }

    /* 4) 外设重建：清 BUSY/ERR 状态，恢复 AF 复用由 MspInit 重配 */
    HAL_I2C_DeInit(&hi2c1);
    HAL_I2C_Init(&hi2c1);
}

/**
 * @brief  任务上下文初始化：先自救总线，再发配置序列
 * @retval 1=成功 0=失败（调用方稍后重试即可，不阻塞系统）
 */
uint8_t MPU6050_TaskInit(void)
{
    MPU6050_BusRecovery();
    return MPU6050_Init();
}
