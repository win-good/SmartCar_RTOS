/**
 ******************************************************************************
 * @file    gas_sensor.h
 * @brief   MQ-2 烟雾 / MQ-3 酒精气体采集模块接口
 * @note    硬件：MQ-2 → PC0(ADC1_IN10, Rank1)，MQ-3 → PC1(ADC1_IN11, Rank2)
 *          模块 AO 输出经分压后进 ADC（模块 5V 供电、ADC 3.3V 量程）；
 *          CubeMX 已配 ADC1 双通道扫描 + DMA2_Stream0 连续搬运，
 *          本模块只读 DMA 缓冲，零 CPU 占用。
 *          阈值按常见工程经验设定，实车需在洁净空气中校准后调整。
 ******************************************************************************
 */
#ifndef GAS_SENSOR_H
#define GAS_SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 报警阈值（ADC 原始值 0~4095，超过即"错误/超标"）：
 * 洁净空气典型值约数百，1600 ≈ 中段偏上，仅作占位，实测后按模块灵敏度曲线调整 */
#define GAS_MQ2_THRESHOLD   1600u   /* MQ-2 烟雾阈值 */
#define GAS_MQ3_THRESHOLD   1600u   /* MQ-3 酒精阈值 */

/**
 * @brief  启动 ADC1+DMA 连续采集（在 MX_FREERTOS_Init 中、任务创建前调用一次）
 */
void GasSensor_Init(void);

/** @brief  MQ-2 烟雾原始值（0~4095） */
uint16_t GasSensor_GetMQ2(void);

/** @brief  MQ-3 酒精原始值（0~4095） */
uint16_t GasSensor_GetMQ3(void);

#ifdef __cplusplus
}
#endif

#endif /* GAS_SENSOR_H */
