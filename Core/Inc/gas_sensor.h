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

/* 是否启用气体传感器报警（默认关闭）：
 * 上电/移除模块后 ADC 引脚可能读到漂移值，阈值若过低会误触发三档制动；
 * 本毕设三档报警以"红外贴障"为主，气体报警默认关闭更稳。
 * 需要气体报警时改为 1，并在洁净空气实测 baseline 后微调阈值。 */
#ifndef GAS_ALARM_ENABLE
#define GAS_ALARM_ENABLE    0u
#endif

/* 报警阈值（ADC 原始值 0~4095，超过即"错误/超标"）：
 * 12bit ADC 下 MQ 洁净空气典型值约 1800~2500（因模块 RL 不同可能 800~3500）。
 * 原阈值 1600 偏低，洁净空气下就可能触发 → 误报三档。
 * 此处取 3000（需实际超标明显才报警），并配合连续多帧确认去抖，见 app_tasks.c。 */
#define GAS_MQ2_THRESHOLD   3000u   /* MQ-2 烟雾阈值 */
#define GAS_MQ3_THRESHOLD   3000u   /* MQ-3 酒精阈值 */

/* 气体超标确认去抖：连续 N 帧（×20ms）超阈值才判定超标，滤除上电/偶发抖动 */
#define GAS_CONFIRM_FRAMES  10u     /* 10 帧 ×20ms = 200ms */

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
