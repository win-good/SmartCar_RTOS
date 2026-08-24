/**
 ******************************************************************************
 * @file    gas_sensor.c
 * @brief   MQ-2 烟雾 / MQ-3 酒精气体采集模块实现
 * @note    ADC1 配为双通道扫描、DMA 连续请求后，转换结果由硬件自动写入
 *          s_adc_buf[0]=MQ-2 / s_adc_buf[1]=MQ-3，无需轮询、无阻塞。
 ******************************************************************************
 */
#include "gas_sensor.h"
#include "adc.h"   /* hadc1 */

/* ADC 双通道 DMA 缓冲：[0]=IN10(MQ-2)，[1]=IN11(MQ-3) */
static uint16_t s_adc_buf[2] = {0u, 0u};

void GasSensor_Init(void)
{
    /* 循环 DMA：ADC 每完成一轮双通道扫描自动把结果写回缓冲，持续刷新 */
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)s_adc_buf, 2);
}

uint16_t GasSensor_GetMQ2(void)
{
    return s_adc_buf[0];
}

uint16_t GasSensor_GetMQ3(void)
{
    return s_adc_buf[1];
}
