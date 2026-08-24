/**
 ******************************************************************************
 * @file    dht11.h
 * @brief   DHT11 温湿度传感器驱动接口（单总线，PE8）
 * @note    PE8 在 CubeMX 中配为推挽输出、初始高电平（总线空闲态）；
 *          收发时动态切换输入/输出。任务上下文调用，内置互斥量，
 *          两次读取间隔须 ≥1s（DHT11 采样周期限制），由调用方限频。
 ******************************************************************************
 */
#ifndef DHT11_H
#define DHT11_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief  初始化：使能 DWT 周期计数器（微秒延时）+ 创建总线互斥量
 * @note   在 MX_FREERTOS_Init 中、任务创建前调用一次
 */
void DHT11_Init(void);

/**
 * @brief  读取一次温湿度（约 5ms，含 20ms 起始信号中的任务级延时）
 * @param  temp_c  输出：温度 ℃（0~50）
 * @param  humi_pct 输出：湿度 %RH（20~90）
 * @retval 1=成功（校验和通过） 0=失败（超时/校验错，输出值不更新含义由调用方自决）
 * @note   必须在任务上下文调用；多线程安全（互斥量串行化总线访问）
 */
uint8_t DHT11_Read(uint8_t *temp_c, uint8_t *humi_pct);

#ifdef __cplusplus
}
#endif

#endif /* DHT11_H */
