/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    dma.c
  * @brief   This file provides code for the configuration
  *          of all the requested memory to memory DMA transfers.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "dma.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure DMA                                                              */
/*----------------------------------------------------------------------------*/

/* USER CODE BEGIN 1 */
DMA_HandleTypeDef hdma_i2c2_tx;
/* 配置 I2C2_TX：DMA1_Stream7 + Channel7（F407 固定映射），Memory→Peripheral 字节宽度。
   CubeMX 未勾选 I2C2 DMA，此手动补全；重新生成代码不丢（USER CODE 区）。
   链接到 hi2c2 的动作在 i2c.c 的 HAL_I2C_MspInit 中完成。 */
void OLED_DMA_Init(void)
{
  hdma_i2c2_tx.Instance                 = DMA1_Stream7;
  hdma_i2c2_tx.Init.Channel             = DMA_CHANNEL_7;
  hdma_i2c2_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
  hdma_i2c2_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
  hdma_i2c2_tx.Init.MemInc              = DMA_MINC_ENABLE;
  hdma_i2c2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_i2c2_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
  hdma_i2c2_tx.Init.Mode                = DMA_NORMAL;
  hdma_i2c2_tx.Init.Priority            = DMA_PRIORITY_MEDIUM;
  hdma_i2c2_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
  if (HAL_DMA_Init(&hdma_i2c2_tx) != HAL_OK)
  {
    Error_Handler();
  }
  /* DMA1_Stream7 中断：优先级 5（FreeRTOS 可管理外设中断必须 ≥5） */
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);

  /* 【关键】I2C2_TX 在 DMA1 上，必须使能 DMA1 时钟，否则 Stream7 根本不工作，
     DMA 传输永远不完成 —— OLED 黑屏的软件根因之一。
     （MX_DMA_Init 由 CubeMX 生成只管 DMA2/ADC，此处补全，重生成不丢。） */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* I2C2 事件/错误中断：HAL 的 DMA 发送完成回调依赖这两个中断，必须使能，
     同属 OLED DMA 方案根因修复；优先级 5 保持 FreeRTOS 可管理。 */
  HAL_NVIC_SetPriority(I2C2_EV_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(I2C2_EV_IRQn);
  HAL_NVIC_SetPriority(I2C2_ER_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(I2C2_ER_IRQn);
}
/* USER CODE END 1 */

/* USER CODE END 1 */

/**
  * Enable DMA controller clock
  */
void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */

