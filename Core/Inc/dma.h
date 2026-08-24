/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    dma.h
  * @brief   This file contains all the function prototypes for
  *          the dma.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DMA_H__
#define __DMA_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* DMA memory to memory transfer handles -------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_DMA_Init(void);
void OLED_DMA_Init(void);              /* I2C2_TX DMA 通道配置（OLED 非阻塞传输用） */
extern DMA_HandleTypeDef hdma_i2c2_tx; /* I2C2_TX DMA 句柄 */

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __DMA_H__ */

