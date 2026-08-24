/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define HCSR04Q_Pin GPIO_PIN_2
#define HCSR04Q_GPIO_Port GPIOE
#define HCSR04Z_Pin GPIO_PIN_3
#define HCSR04Z_GPIO_Port GPIOE
#define HCSR04Y_Pin GPIO_PIN_4
#define HCSR04Y_GPIO_Port GPIOE
#define HCSR04H_Pin GPIO_PIN_5
#define HCSR04H_GPIO_Port GPIOE
#define HW01Z_Pin GPIO_PIN_6
#define HW01Z_GPIO_Port GPIOE
#define HW01Y_Pin GPIO_PIN_7
#define HW01Y_GPIO_Port GPIOE
#define DHT11_DATA_Pin GPIO_PIN_8
#define DHT11_DATA_GPIO_Port GPIOE
#define AIN1_Pin GPIO_PIN_9
#define AIN1_GPIO_Port GPIOE
#define AIN2_Pin GPIO_PIN_10
#define AIN2_GPIO_Port GPIOE
#define BIN1_Pin GPIO_PIN_11
#define BIN1_GPIO_Port GPIOE
#define BIN2_Pin GPIO_PIN_12
#define BIN2_GPIO_Port GPIOE
#define STBY_Pin GPIO_PIN_13
#define STBY_GPIO_Port GPIOE
#define LED2_Pin GPIO_PIN_14
#define LED2_GPIO_Port GPIOE
#define LED3_Pin GPIO_PIN_15
#define LED3_GPIO_Port GPIOE
#define LED1_Pin GPIO_PIN_5
#define LED1_GPIO_Port GPIOB

/* ============================ 模式按键（PD0/PD1/PD2，低电平触发，内部上拉） ============================
 * PD0/1/2 在 STM32F407VGT6 上为 FSMC D2/D3/D4，未被现有外设占用，开发板两排 22pin 引脚上
 * 通常引出，相邻便于接线。PD 端口已在 gpio.c 中使能时钟。
 * KEY_MODE1_Pin → MODE_NORMAL     按下接地 → 切换到模式1（普通避障）
 * KEY_MODE2_Pin → MODE_FUSION     按下接地 → 切换到模式2（融合避障）
 * KEY_MODE3_Pin → MODE_BLUETOOTH   按下接地 → 切换到模式3（蓝牙遥控） */
#define KEY_MODE1_Pin       GPIO_PIN_0
#define KEY_MODE1_GPIO_Port GPIOD
#define KEY_MODE2_Pin       GPIO_PIN_1
#define KEY_MODE2_GPIO_Port GPIOD
#define KEY_MODE3_Pin       GPIO_PIN_2
#define KEY_MODE3_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
