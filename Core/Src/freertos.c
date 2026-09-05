/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "tb6612_motor.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "hcsr04.h"
#include "app_rtos.h"
#include "OLED.h"
#include "gas_sensor.h"
#include "dht11.h"
#include "beep.h"
#include "mpu6050.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* 注：defaultTask 已移除，全部任务由 App_Init() 统一创建（见 App/app_rtos.c） */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  if (TB6612_Motor_Init() != HAL_OK)
  {
    Error_Handler();
  }

  /* 超声波输入捕获初始化（须在 MX_TIM5_Init 之后） */
  HCSR04_Init();

  /* OLED 初始化（I2C2 已由 MX_I2C2_Init 配置，内部含上电延时与清屏） */
  OLED_Init();

  /* 气体 ADC 连续采集（ADC1+DMA 循环搬运，零 CPU 占用） */
  GasSensor_Init();

  /* DHT11 初始化（DWT 微秒延时 + 总线互斥量，总线置空闲高） */
  DHT11_Init();

  /* 无源蜂鸣器初始化（TIM8_CH2N 可变频率，PB0 重配，静默=常高） */
  Beep_Init();

  /* MPU6050 初始化（I2C1；失败不致命，mpu_ok=0 时航向修正自动禁用） */
  /* 2026-08-28：MPU6050 初始化移出启动序列，改由 TaskSensor 异步完成。
   * 原因：外接 MPU6050 模块若接线/上电异常，启动前同步初始化会把问题
   * 放大成"整机起不来、OLED 无显示"；异步后 MPU 异常只禁用航向修正
   * (mpu_ok=0)，其余功能照常，且传感任务会周期重试自愈（见 app_tasks.c）。 */

  /* 创建 App 层全部任务（传感/决策/电机/蓝牙/K230/显示） */
  App_Init();

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* USER CODE BEGIN RTOS_THREADS */
  /* 线程已由 App_Init() 统一创建 */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* ============================ 任务栈溢出钩子（2026-09-05 改为非冻结） ============================
 * 上一版(09-04)钩子里用 taskDISABLE_INTERRUPTS()+死循环"冻结现场"，
 * 结果一旦触发整机停摆：电机停在短路制动态(嗡鸣不转)、OLED 定格、
 * TaskDisplay 没机会运行（承诺的 OVF 显示永远打不出来），只能等看门狗复位。
 * 现改为【非冻结】：仅记录任务名并置位，立即返回让系统继续跑；
 * 由 TaskDisplay 在 OLED 上打印 "OVF:<任务名>" 并蜂鸣提示，方便定位。
 * 注意：当前 FreeRTOSConfig.h 中 configCHECK_FOR_STACK_OVERFLOW=0（检测关闭），
 *       本钩子不会被调用，仅作为将来重新开启检测时的安全实现保留。
 * 约束：钩子运行在中断禁止上下文，禁止调用 RTOS API 与阻塞函数。 */
volatile char g_stack_overflow_task[16] = "";   /* 溢出的任务名（空串=未溢出） */

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    /* 记录任务名（手动拷贝，不依赖字符串库） */
    for (uint8_t i = 0u; i < 15u; i++) {
        g_stack_overflow_task[i] = pcTaskName[i];
        if (pcTaskName[i] == '\0') break;
    }
    g_stack_overflow_task[15] = '\0';
    /* 直接返回，不冻结系统；故障提示交给 TaskDisplay 显示 */
}

/* USER CODE END Application */

