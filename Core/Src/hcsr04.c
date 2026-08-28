/**
 ******************************************************************************
 * @file    hcsr04.c
 * @brief   HC-SR04 四路超声波驱动实现
 * @note    测距流程（由 App 传感任务调用）：
 *           1) HCSR04_Trigger(idx)：拉高触发脚 ≥10us（DWT 微秒延时）
 *           2) TIM5 输入捕获中断测量回波脉宽（见 HAL_TIM_IC_CaptureCallback）
 *           3) HCSR04_GetDistanceCm(idx)：换算 cm
 *          每次 Trigger 会先清空该通道状态，避免残留回波造成误读。
 ******************************************************************************
 */
#include "hcsr04.h"
#include "tim.h"

/* ============================ 触发脚映射 ============================ */
static GPIO_TypeDef *const s_trig_port[HCSR04_NUM] = {
    HCSR04Q_GPIO_Port,   /* 前 */
    HCSR04Z_GPIO_Port,   /* 左 */
    HCSR04Y_GPIO_Port,   /* 右 */
    HCSR04H_GPIO_Port,   /* 后 */
};
static const uint16_t s_trig_pin[HCSR04_NUM] = {
    HCSR04Q_Pin,
    HCSR04Z_Pin,
    HCSR04Y_Pin,
    HCSR04H_Pin,
};

/* 回波捕获通道：PA0~PA3 → TIM5_CH1~CH4 */
static const uint32_t s_echo_ch[HCSR04_NUM] = {
    TIM_CHANNEL_1,
    TIM_CHANNEL_2,
    TIM_CHANNEL_3,
    TIM_CHANNEL_4,
};

/* ============================ 通道状态机 ============================ */
typedef enum
{
  HC_IDLE = 0,   /* 等待上升沿 */
  HC_RISING,     /* 已捕获上升沿，等待下降沿 */
  HC_MEASURED    /* 一次完整测量完成 */
} HCSR04_State_t;

typedef struct
{
  volatile HCSR04_State_t state;
  volatile uint32_t start_us;  /* 上升沿时刻 */
  volatile uint32_t width_us;  /* 回波高电平脉宽(us) */
} HCSR04_Chan_t;

static HCSR04_Chan_t s_chan[HCSR04_NUM];

/* ============================ 微秒延时（DWT） ============================ */
static void DWT_Init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0u;
  DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

static void delay_us(uint32_t us)
{
  uint32_t start = DWT->CYCCNT;
  uint32_t ticks = us * (SystemCoreClock / 1000000u);
  while ((DWT->CYCCNT - start) < ticks) { }
}

/* ============================ 输入捕获回调 ============================ */
/* 由 HAL 在调用回调前写入 htim->Channel（HAL_TIM_ACTIVE_CHANNEL_x 位掩码） */
static uint8_t ActiveChannelToIndex(uint32_t active_ch)
{
  switch (active_ch)
  {
    case HAL_TIM_ACTIVE_CHANNEL_1: return HCSR04_FRONT;
    case HAL_TIM_ACTIVE_CHANNEL_2: return HCSR04_LEFT;
    case HAL_TIM_ACTIVE_CHANNEL_3: return HCSR04_RIGHT;
    case HAL_TIM_ACTIVE_CHANNEL_4: return HCSR04_BACK;
    default:                       return HCSR04_NUM;   /* 非法 */
  }
}

/**
 * @brief  TIM5 输入捕获中断回调（由 stm32f4xx_it.c 的 TIM5_IRQHandler 触发）
 * @note   上升沿记录起点并切到下降沿；下降沿算出脉宽并切回上升沿。
 *         32 位计数器回绕由“差值”自动处理。
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim != &htim5)
  {
    return;
  }

  uint8_t idx = ActiveChannelToIndex(htim->Channel);
  if (idx >= HCSR04_NUM)
  {
    return;
  }
  uint32_t ch = s_echo_ch[idx];
  uint32_t cnt = HAL_TIM_ReadCapturedValue(&htim5, ch);

  if (s_chan[idx].state == HC_IDLE)
  {
    s_chan[idx].start_us = cnt;
    s_chan[idx].state = HC_RISING;
    __HAL_TIM_SET_CAPTUREPOLARITY(&htim5, ch, TIM_INPUTCHANNELPOLARITY_FALLING);
  }
  else if (s_chan[idx].state == HC_RISING)
  {
    s_chan[idx].width_us = cnt - s_chan[idx].start_us;
    s_chan[idx].state = HC_MEASURED;
    __HAL_TIM_SET_CAPTUREPOLARITY(&htim5, ch, TIM_INPUTCHANNELPOLARITY_RISING);
  }
}

/* ============================ GPIO 轮询测距（回退方案，2026-08-28） ============================
 * 背景：HC-SR04 的 Echo 输出为 5V 电平。PA0/PA1 对 5V 信号勉强可识别，
 *       但 PA2/PA3 遇到超过 VDD(3.3V) 的输入会被内部钳位卡死在低电平附近，
 *       导致 TIM5_CH3/CH4 输入捕获等不到完整边沿 → 右/后两路永远无数据。
 * 方案：Echo 引脚本身就是 GPIO（PA0~PA3），改用 GPIO 输入读电平+DWT 计时测脉宽：
 *       Trigger 拉高后先等回波上升沿，再计高电平宽度，30ms 超时判超量程。
 *       此路径不依赖 TIM5 输入捕获；原捕获通道保持工作互不干扰。
 * 注意：函数内含忙等，最坏约 30ms，仅允许在传感任务中逐帧调用。 */
#define HCSR04_POLL_TIMEOUT_US  30000u   /* HC-SR04 最大量程回波约 25~38ms */

static int32_t HCSR04_PollMeasure(HCSR04_Index_t idx)
{
  GPIO_TypeDef *port = GPIOA;                      /* 四路回波均在 PA0~PA3 */
  uint16_t pin = (uint16_t)(1u << idx);

  /* 等待上升沿（模块响应延迟通常 <1ms；5ms 未见回波=无模块/接线错误，及时放弃） */
  uint32_t t0 = DWT->CYCCNT;
  while (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_RESET)
  {
    if ((DWT->CYCCNT - t0) > 5u * (SystemCoreClock / 1000000u))
    {
      return -1;
    }
  }

  /* 计时高电平脉宽，30ms 超时（超量程/无回波保护） */
  t0 = DWT->CYCCNT;
  while (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET)
  {
    if ((DWT->CYCCNT - t0) > HCSR04_POLL_TIMEOUT_US * (SystemCoreClock / 1000000u))
    {
      return -1;
    }
  }
  return (int32_t)((DWT->CYCCNT - t0) / (SystemCoreClock / 1000000u));
}

/* ============================ 对外接口 ============================ */
/**
 * @brief  初始化：使能 DWT 微秒延时，启动 4 路输入捕获中断
 */
void HCSR04_Init(void)
{
  DWT_Init();
  for (uint8_t i = 0u; i < HCSR04_NUM; i++)
  {
    s_chan[i].state = HC_IDLE;
    s_chan[i].start_us = 0u;
    s_chan[i].width_us = 0u;
    HAL_TIM_IC_Start_IT(&htim5, s_echo_ch[i]);
  }
}

/**
 * @brief  触发指定通道测距（≥10us 高电平脉冲）
 * @note   每次触发前清空状态，丢弃上一帧残留
 */
void HCSR04_Trigger(HCSR04_Index_t idx)
{
  if (idx >= HCSR04_NUM)
  {
    return;
  }

  s_chan[idx].state = HC_IDLE;
  HAL_GPIO_WritePin(s_trig_port[idx], s_trig_pin[idx], GPIO_PIN_SET);
  delay_us(12u);
  HAL_GPIO_WritePin(s_trig_port[idx], s_trig_pin[idx], GPIO_PIN_RESET);

  /* 2026-08-28：GPIO 轮询同步完成本帧测量（绕开 PA2/PA3 捕获失效）。
   * 捕获通道路径仍并行工作：若捕获先完成已置 HC_MEASURED，轮询读到的是
   * 下降沿后的低电平，返回 -1，不会覆盖捕获的有效值。 */
  int32_t us = HCSR04_PollMeasure(idx);
  if (us >= 0)
  {
    s_chan[idx].width_us = (uint32_t)us;
    s_chan[idx].state = HC_MEASURED;
  }
}

/**
 * @brief  获取原始回波脉宽(us)
 * @retval 有效脉宽；若该帧未完成/超量程返回 -1
 */
int32_t HCSR04_GetPulseUs(HCSR04_Index_t idx)
{
  if (idx >= HCSR04_NUM)
  {
    return -1;
  }
  if (s_chan[idx].state != HC_MEASURED)
  {
    return -1;
  }
  return (int32_t)s_chan[idx].width_us;
}

/**
 * @brief  获取距离(cm)，声速 340m/s 往返：d(cm) = 脉宽us / 58
 * @retval 有效距离；无效/超量程返回 HCSR04_INVALID_CM(0xFFFF)
 */
uint16_t HCSR04_GetDistanceCm(HCSR04_Index_t idx)
{
  int32_t us = HCSR04_GetPulseUs(idx);
  if (us < 0)
  {
    return HCSR04_INVALID_CM;
  }

  uint32_t cm = (uint32_t)us / 58u;
  if (cm > HCSR04_MAX_RANGE_CM)
  {
    return HCSR04_INVALID_CM;
  }
  return (uint16_t)cm;
}
