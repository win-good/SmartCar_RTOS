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
