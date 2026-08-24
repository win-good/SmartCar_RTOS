/**
 ******************************************************************************
 * @file    dht11.c
 * @brief   DHT11 温湿度驱动实现（单总线协议）
 * @note    时序分两段：
 *          - 起始信号的低电平保持 20ms 用 osDelay（任务级，让出 CPU）；
 *          - 响应/数据位阶段（共约 4~5ms）用 DWT 微秒忙等——单总线位时序
 *            无法被任务切换打断，且总时长短、不屏蔽中断，实时性可接受。
 *          互斥量保证任意时刻只有一个任务占用总线。
 ******************************************************************************
 */
#include "dht11.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "cmsis_os2.h"

/* 总线互斥量：DHT11 单总线任意时刻只能一个主机访问 */
static SemaphoreHandle_t s_bus_mutex = NULL;

/* ---------------- DWT 微秒延时（与 hcsr04 同款，模块自包含） ---------------- */
static void DHT11_DelayUs(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000u);
    while ((DWT->CYCCNT - start) < ticks) { }
}

/* ---------------- 总线方向切换 ---------------- */
static void DHT11_BusOutput(void)
{
    GPIO_InitTypeDef g = {0};
    g.Pin   = DHT11_DATA_Pin;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(DHT11_DATA_GPIO_Port, &g);
}

static void DHT11_BusInput(void)
{
    GPIO_InitTypeDef g = {0};
    g.Pin  = DHT11_DATA_Pin;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(DHT11_DATA_GPIO_Port, &g);
}

#define DHT11_READ()  HAL_GPIO_ReadPin(DHT11_DATA_GPIO_Port, DHT11_DATA_Pin)

/* 忙等电平，超时（约 max_us）则 goto 失败；基于 DWT 计数 */
#define WAIT_LEVEL(LEVEL, MAX_US)                                   \
    do {                                                            \
        uint32_t _s = DWT->CYCCNT;                                  \
        uint32_t _t = (MAX_US) * (SystemCoreClock / 1000000u);      \
        while (DHT11_READ() == (LEVEL)) {                           \
            if ((DWT->CYCCNT - _s) > _t) { goto fail; }             \
        }                                                           \
    } while (0)

void DHT11_Init(void)
{
    /* DWT 周期计数器使能（微秒延时基础） */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

    if (s_bus_mutex == NULL) {
        s_bus_mutex = xSemaphoreCreateMutex();
    }
    DHT11_BusOutput();
    HAL_GPIO_WritePin(DHT11_DATA_GPIO_Port, DHT11_DATA_Pin, GPIO_PIN_SET); /* 空闲高 */
}

uint8_t DHT11_Read(uint8_t *temp_c, uint8_t *humi_pct)
{
    uint8_t data[5] = {0};
    uint8_t ok = 0;

    if ((s_bus_mutex == NULL) || (temp_c == NULL) || (humi_pct == NULL)) {
        return 0;
    }
    if (xSemaphoreTake(s_bus_mutex, pdMS_TO_TICKS(20)) != pdTRUE) {
        return 0;   /* 总线被占用，本次放弃 */
    }

    /* ---------- 1. 起始信号：拉低 ≥18ms（用任务延时让出 CPU），再拉高 20~40µs ---------- */
    DHT11_BusOutput();
    HAL_GPIO_WritePin(DHT11_DATA_GPIO_Port, DHT11_DATA_Pin, GPIO_PIN_RESET);
    osDelay(20);
    HAL_GPIO_WritePin(DHT11_DATA_GPIO_Port, DHT11_DATA_Pin, GPIO_PIN_SET);
    DHT11_DelayUs(30);

    /* ---------- 2. 释放总线并等待从机响应（低80µs + 高80µs） ---------- */
    DHT11_BusInput();
    WAIT_LEVEL(GPIO_PIN_SET, 100);     /* 从机应在 20~40µs 内拉低 */
    WAIT_LEVEL(GPIO_PIN_RESET, 100);   /* 从机低 80µs */
    WAIT_LEVEL(GPIO_PIN_SET, 100);     /* 从机高 80µs（准备发数据） */

    /* ---------- 3. 接收 40bit：每位=50µs低 + (26~28µs低→0 / 70µs高→1) ---------- */
    for (uint8_t i = 0; i < 40u; i++) {
        WAIT_LEVEL(GPIO_PIN_RESET, 60);          /* 位起始低电平结束 */
        DHT11_DelayUs(40);                        /* 40µs 后采样：仍高=1，已低=0 */
        if (DHT11_READ() == GPIO_PIN_SET) {
            data[i >> 3] |= (uint8_t)(0x80u >> (i & 0x07u));
            WAIT_LEVEL(GPIO_PIN_SET, 80);         /* 等该位高电平结束 */
        }
    }

    /* ---------- 4. 校验：湿整+湿小+温整+温小 = 校验和 ---------- */
    if (data[4] == (uint8_t)(data[0] + data[1] + data[2] + data[3])) {
        *humi_pct = data[0];
        *temp_c   = data[2];
        ok = 1;
    }

fail:
    /* 恢复空闲高，释放总线 */
    DHT11_BusOutput();
    HAL_GPIO_WritePin(DHT11_DATA_GPIO_Port, DHT11_DATA_Pin, GPIO_PIN_SET);
    xSemaphoreGive(s_bus_mutex);
    return ok;
}
