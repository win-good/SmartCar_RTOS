/**
 ******************************************************************************
 * @file    app_rtos.c
 * @brief   毕设智能小车 FreeRTOS 任务框架 —— 资源创建与任务注册
 * @note    任务规划（优先级/栈/周期见下方创建参数）：
 *            TaskSensor   AboveNormal  512字  20ms  传感器采集
 *            TaskDecision Normal       512字  20ms  状态机+三级预警决策
 *            TaskMotor    AboveNormal  256字  10ms  TB6612 电机控制
 *            TaskBt       Normal       512字  事件  蓝牙指令解析
 *            TaskK230     Normal       512字  事件  K230 视觉行协议解析
 *            TaskRadar    Normal       512字  事件  LD2450 雷达目标帧解析
 *            TaskDisplay  Low          512字  50ms  LED/蜂鸣器/OLED（含开机版本横幅）
 *          CubeMX 中已删除 defaultTask，所有任务由本文件统一创建。
 ******************************************************************************
 */
#include "app_rtos.h"

/* ============================ 全局数据实例 ============================ */
SensorData_t g_sensor;
Decision_t   g_decision;

osMessageQueueId_t q_bt_cmd   = NULL;

/* ============================ 任务句柄与属性 ============================ */
osThreadId_t t_sensor   = NULL;
osThreadId_t t_decision = NULL;
osThreadId_t t_motor    = NULL;
osThreadId_t t_bt       = NULL;
osThreadId_t t_k230     = NULL;
osThreadId_t t_radar    = NULL;   /* LD2450 雷达（2026-08-28 新增） */
osThreadId_t t_display  = NULL;

/* 任务入口函数声明（实现见 app_tasks.c） */
extern void TaskSensor_Start(void *argument);
extern void TaskDecision_Start(void *argument);
extern void TaskMotor_Start(void *argument);
extern void TaskBt_Start(void *argument);
extern void TaskK230_Start(void *argument);
extern void TaskRadar_Start(void *argument);
extern void TaskDisplay_Start(void *argument);

/* ============================ 初始化 ============================ */
/**
 * @brief  创建消息队列与全部任务
 * @note   在 freertos.c 的 MX_FREERTOS_Init() 中、
 *         “USER CODE BEGIN RTOS_MUTEX” 区域内调用
 */
void App_Init(void)
{
    /* ---------- 共享数据默认值 ---------- */
    g_sensor.dist_front_cm = 0xFFFF;
    g_sensor.dist_left_cm  = 0xFFFF;
    g_sensor.dist_right_cm = 0xFFFF;
    g_sensor.dist_back_cm  = 0xFFFF;
    g_sensor.ir_left       = 1;   /* 默认无障碍 */
    g_sensor.ir_right      = 1;
    g_sensor.radar_present = 0;   /* 雷达默认无目标 */
    g_sensor.radar_dist_cm = 0xFFFF;
    g_sensor.vis_target_seen = 0; /* 视觉默认无目标 */
    g_sensor.vis_dist_cm   = 0xFFFF;  /* 【必须0xFFFF】0 会被当成"距离0cm"误触发四级 */
    g_sensor.fusion_dist_cm = 0xFFFF;
    g_sensor.fusion_valid  = 0;
    g_sensor.update_tick   = 0;

    /* 2026-08-29：上电默认遥控模式3，初始静止不动；
     * 按按键1/2 或发 MA/MB 才进入自主避障模式（自主模式自动巡航前进）。 */
    g_decision.mode          = MODE_BLUETOOTH;
    g_decision.alert_level   = ALERT_NONE;
    g_decision.target_speed_l = 0;
    g_decision.target_speed_r = 0;
    g_decision.motor_enabled = 0;            /* 传感器就绪前保持制动 */

    /* ---------- 指令队列 ---------- */
    q_bt_cmd   = osMessageQueueNew(8, sizeof(AppCmd_t), NULL);  /* 蓝牙指令，深度8 */
    /* 2026-08-29：原 q_k230_cmd 队列从未使用（K230 视觉直写 g_sensor），已删除 */

    /* ---------- 创建任务 ---------- */
    const osThreadAttr_t attr_sensor = {
        .name = "TaskSensor", .stack_size = 512 * 4,
        .priority = (osPriority_t)osPriorityAboveNormal,
    };
    t_sensor = osThreadNew(TaskSensor_Start, NULL, &attr_sensor);

    const osThreadAttr_t attr_decision = {
        .name = "TaskDecision", .stack_size = 512 * 4,
        .priority = (osPriority_t)osPriorityNormal,
    };
    t_decision = osThreadNew(TaskDecision_Start, NULL, &attr_decision);

    const osThreadAttr_t attr_motor = {
        .name = "TaskMotor", .stack_size = 256 * 4,
        .priority = (osPriority_t)osPriorityAboveNormal,
    };
    t_motor = osThreadNew(TaskMotor_Start, NULL, &attr_motor);

    const osThreadAttr_t attr_bt = {
        .name = "TaskBt", .stack_size = 512 * 4,
        .priority = (osPriority_t)osPriorityNormal,
    };
    t_bt = osThreadNew(TaskBt_Start, NULL, &attr_bt);

    const osThreadAttr_t attr_k230 = {
        .name = "TaskK230", .stack_size = 512 * 4,
        .priority = (osPriority_t)osPriorityNormal,
    };
    t_k230 = osThreadNew(TaskK230_Start, NULL, &attr_k230);

    /* LD2450 毫米波雷达任务（2026-08-28 新增）：
     * USART6 256000 中断接收 → 解析目标帧 → 写 g_sensor.radar_*；
     * 雷达数据只参与预警分级与模式2融合，不直接驱动电机。 */
    const osThreadAttr_t attr_radar = {
        .name = "TaskRadar", .stack_size = 512 * 4,
        .priority = (osPriority_t)osPriorityNormal,
    };
    t_radar = osThreadNew(TaskRadar_Start, NULL, &attr_radar);

    const osThreadAttr_t attr_display = {
        .name = "TaskDisplay", .stack_size = 512 * 4,
        .priority = (osPriority_t)osPriorityLow,
    };
    t_display = osThreadNew(TaskDisplay_Start, NULL, &attr_display);
}

/* ============================ 独立看门狗（2026-08-24 新增） ============================
 * 目的：整机"死机自愈"。旧版任何一处死循环（NMI/CSS、Error_Handler、任务死锁）
 *       都只能断电恢复；现在由 IWDG 在超时后自动复位整机。
 * 选型：IWDG（独立看门狗），时钟来自 LSI（约32kHz），与系统时钟/HSE 无关，
 *       即使 HSE 起振失败、PLL 没配上，看门狗依旧工作——覆盖最坏场景。
 *
 * 【重要】超时不可太短！
 *   App_Watchdog_Init() 在 main.c 里 HAL_Init() 之后立即启动看门狗，但此时
 *   FreeRTOS 尚未启动，唯一喂狗点（TaskDisplay）还没运行。从启动到 TaskDisplay
 *   首次喂狗，要经历 SystemClock_Config + 全部外设 MX_*_Init + MX_FREERTOS_Init
 *   （内含 OLED_Init / MPU6050_Init / HCSR04_Init 等）。若 OLED 未接好，其重试
 *   与总线自救可能使初始化耗时数秒。若超时设得太短（如 2.7s），初始化还没完成
 *   就被看门狗复位 → 反复复位，表现为"程序卡死、所有外设无反应"。
 *   因此超时需覆盖最坏初始化耗时：取 20s（预分频 256 → 125Hz，Reload 2500）。
 *   正常运行 TaskDisplay 每 50ms 喂一次，20s 裕量 400 倍，绝不会误触发。 */
static IWDG_HandleTypeDef s_hiwdg;

void App_Watchdog_Init(void)
{
    s_hiwdg.Instance       = IWDG;
    s_hiwdg.Init.Prescaler = IWDG_PRESCALER_256;     /* LSI 256 分频 → 125Hz */
    s_hiwdg.Init.Reload    = 2500u;                  /* 2500/125Hz = 20s 超时 */
    if (HAL_IWDG_Init(&s_hiwdg) == HAL_OK)
    {
        HAL_IWDG_Refresh(&s_hiwdg);                  /* 初始喂一次，从满周期开始计 */
    }
    /* 初始化失败不致命：最坏情况等于没有看门狗，不影响其他功能 */
}

void App_Watchdog_Feed(void)
{
    (void)HAL_IWDG_Refresh(&s_hiwdg);
}
