/**
 ******************************************************************************
 * @file    app_rtos.c
 * @brief   毕设智能小车 FreeRTOS 任务框架 —— 资源创建与任务注册
 * @note    任务规划（优先级/栈/周期见下方创建参数）：
 *            TaskSensor   AboveNormal  512字  20ms  传感器采集
 *            TaskDecision Normal       512字  20ms  状态机+三级预警决策
 *            TaskMotor    AboveNormal  256字  10ms  TB6612 电机控制
 *            TaskBt       Normal       512字  事件  蓝牙指令解析
 *            TaskK230     Normal       512字  事件  K230 协议解析
 *            TaskDisplay  Low          512字  50ms  LED/蜂鸣器/OLED
 *          CubeMX 中已删除 defaultTask，所有任务由本文件统一创建。
 ******************************************************************************
 */
#include "app_rtos.h"

/* ============================ 全局数据实例 ============================ */
SensorData_t g_sensor;
Decision_t   g_decision;

osMessageQueueId_t q_bt_cmd   = NULL;
osMessageQueueId_t q_k230_cmd = NULL;

/* ============================ 任务句柄与属性 ============================ */
osThreadId_t t_sensor   = NULL;
osThreadId_t t_decision = NULL;
osThreadId_t t_motor    = NULL;
osThreadId_t t_bt       = NULL;
osThreadId_t t_k230     = NULL;
osThreadId_t t_display  = NULL;

/* 任务入口函数声明（实现见 app_tasks.c） */
extern void TaskSensor_Start(void *argument);
extern void TaskDecision_Start(void *argument);
extern void TaskMotor_Start(void *argument);
extern void TaskBt_Start(void *argument);
extern void TaskK230_Start(void *argument);
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
    g_sensor.ir_left       = 0;   /* 默认无红外接触（1=检出障碍） */
    g_sensor.ir_right      = 0;
    g_sensor.mq2_ok        = 1;   /* 默认无气体超标（否则全局零初始化=0，上电即误报三级） */
    g_sensor.mq3_ok        = 1;
    g_sensor.gas_ok        = 0;   /* 气体 ADC 需预热，有效前屏蔽气体报警 */
    g_sensor.update_tick   = 0;

    g_decision.mode          = MODE_IDLE;    /* 上电默认：电机停止、仅预警，需 BT/按键切换模式 */
    g_decision.alert_level   = ALERT_NONE;
    g_decision.target_speed_l = 0;
    g_decision.target_speed_r = 0;
    g_decision.motor_enabled = 0;            /* 传感器就绪前保持制动 */

    /* ---------- 指令队列 ---------- */
    q_bt_cmd   = osMessageQueueNew(8, sizeof(AppCmd_t), NULL);  /* 蓝牙指令，深度8 */
    q_k230_cmd = osMessageQueueNew(8, sizeof(AppCmd_t), NULL);  /* K230 指令，深度8 */

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

    const osThreadAttr_t attr_display = {
        .name = "TaskDisplay", .stack_size = 1024 * 3,  /* 3KB：OLED_Update 每页 Buf[129] + 显示缓冲 + OLED_Init 调用栈 */
        .priority = (osPriority_t)osPriorityLow,
    };
    t_display = osThreadNew(TaskDisplay_Start, NULL, &attr_display);
}
