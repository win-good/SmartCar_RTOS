/**
 ******************************************************************************
 * @file    app_tasks.c
 * @brief   毕设智能小车 FreeRTOS 任务框架 —— 六个任务的实现
 * @author  STM32F407VGT6 + FreeRTOS (CMSIS-RTOS2 封装) + K230 视觉板
 *
 * @details
 * 【预警四级体系】（高级覆盖低级，LED/蜂鸣器/OLED 互斥只显最高级）
 *   一级预警 ：前超声波 <1m       → 减速；绿灯亮；蜂鸣器 2kHz 100ms 间歇
 *   一级警报 ：前超声波 <30cm     → 对比左右超声波转向；黄灯亮；3kHz 150ms 间歇
 *   2.5 级   ：左前红外触发(<10cm)→ 黄灯亮 + 2.5kHz 滴答变调（双音短促）
 *                               +→ 自动转向 / 死胡同掉头
 *   二级警报 ：右前红外(<1cm) 或烟雾/酒精超阈值
 *             → 红灯亮 + 4kHz 长鸣 + 任何模式速度强制清零
 *
 *   传感器分工：
 *     - 4 路 HCSR04 超声波 → 负责 1/2 级（旋转轮询各路距离）
 *     - 左红外(2.5 级电位器≈10cm) / 右红外(3 级电位器≈1cm) → 红外避级
 *     - 烟雾 MQ-2 / 酒精 MQ-3 → 可选二级警报（GAS_ALARM_ENABLE 宏开关）
 *
 * 【响应时序（"上快下慢"非对称）】
 *   - 升级（检测到障碍）→ 1 帧（20ms）即确认；
 *   - 降级（移开障碍）→ 4 帧（80ms）连续保持更低等级才允许降，
 *     体验上感觉"移开约 1 秒内停止蜂鸣"，与用户需求一致。
 *
 * 【六任务框架】（优先级、栈、周期）
 *   TaskSensor   AboveNormal  2KB   20ms   轮询4 路超声波 + 红外去抖 + 气体 + DHT11 + MPU6050
 *   TaskDecision Normal       2KB   20ms   BT/K230 指令解析 + 预警分级（迟滞）+ 模式规划
 *   TaskMotor    AboveNormal  1KB   10ms   TB6612 电机调速（motor_enabled 门控）
 *   TaskBt       Normal       2KB   5ms    USART1 中断逐字节入环形缓冲 → 30ms 静默后解析
 *   TaskK230     Normal       2KB   10ms   USART3 视觉协议（骨架）
 *   TaskDisplay  Low          3KB   50ms   LED 互斥 + 蜂鸣器分档 + OLED 全屏重绘
 *
 * 【蓝牙协议】（JDY-31，USART1 9600 N81）
 *   "MA" / "MB" / "MC"   → 切换模式 1/2/3
 *   "W" / "S" / "A" / "D" → 遥控前后左右（仅模式3 有效）
 *   "U" → 掉头          "X" → 停止
 *   "TH"→ 查询温湿度（板端立即回传 "[TH] T:xxC H:xx%\r\n"）
 *   "RST"→ 软件复位（NVIC_SystemReset 全系统重启）
 *   每条指令解析后立即回传 "[OK] XXX\r\n" 给手机，便于确认指令已收到；
 *   解析器忽略空格/Tab/换行等空白，兼容手机按钮控制模式（按下时尾部带 "\n    "）。
 *
 * 【按键模式切换】（PD0 / PD1 / PD2，内部上拉，低电平触发）
 *   KEY_MODE1（PD0）→ MODE_NORMAL     按下接地 → 模式1（普通避障）
 *   KEY_MODE2（PD1）→ MODE_FUSION     按下接地 → 模式2（融合避障）
 *   KEY_MODE3（PD2）→ MODE_BLUETOOTH  按下接地 → 模式3（蓝牙遥控）
 *
 * 【直行保持】MPU6050 航向积分，直线段偏航 >3° 时差速修正 TB6612。
 *
 * 【默认行为】MODE_IDLE（电机不动，仅传感器预警），需 BT/按键切换才自主移动。
 ******************************************************************************
 */
#include "app_rtos.h"
#include "hcsr04.h"
#include "infrared.h"
#include "tb6612_motor.h"
#include "OLED.h"
#include "gas_sensor.h"
#include "dht11.h"
#include "beep.h"
#include "mpu6050.h"
#include "usart.h"   /* huart1：JDY-31 蓝牙 */
#include <string.h>
#include <stdio.h>

/* ============================ 私有定义 ============================ */
#define DIST_VALID_MAX_CM   300u

/* 预警阈值（cm，2026-08-23 实测定稿） */
#define TH_WARN_CM          100u  /* 一级预警：前超声波 <1m → 减速        */
#define TH_ALARM_CM         30u   /* 一级警报：前超声波 <30cm → 转向避让  */
/* 2.5 级 / 3 级由红外二值触发（硬件电位器定距：左≈10cm / 右≈1cm），无软件阈值 */
#define TH_CONTACT_CM       8u    /* 接触兜底：前超声波 ≤8cm 视为贴障，强制三档  */
#define TH_DEADEND_CM       20u   /* 死胡同判定：左/右/后均 <20cm → 掉头   */

/* 红外去抖：连续 N 帧检测到障碍才确认，滤除瞬时抖动/噪声 */
#define IR_DEBOUNCE_FRAMES  1u    /* 1 帧 ×20ms = 20ms 即确认（用户要求"减少识别延迟"） */

/* 预警降级迟滞：等级下降（移开障碍）需连续 N 帧保持更低等级才确认，
 * 避免抖动/回弹反复闪灯/蜂鸣。上升（检测到障碍）保持即时响应 */
#define ALERT_HOLD_FRAMES   4u    /* 4 帧 ×20ms = 80ms 才允许降级（缩短到接近用户感受的"1 秒"内） */

/* 直行航向修正：偏航超过该值(0.1°)开始差速修正（3° = 30） */
#define YAW_CORRECT_TH_DEG10  30
#define YAW_CORRECT_GAIN      5    /* 每 1° 偏航补偿 5% 差速 */
#define YAW_CORRECT_MAX       20   /* 补偿上限 ±20% */

/* 速度档（±100 占空比百分比） */
#define SPEED_SLOW          30
#define SPEED_CRUISE        50
#define SPEED_TURN          40
#define SPEED_UTURN         45

/* ============================ 蓝牙接收（中断字节 → 环形缓冲） ============================ */
static uint8_t  s_bt_rx_byte;                 /* 中断接收单字节缓冲 */
static uint8_t  s_bt_ring[64];                /* 环形缓冲 */
static volatile uint16_t s_bt_rd = 0, s_bt_wr = 0;

/* UART 接收完成回调（USART1 中断上下文，优先级 5 满足 FreeRTOS 要求） */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        uint16_t next = (uint16_t)((s_bt_wr + 1u) & 0x3Fu);
        if (next != s_bt_rd) {                 /* 满则丢弃最旧，防溢出崩溃 */
            s_bt_ring[s_bt_wr] = s_bt_rx_byte;
            s_bt_wr = next;
        }
        HAL_UART_Receive_IT(&huart1, &s_bt_rx_byte, 1);   /* 续接下一字节 */
    }
}

static int Bt_RingGet(void)
{
    if (s_bt_rd == s_bt_wr) return -1;
    uint8_t c = s_bt_ring[s_bt_rd];
    s_bt_rd = (uint16_t)((s_bt_rd + 1u) & 0x3Fu);
    return (int)c;
}

/* ============================ 预警分级 ============================ */
/**
 * @brief  四级预警计算（超声波管 1/2 级，红外管 2.5/3 级，气体→3 级）
 * @note   只返回当前最高级；显示层互斥，低级别不再残留点亮。
 */
static uint8_t CalcAlertLevel(const SensorData_t *s)
{
    /* 传感器未就绪（尚无有效数据）时一律视为无预警，避免上电即误报。
     * update_tick==0 表示传感任务还没跑满一帧。 */
    if (s->update_tick == 0u) {
        return ALERT_NONE;
    }

    /* --- 二级警报：右前红外接触(<1cm) 或 烟雾/酒精超阈值 ---
     * 气体报警默认关闭（GAS_ALARM_ENABLE=0），避免 ADC 漂移/上电尖峰误触三档；
     * 需启用时置 1，且仅在 gas_ok（预热完成）+ gas_over（连续帧确认）后生效。 */
#if (GAS_ALARM_ENABLE == 1u)
    if (s->ir_right_contact || (s->gas_ok && s->gas_over)) {
        return ALERT_LEVEL3;
    }
#else
    if (s->ir_right_contact) {
        return ALERT_LEVEL3;
    }
#endif
    /* --- 接触兜底：前超声波 ≤8cm（贴障）强制三档，即使红外未触发 ---
     * 解决"物体贴着红外只显示二档"：红外电平与硬件极性不符时，
     * 由前超声波近距离兜底保证贴障一定三档。 */
    if (s->dist_front_cm <= TH_CONTACT_CM) {
        return ALERT_LEVEL3;
    }
    /* --- 2.5级：左前红外触发(<10cm) → 自动转向/死胡同掉头 --- */
    if (s->ir_left_contact) {
        return ALERT_LEVEL25;
    }
    /* --- 一级警报：前超声波 <30cm → 对比左右转向 --- */
    if (s->dist_front_cm <= TH_ALARM_CM) {
        return ALERT_LEVEL2;
    }
    /* --- 一级预警：前超声波 <1m → 减速 --- */
    if (s->dist_front_cm <= TH_WARN_CM) {
        return ALERT_LEVEL1;
    }
    return ALERT_NONE;
}

/* ============================ 运动规划 ============================ */
/**
 * @brief  通用"对比左右超声波转向"（一级警报与 2.5 级共用）
 */
static void PlanTurnBySide(const SensorData_t *s, Decision_t *out)
{
    uint16_t dl = s->dist_left_cm, dr = s->dist_right_cm;
    uint8_t open_l = (dl > TH_ALARM_CM);
    uint8_t open_r = (dr > TH_ALARM_CM);

    if (open_r && !open_l)      { out->target_speed_l =  SPEED_TURN; out->target_speed_r = -SPEED_TURN; }
    else if (open_l && !open_r) { out->target_speed_l = -SPEED_TURN; out->target_speed_r =  SPEED_TURN; }
    else if (open_l && open_r)  {
        if (dr >= dl) { out->target_speed_l = SPEED_TURN; out->target_speed_r = SPEED_SLOW; }
        else          { out->target_speed_l = SPEED_SLOW; out->target_speed_r = SPEED_TURN; }
    } else                      { out->target_speed_l = -SPEED_SLOW; out->target_speed_r = -SPEED_SLOW; }
}

/**
 * @brief  2.5 级自动转向：依托左右+后超声波选向；左/右/后全堵=死胡同→掉头
 */
static void PlanLevel25(const SensorData_t *s, Decision_t *out)
{
    uint16_t dl = s->dist_left_cm, dr = s->dist_right_cm, db = s->dist_back_cm;
    uint8_t open_l = (dl > TH_ALARM_CM);
    uint8_t open_r = (dr > TH_ALARM_CM);

    if (!open_l && !open_r && (db <= TH_DEADEND_CM)) {
        /* 死胡同：原地 180° 掉头（差速旋转，靠时间完成半圈；
         * 简单可靠做法：原地旋转 1.2s，由决策层进入后持续执行） */
        out->target_speed_l =  SPEED_UTURN;
        out->target_speed_r = -SPEED_UTURN;
        return;
    }
    PlanTurnBySide(s, out);
}

/**
 * @brief  直线段航向修正：偏航 >3° 时差速补偿（直行纠偏，MPU6050）
 */
static void ApplyYawCorrection(int16_t base, Decision_t *out)
{
    if (!g_sensor.mpu_ok) return;
    int16_t yaw = g_sensor.yaw_deg10;
    if (yaw > YAW_CORRECT_TH_DEG10 || yaw < -YAW_CORRECT_TH_DEG10) {
        int16_t comp = (int16_t)((yaw / 10) * YAW_CORRECT_GAIN);
        if (comp >  YAW_CORRECT_MAX) comp =  YAW_CORRECT_MAX;
        if (comp < -YAW_CORRECT_MAX) comp = -YAW_CORRECT_MAX;
        /* yaw>0=右偏 → 左轮减、右轮加，把车头拉回 */
        out->target_speed_l = (int16_t)(base - comp);
        out->target_speed_r = (int16_t)(base + comp);
    } else {
        out->target_speed_l = base;
        out->target_speed_r = base;
    }
}

/**
 * @brief  模式1：普通避障运动规划
 */
static void PlanModeNormal(const SensorData_t *s, Decision_t *out)
{
    uint16_t df = s->dist_front_cm;

    /* 一级警报：<30cm 对比左右转向 */
    if (df <= TH_ALARM_CM) { PlanTurnBySide(s, out); return; }

    /* 一级预警：<1m 减速直行（带航向修正） */
    if (df <= TH_WARN_CM)  { ApplyYawCorrection(SPEED_SLOW, out); return; }

    /* 侧向过近轻微修偏 */
    if ((s->dist_left_cm <= TH_ALARM_CM) && (s->dist_right_cm > s->dist_left_cm)) {
        out->target_speed_l = SPEED_CRUISE + 10; out->target_speed_r = SPEED_CRUISE - 10; return;
    }
    if ((s->dist_right_cm <= TH_ALARM_CM) && (s->dist_left_cm > s->dist_right_cm)) {
        out->target_speed_l = SPEED_CRUISE - 10; out->target_speed_r = SPEED_CRUISE + 10; return;
    }

    /* 巡航直行（带航向修正） */
    ApplyYawCorrection(SPEED_CRUISE, out);
}

/**
 * @brief  模式2：视觉+毫米波融合优先，无效回退模式1
 */
static void PlanModeFusion(const SensorData_t *s, Decision_t *out)
{
    if (s->fusion_valid) {
        if (s->fusion_dist_cm <= TH_ALARM_CM)      { PlanTurnBySide(s, out); return; }
        if (s->fusion_dist_cm <= TH_WARN_CM + 200u) { ApplyYawCorrection(SPEED_SLOW, out); return; } /* (0.3m,3m] 减速 */
    }
    PlanModeNormal(s, out);
}

/* ============================ 任务1：传感采集 ============================ */
void TaskSensor_Start(void *argument)
{
    (void)argument;
    static uint8_t trig_idx = HCSR04_FRONT;
    static uint8_t read_idx = HCSR04_FRONT;
    uint32_t last_dht_tick = 0;
    uint32_t last_tick = osKernelGetTickCount();

    for (;;) {
        /* 1. 读上一帧触发的那只超声波 */
        uint16_t dist = HCSR04_GetDistanceCm((HCSR04_Index_t)read_idx);
        switch (read_idx) {
            case HCSR04_FRONT: g_sensor.dist_front_cm = dist; break;
            case HCSR04_LEFT:  g_sensor.dist_left_cm  = dist; break;
            case HCSR04_RIGHT: g_sensor.dist_right_cm = dist; break;
            case HCSR04_BACK:  g_sensor.dist_back_cm  = dist; break;
            default: break;
        }

        /* 2. 触发下一只 */
        HCSR04_Trigger((HCSR04_Index_t)trig_idx);
        read_idx = trig_idx;
        trig_idx = (uint8_t)((trig_idx + 1u) % HCSR04_NUM);

        /* 超声波卡死自恢复：所有通道连续多次读到 INVALID（无回波）
         * 视为 TIM5 输入捕获中断丢失，重启 4 路 IC + 中断。 */
        static uint8_t invalid_streak = 0u;
        if ((g_sensor.dist_front_cm == HCSR04_INVALID_CM) &&
            (g_sensor.dist_left_cm  == HCSR04_INVALID_CM) &&
            (g_sensor.dist_right_cm == HCSR04_INVALID_CM) &&
            (g_sensor.dist_back_cm  == HCSR04_INVALID_CM)) {
            if (++invalid_streak >= 8u) {  /* 8 帧 ×20ms ×4 通道 = ~640ms 全无回波 */
                invalid_streak = 0u;
                /* 重启 TIM5 全部 4 路输入捕获中断（地址 = tim.h 中已定义 s_echo_ch） */
                extern TIM_HandleTypeDef htim5;
                static const uint32_t chs[4] = {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4};
                for (uint8_t i = 0u; i < 4u; i++) {
                    HAL_TIM_IC_Stop_IT(&htim5, chs[i]);
                    HAL_TIM_IC_Start_IT(&htim5, chs[i]);
                }
            }
        } else {
            invalid_streak = 0u;
        }

        /* 3. 红外（左=2.5级探测器 右=3级探测器）
         * 原始电平存 ir_left/ir_right 供 OLED 显示；去抖后接触标志
         * ir_left_contact/ir_right_contact 供决策分级。
         * 用 Infrared_Detected 按 IR_ACTIVE_LEVEL 归一化极性，适配不同模块。 */
        uint8_t det_l = Infrared_Detected(IR_LEFT);
        uint8_t det_r = Infrared_Detected(IR_RIGHT);
        g_sensor.ir_left  = det_l;
        g_sensor.ir_right = det_r;
        /* 连续 N 帧检出才置接触标志，清除时立即复位 */
        static uint8_t irl_cnt = 0, irr_cnt = 0;
        irl_cnt = det_l ? (uint8_t)(irl_cnt + 1u) : 0u;
        irr_cnt = det_r ? (uint8_t)(irr_cnt + 1u) : 0u;
        g_sensor.ir_left_contact  = (irl_cnt >= IR_DEBOUNCE_FRAMES) ? 1u : 0u;
        g_sensor.ir_right_contact = (irr_cnt >= IR_DEBOUNCE_FRAMES) ? 1u : 0u;

        /* 4. 气体：MQ 上电需预热（传感器热板 20s+ 才稳定），预热完成前
         *    不认可数据、不参与报警。阈值已提高至 3000，并用连续帧去抖，
         *    避免上电/移除模块后 ADC 漂移误触发三档。 */
        g_sensor.mq2_raw = GasSensor_GetMQ2();
        g_sensor.mq3_raw = GasSensor_GetMQ3();
        g_sensor.mq2_ok  = (g_sensor.mq2_raw < GAS_MQ2_THRESHOLD) ? 1u : 0u;
        g_sensor.mq3_ok  = (g_sensor.mq3_raw < GAS_MQ3_THRESHOLD) ? 1u : 0u;
        if (!g_sensor.gas_ok) {
            /* 预热约 5s（250 帧）后才认可气体数据（比 1s 更能避开上电尖峰） */
            static uint32_t gas_warmup = 0u;
            if (gas_warmup++ >= 250u) {
                g_sensor.gas_ok = 1u;
            }
        }
        /* 超标去抖：连续 GAS_CONFIRM_FRAMES 帧任一路超阈值才置 gas_over */
        if (g_sensor.gas_ok) {
            static uint8_t gas_cnt = 0u;
            uint8_t over = ((g_sensor.mq2_ok == 0u) || (g_sensor.mq3_ok == 0u)) ? 1u : 0u;
            gas_cnt = over ? (uint8_t)(gas_cnt + 1u) : 0u;
            g_sensor.gas_over = (gas_cnt >= GAS_CONFIRM_FRAMES) ? 1u : 0u;
        } else {
            g_sensor.gas_over = 0u;
        }

        /* 5. MPU6050 航向积分（20ms 一拍）；停车时自动锚定清零 */
        {
            int16_t yaw = 0, gz = 0;
            if (MPU6050_Update(20, &yaw, &gz)) {
                g_sensor.mpu_ok = 1u;
                g_sensor.yaw_deg10 = yaw;
                g_sensor.gz_dps10  = gz;
                /* 车静止（速度指令≈0）时视为重新出发，锚定航向零点 */
                if ((g_decision.target_speed_l == 0) && (g_decision.target_speed_r == 0)) {
                    MPU6050_ResetYaw();
                    g_sensor.yaw_deg10 = 0;
                }
            } else {
                g_sensor.mpu_ok = 0u;
            }
        }

        /* 6. DHT11 限频 1s */
        if ((osKernelGetTickCount() - last_dht_tick) >= 1000u) {
            last_dht_tick = osKernelGetTickCount();
            uint8_t t = 0, h = 0;
            if (DHT11_Read(&t, &h)) {
                g_sensor.temp_c = (int8_t)t; g_sensor.humi_pct = h; g_sensor.dht_ok = 1u;
            } else {
                g_sensor.dht_ok = 0u;
            }
        }

        g_sensor.update_tick = osKernelGetTickCount();
        last_tick = g_sensor.update_tick;
        osDelay(20);
    }
}

/* ============================ 任务2：决策状态机 ============================ */
void TaskDecision_Start(void *argument)
{
    (void)argument;
    AppCmd_t cmd;
    SensorData_t snap;
    uint8_t warm_cycles = 0;

    for (;;) {
        /* ---------- 0. 物理按键模式切换（PD0/PD1/PD2，低电平触发，内部上拉） ---------- */
        static uint8_t key_lock = 0u;  /* 简易防抖：按下置位，松开清零，避免长按连切 */
        uint8_t k1 = (HAL_GPIO_ReadPin(KEY_MODE1_GPIO_Port, KEY_MODE1_Pin) == GPIO_PIN_RESET) ? 1u : 0u;
        uint8_t k2 = (HAL_GPIO_ReadPin(KEY_MODE2_GPIO_Port, KEY_MODE2_Pin) == GPIO_PIN_RESET) ? 1u : 0u;
        uint8_t k3 = (HAL_GPIO_ReadPin(KEY_MODE3_GPIO_Port, KEY_MODE3_Pin) == GPIO_PIN_RESET) ? 1u : 0u;
        if ((k1 || k2 || k3) && !key_lock) {
            key_lock = 1u;
            if      (k1) { g_decision.mode = MODE_NORMAL;     MPU6050_ResetYaw(); }
            else if (k2) { g_decision.mode = MODE_FUSION;     MPU6050_ResetYaw(); }
            else if (k3) { g_decision.mode = MODE_BLUETOOTH;  MPU6050_ResetYaw(); }
        } else if (!(k1 || k2 || k3)) {
            key_lock = 0u;  /* 全松开后清零，允许下次按键触发 */
        }

        /* ---------- 1. 蓝牙指令 ---------- */
        while (osMessageQueueGet(q_bt_cmd, &cmd, NULL, 0) == osOK) {
            switch (cmd.cmd) {
                case BT_CMD_MODE1: g_decision.mode = MODE_NORMAL;   MPU6050_ResetYaw(); break;
                case BT_CMD_MODE2: g_decision.mode = MODE_FUSION;   MPU6050_ResetYaw(); break;
                case BT_CMD_MODE3: g_decision.mode = MODE_BLUETOOTH; MPU6050_ResetYaw(); break;
                case BT_CMD_FWD:   g_decision.rc_speed_l =  SPEED_CRUISE; g_decision.rc_speed_r =  SPEED_CRUISE; g_decision.rc_active = 1; break;
                case BT_CMD_BACK:  g_decision.rc_speed_l = -SPEED_SLOW;  g_decision.rc_speed_r = -SPEED_SLOW;  g_decision.rc_active = 1; break;
                case BT_CMD_TURNL: g_decision.rc_speed_l = -SPEED_TURN; g_decision.rc_speed_r =  SPEED_TURN; g_decision.rc_active = 1; break;
                case BT_CMD_TURNR: g_decision.rc_speed_l =  SPEED_TURN; g_decision.rc_speed_r = -SPEED_TURN; g_decision.rc_active = 1; break;
                case BT_CMD_UTURN: g_decision.rc_speed_l =  SPEED_UTURN; g_decision.rc_speed_r = -SPEED_UTURN; g_decision.rc_active = 1; break;
                case BT_CMD_STOP:  g_decision.rc_speed_l = 0; g_decision.rc_speed_r = 0; g_decision.rc_active = 1; break;
                case BT_CMD_RESET:
                    /* 软件复位：全系统重启（恢复上电默认模式/传感器预热） */
                    NVIC_SystemReset();
                    break;
                default: break;   /* TH 查询在 TaskBt 内直接回传，不入队列 */
            }
        }

        /* ---------- 2. K230 指令（待串口驱动） ---------- */
        while (osMessageQueueGet(q_k230_cmd, &cmd, NULL, 0) == osOK) { }

        /* ---------- 3. 快照 + 分级（互斥：只保留最高级；带降级迟滞） ----------
         * 升级（检测到障碍）立即生效；降级（移开障碍）需连续 N 帧保持更低
         * 等级才确认，避免抖动闪灯/蜂鸣。"上快下慢"的非对称响应。 */
        static uint8_t lower_count = 0u;
        static uint8_t last_applied_level = ALERT_NONE;
        snap = g_sensor;
        uint8_t new_level = CalcAlertLevel(&snap);

        if (new_level >= last_applied_level) {
            /* 升级或同级：立即生效 */
            last_applied_level = new_level;
            lower_count = 0u;
        } else {
            /* 降级：累计连续低帧，达标才允许降 */
            lower_count = (uint8_t)(lower_count + 1u);
            if (lower_count >= ALERT_HOLD_FRAMES) {
                last_applied_level = new_level;
                lower_count = 0u;
            }
            /* 否则保持上一等级，实现"移开约100ms后才停蜂鸣"的舒适体验 */
        }
        g_decision.alert_level = last_applied_level;

        /* 模式2 融合叠加（只升不降） */
        if ((g_decision.mode == MODE_FUSION) && (snap.fusion_valid)) {
            if ((snap.fusion_dist_cm <= TH_ALARM_CM) && (g_decision.alert_level < ALERT_LEVEL2))
                g_decision.alert_level = ALERT_LEVEL2;
            else if ((snap.fusion_dist_cm <= 300u) && (g_decision.alert_level < ALERT_LEVEL1))
                g_decision.alert_level = ALERT_LEVEL1;
        }

        /* now 供步骤7预热判定使用；C/R 接管窗口已移除 */
        uint32_t now = osKernelGetTickCount();

        /* ---------- 4. 按模式规划 ---------- */
        switch (g_decision.mode) {
        case MODE_IDLE:    /* 停止模式：电机不动，仅预警 */
            g_decision.target_speed_l = 0;
            g_decision.target_speed_r = 0;
            g_decision.rc_active = 0;
            break;
        case MODE_NORMAL:  PlanModeNormal(&snap, &g_decision); break;
        case MODE_FUSION:  PlanModeFusion(&snap, &g_decision); break;
        case MODE_BLUETOOTH:
            if (g_decision.rc_active) {
                g_decision.target_speed_l = g_decision.rc_speed_l;
                g_decision.target_speed_r = g_decision.rc_speed_r;
            }
            break;
        default: g_decision.mode = MODE_NORMAL; break;
        }

        /* ---------- 5. 2.5 级：自动转向/死胡同掉头（覆盖当前规划） ---------- */
        if (g_decision.alert_level == ALERT_LEVEL25) {
            PlanLevel25(&snap, &g_decision);
        }

        /* ---------- 6. 二级警报：任何模式强制清零 ---------- */
        if (g_decision.alert_level == ALERT_LEVEL3) {
            g_decision.target_speed_l = 0;
            g_decision.target_speed_r = 0;
        }

        /* ---------- 7. 预热保护 ---------- */
        uint8_t fresh = (g_sensor.update_tick != 0u) && ((now - g_sensor.update_tick) < 100u);
        warm_cycles = fresh ? (uint8_t)(warm_cycles + 1u) : 0u;
        g_decision.motor_enabled = (warm_cycles >= 4u) ? 1u : 0u;

        osDelay(20);
    }
}

/* ============================ 任务3：电机控制 ============================ */
void TaskMotor_Start(void *argument)
{
    (void)argument;
    for (;;) {
        if (g_decision.motor_enabled) {
            TB6612_Motor_SetSpeedPercent(g_decision.target_speed_l,
                                         g_decision.target_speed_r);
        } else {
            TB6612_Motor_Stop();
        }
        osDelay(10);
    }
}

/* ============================ 任务4：蓝牙通信（协议解析） ============================ */
/**
 * 手机串口助手发送内容与功能对照（JDY-31，9600 N81，发送不带回车也可）：
 *   MA → 切换模式1（普通避障）   MB → 切换模式2（融合避障）
 *   MC → 切换模式3（蓝牙遥控）   TH → 查询温湿度（板端回传 T:xxC H:xx%）
 *   W  → 前进   S → 后退   A → 左转   D → 右转   U → 掉头   X → 停止
 *   RST→ 软件复位（NVIC_SystemReset）
 *   每条指令解析后立即向手机回传 ASCII 状态反馈，便于确认指令已收到。
 */
void TaskBt_Start(void *argument)
{
    (void)argument;
    static char line[16];
    uint8_t len = 0;
    uint32_t last_byte_tick = osKernelGetTickCount();

    HAL_UART_Receive_IT(&huart1, &s_bt_rx_byte, 1);   /* 启动中断接收 */

    for (;;) {
        int c = Bt_RingGet();
        if (c >= 0) {
            last_byte_tick = osKernelGetTickCount();
            if ((char)c != '\r' && (char)c != '\n') {
                /* 跳过空格/Tab 等无意义空白字符（手机按钮控制模式会在指令后附 "\n    "） */
                if ((char)c == ' ' || (char)c == '\t') {
                    /* 忽略，不入缓冲 */
                } else if (len < (uint8_t)(sizeof(line) - 1u)) {
                    line[len++] = (char)c;
                }
            }
        } else {
            /* 30ms 无新字节视为一帧结束 */
            if ((len > 0u) && ((osKernelGetTickCount() - last_byte_tick) >= 30u)) {
                /* 兜底：剔除末尾可能残留的空白，保证 strcmp 严格匹配 */
                while ((len > 0u) && (line[len - 1u] == ' ' || line[len - 1u] == '\t')) {
                    len--;
                }
                line[len] = '\0';
                AppCmd_t cmd = {0};
                uint8_t hit = 1u;
                if      (strcmp(line, "MA") == 0) { cmd.cmd = BT_CMD_MODE1;   { const char *_s="[OK] Mode1\r\n"; HAL_UART_Transmit(&huart1,(uint8_t*)_s,strlen(_s),50); } }
                else if (strcmp(line, "MB") == 0) { cmd.cmd = BT_CMD_MODE2;   { const char *_s="[OK] Mode2\r\n"; HAL_UART_Transmit(&huart1,(uint8_t*)_s,strlen(_s),50); } }
                else if (strcmp(line, "MC") == 0) { cmd.cmd = BT_CMD_MODE3;   { const char *_s="[OK] Mode3\r\n"; HAL_UART_Transmit(&huart1,(uint8_t*)_s,strlen(_s),50); } }
                else if (strcmp(line, "W")  == 0) { cmd.cmd = BT_CMD_FWD;     { const char *_s="[OK] FWD\r\n";  HAL_UART_Transmit(&huart1,(uint8_t*)_s,strlen(_s),50); } }
                else if (strcmp(line, "S")  == 0) { cmd.cmd = BT_CMD_BACK;    { const char *_s="[OK] BACK\r\n"; HAL_UART_Transmit(&huart1,(uint8_t*)_s,strlen(_s),50); } }
                else if (strcmp(line, "A")  == 0) { cmd.cmd = BT_CMD_TURNL;   { const char *_s="[OK] LEFT\r\n"; HAL_UART_Transmit(&huart1,(uint8_t*)_s,strlen(_s),50); } }
                else if (strcmp(line, "D")  == 0) { cmd.cmd = BT_CMD_TURNR;   { const char *_s="[OK] RIGHT\r\n";HAL_UART_Transmit(&huart1,(uint8_t*)_s,strlen(_s),50); } }
                else if (strcmp(line, "U")  == 0) { cmd.cmd = BT_CMD_UTURN;   { const char *_s="[OK] UTURN\r\n";HAL_UART_Transmit(&huart1,(uint8_t*)_s,strlen(_s),50); } }
                else if (strcmp(line, "X")  == 0) { cmd.cmd = BT_CMD_STOP;    { const char *_s="[OK] STOP\r\n"; HAL_UART_Transmit(&huart1,(uint8_t*)_s,strlen(_s),50); } }
                else if (strcmp(line, "RST")== 0) { cmd.cmd = BT_CMD_RESET;   { const char *_s="[OK] RST\r\n";  HAL_UART_Transmit(&huart1,(uint8_t*)_s,strlen(_s),50); } }
                else if (strcmp(line, "TH") == 0) {
                    /* 温湿度查询：直接回传，不入决策队列 */
                    char rep[24];
                    int n = snprintf(rep, sizeof(rep), "[TH] T:%dC H:%u%%\r\n",
                                     g_sensor.temp_c, g_sensor.humi_pct);
                    HAL_UART_Transmit(&huart1, (uint8_t *)rep, (uint16_t)n, 50);
                }
                else hit = 0u;
                if (hit && (cmd.cmd != 0u)) {
                    osMessageQueuePut(q_bt_cmd, &cmd, 0, 0);
                }
                len = 0;
            }
            osDelay(5);
        }
    }
}

/* ============================ 任务5：K230 通信 ============================ */
void TaskK230_Start(void *argument)
{
    (void)argument;
    for (;;) {
        /* TODO(串口驱动): IDLE+DMA 收帧后 osMessageQueuePut(q_k230_cmd,...) */
        osDelay(10);
    }
}

/* ============================ 任务6：显示与声光预警（互斥显示） ============================ */
static void OLED_ShowDistCm(int16_t x, int16_t y, uint16_t dist_cm)
{
    if (dist_cm > 999u) OLED_ShowString(x, y, "---", OLED_8X16);
    else                OLED_ShowNum(x, y, dist_cm, 3, OLED_8X16);
}

void TaskDisplay_Start(void *argument)
{
    (void)argument;

    /* OLED 已在 freertos.c（MX_FREERTOS_Init）中初始化完毕，
     * 此处不再重复初始化（重复初始化会导致 I2C 状态错乱、显示失败）。 */

    for (;;) {
        uint8_t lv = g_decision.alert_level;

        /* --- 清屏：彻底解决"下半屏残影/上一帧阴影"问题 ---
         * 之前每行只清部分区域（F:/R: 标签 + 3 位距离），8x16 行右侧 x=104-128
         * 的 24px 永远不会被写入，上一帧内容残留；现在直接全屏清，再统一重绘。 */
        OLED_Clear();

        /* --- LED 互斥：只点亮当前最高级对应灯（灌电流：低电平点亮） --- */
        HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, (lv == ALERT_LEVEL1) ? GPIO_PIN_RESET : GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED_Y_GPIO_Port, LED_Y_Pin,
            ((lv == ALERT_LEVEL2) || (lv == ALERT_LEVEL25)) ? GPIO_PIN_RESET : GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, (lv == ALERT_LEVEL3) ? GPIO_PIN_RESET : GPIO_PIN_SET);

        /* --- 蜂鸣器分档（缩短周期到 200ms 一轮，遮挡响应延迟 ≤200ms，
         *     兼顾柔和间歇与即时反馈，避免"等1秒才响"的体验问题） --- */
        static uint8_t tick_cnt = 0;
        tick_cnt = (uint8_t)((tick_cnt + 1u) & 0x03u);  /* 4 拍一轮：50/100/150/200ms */
        switch (lv) {
            case ALERT_LEVEL1:   /* 一级预警：2kHz，100ms 响 / 100ms 停（轻柔间歇） */
                if (tick_cnt < 2u) Beep_SetFreq(BEEP_FREQ_LEVEL1_HZ); else Beep_Off();
                break;
            case ALERT_LEVEL2:   /* 一级警报：3kHz，150ms 响 / 50ms 停（急促提示） */
                if (tick_cnt < 3u) Beep_SetFreq(BEEP_FREQ_LEVEL2_HZ); else Beep_Off();
                break;
            case ALERT_LEVEL25:  /* 2.5 级：间歇"滴滴"短促双音，明确"自动避障转向中"提示 */
                /* 200ms 周期内：50ms 高频+50ms 低频各响一次（共100ms），余100ms停 */
                if      (tick_cnt == 0u) Beep_SetFreq(BEEP_FREQ_LEVEL25_HZ); /* 2.5kHz 滴 */
                else if (tick_cnt == 1u) Beep_SetFreq(BEEP_FREQ_LEVEL1_HZ);  /* 2kHz   滴 */
                else                     Beep_Off();
                break;
            case ALERT_LEVEL3:   /* 二级警报：4kHz 长鸣（持续高音警示） */
                Beep_SetFreq(BEEP_FREQ_LEVEL3_HZ);
                break;
            default:
                Beep_Off();
                break;
        }

        /* --- OLED 布局（64px 高，混排字体，5 行） ---
         * y0  (6x8)  模式+当前等级+温湿度：M:1 L:2.5 T:26C H:55
         * y8  (8x16) 前/左距离
         * y24 (8x16) 右/后距离
         * y40 (6x8)  气体状态+航向角
         * y48 (6x8)  最下排：ALARM 触发串（1/2/2.5/3 并排显示） */
        char buf[28];
        const char *lv_str = (lv == ALERT_NONE)   ? "L:0  " :
                             (lv == ALERT_LEVEL1) ? "L:1  " :
                             (lv == ALERT_LEVEL2) ? "L:2  " :
                             (lv == ALERT_LEVEL25)? "L:2.5" : "L:3  ";
        snprintf(buf, sizeof(buf), "M:%u %s T:%dC H:%u",
                 g_decision.mode, lv_str, (int)g_sensor.temp_c, g_sensor.humi_pct);
        OLED_ShowString(0, 0, "                     ", OLED_6X8);
        OLED_ShowString(0, 0, buf, OLED_6X8);

        OLED_ShowString(0, 8,  "F:", OLED_8X16);  OLED_ShowDistCm(16, 8,  g_sensor.dist_front_cm);
        OLED_ShowString(64, 8, "L:", OLED_8X16);  OLED_ShowDistCm(80, 8,  g_sensor.dist_left_cm);
        OLED_ShowString(0, 24, "R:", OLED_8X16);  OLED_ShowDistCm(16, 24, g_sensor.dist_right_cm);
        OLED_ShowString(64, 24,"B:", OLED_8X16);  OLED_ShowDistCm(80, 24, g_sensor.dist_back_cm);

        snprintf(buf, sizeof(buf), "Q2:%s Q3:%s Y:%d.%d",
                 g_sensor.mq2_ok ? "OK" : "ER",
                 g_sensor.mq3_ok ? "OK" : "ER",
                 (int)(g_sensor.yaw_deg10 / 10),
                 (int)((g_sensor.yaw_deg10 < 0 ? -g_sensor.yaw_deg10 : g_sensor.yaw_deg10) % 10));
        OLED_ShowString(0, 40, "                     ", OLED_6X8);
        OLED_ShowString(0, 40, buf, OLED_6X8);

        /* 最下排：触发的一、二级警报并排打印（用户要求） */
        {
            uint16_t df = g_sensor.dist_front_cm;
            char l3[20];
            l3[0] = '\0';
            if (df <= TH_WARN_CM)   strcat(l3, "1 ");
            if (df <= TH_ALARM_CM)  strcat(l3, "2 ");
            if (g_sensor.ir_left_contact)  strcat(l3, "2.5 ");
            if (lv == ALERT_LEVEL3) strcat(l3, "3 ");
            snprintf(buf, sizeof(buf), "ALARM:%-13s", (l3[0] ? l3 : "--"));
            OLED_ShowString(0, 48, "                     ", OLED_6X8);
            OLED_ShowString(0, 48, buf, OLED_6X8);
        }

        OLED_Update();
        osDelay(50);
    }
}
