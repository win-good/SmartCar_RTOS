/**
 ******************************************************************************
 * @file    app_tasks.c
 * @brief   毕设智能小车 FreeRTOS 任务框架 —— 六个任务的实现
 *
 * 【2026-08-23 重构版】预警四级（高级覆盖低级，声光显示互斥只显最高级）：
 *   一级预警 ：前超声波 <1m      → 减速；绿灯；2kHz 100ms 间歇
 *   一级警报 ：前超声波 <30cm    → 对比左右超声波转向；黄灯；3kHz 50ms 间歇
 *   2.5级    ：左前红外触发(<10cm) → 黄灯 + 2.5k 滴答变调 + 自动转向；
 *              左右后全堵=死胡同 → 掉头
 *   二级警报 ：右前红外触发(<1cm) 或 烟雾/酒精超阈值 → 红灯 + 4kHz 长鸣 +
 *              任何模式速度强制清零
 * 职责划分：超声波负责 1、2 级；左红外=2.5 级探测器(电位器≈10cm 保持现值)、
 *          右红外=3 级探测器(电位器调至最近≈1cm)。
 *
 * 蓝牙协议（JDY-31，USART1 9600，手机串口助手发送字符串；2026-08-24 增强）：
 *   "MA"=切模式1 → 回 "[OK] Mode1 Normal"   "MB"=切模式2 → 回 "[OK] Mode2 Fusion"
 *   "MC"=切模式3 → 回 "[OK] Mode3 Remote"   "TH"=查询温湿度 → 回 "T:26C H:55%"
 *   "W"=前进→回"[OK] FWD"  "S"=后退→回"[OK] BACK"  "A"=左转→回"[OK] LEFT"
 *   "D"=右转→回"[OK] RIGHT" "U"=掉头→回"[OK] UTURN" "X"=停止→回"[OK] STOP"
 *   未识别指令 → 回 "[ERR] Unknown"。
 *   每条指令收到即回传状态文本；W/S/A/D/U 在非模式3 下发会自动切到模式3 再执行，
 *   "X" 停止在任意模式下都立即生效——修复旧版"前进后退无反馈、停止按两次"的 BUG。
 *
 * 模式按键（2026-08-24 新增，PD0/PD1/PD2，内部上拉低有效）：
 *   按键1=模式1 / 按键2=模式2 / 按键3=模式3，与蓝牙 MA/MB/MC 等效；
 *   消抖用"按下锁定 + 三键全松解锁"，长按不会连切。
 *
 * 可靠性（2026-08-24 新增）：TaskDisplay 每帧喂独立看门狗(约2.7s)并做 OLED
 *   I2C 自检（总线卡死自动恢复+重初始化），死机/花屏均可自愈。
 *
 * 直行保持：MPU6050 航向积分，直线段偏航 >3° 时差速修正 TB6612。
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
#define TH_DEADEND_CM       20u   /* 死胡同判定：左/右/后均 <20cm → 掉头   */

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
    /* --- 二级警报：右前红外接触(<1cm) 或 烟雾/酒精超阈值 --- */
    if ((s->ir_right == 0u) || (s->mq2_ok == 0u) || (s->mq3_ok == 0u)) {
        return ALERT_LEVEL3;
    }
    /* --- 2.5级：左前红外触发(<10cm) → 自动转向/死胡同掉头 --- */
    if (s->ir_left == 0u) {
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

        /* 3. 红外（左=2.5级探测器 右=3级探测器） */
        g_sensor.ir_left  = Infrared_Read(IR_LEFT);
        g_sensor.ir_right = Infrared_Read(IR_RIGHT);

        /* 4. 气体 */
        g_sensor.mq2_raw = GasSensor_GetMQ2();
        g_sensor.mq3_raw = GasSensor_GetMQ3();
        g_sensor.mq2_ok  = (g_sensor.mq2_raw < GAS_MQ2_THRESHOLD) ? 1u : 0u;
        g_sensor.mq3_ok  = (g_sensor.mq3_raw < GAS_MQ3_THRESHOLD) ? 1u : 0u;

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
        /* ---------- 0. 模式按键（PD0/PD1/PD2，2026-08-24 新增） ----------
         * 与蓝牙 MA/MB/MC 等效：按键1=模式1 / 按键2=模式2 / 按键3=模式3。
         * 消抖策略：首次检测到按下立即切换并置 key_lock，三键全部松开才解锁，
         * 长按不会连续切换；20ms 任务周期天然滤除毫秒级抖动。 */
        {
            static uint8_t key_lock = 0u;
            uint8_t k1 = (HAL_GPIO_ReadPin(KEY_MODE1_GPIO_Port, KEY_MODE1_Pin) == GPIO_PIN_RESET) ? 1u : 0u;
            uint8_t k2 = (HAL_GPIO_ReadPin(KEY_MODE2_GPIO_Port, KEY_MODE2_Pin) == GPIO_PIN_RESET) ? 1u : 0u;
            uint8_t k3 = (HAL_GPIO_ReadPin(KEY_MODE3_GPIO_Port, KEY_MODE3_Pin) == GPIO_PIN_RESET) ? 1u : 0u;
            if ((k1 || k2 || k3) && !key_lock) {
                key_lock = 1u;   /* 锁定，防长按连切 */
                if      (k1) { g_decision.mode = MODE_NORMAL;    MPU6050_ResetYaw(); }
                else if (k2) { g_decision.mode = MODE_FUSION;    MPU6050_ResetYaw(); }
                else if (k3) { g_decision.mode = MODE_BLUETOOTH; MPU6050_ResetYaw(); }
            } else if (!(k1 || k2 || k3)) {
                key_lock = 0u;   /* 全松解锁，允许下次按键触发 */
            }
        }

        /* ---------- 1. 蓝牙指令 ---------- */
        while (osMessageQueueGet(q_bt_cmd, &cmd, NULL, 0) == osOK) {
            /* 2026-08-24：TaskBt 对"非模式3下发的遥控指令"置 arg[0]=1，
             * 此处自动切入模式3 再执行，保证手机端"发W车就走"，与回传文本一致 */
            if ((cmd.arg[0] == 1) && (g_decision.mode != MODE_BLUETOOTH)) {
                g_decision.mode = MODE_BLUETOOTH;
                MPU6050_ResetYaw();
            }
            switch (cmd.cmd) {
                case BT_CMD_MODE1: g_decision.mode = MODE_NORMAL;   MPU6050_ResetYaw(); break;
                case BT_CMD_MODE2: g_decision.mode = MODE_FUSION;   MPU6050_ResetYaw(); break;
                case BT_CMD_MODE3: g_decision.mode = MODE_BLUETOOTH; MPU6050_ResetYaw(); break;
                case BT_CMD_FWD:   g_decision.rc_speed_l =  SPEED_CRUISE; g_decision.rc_speed_r =  SPEED_CRUISE; g_decision.rc_active = 1; break;
                case BT_CMD_BACK:  g_decision.rc_speed_l = -SPEED_SLOW;  g_decision.rc_speed_r = -SPEED_SLOW;  g_decision.rc_active = 1; break;
                case BT_CMD_TURNL: g_decision.rc_speed_l = -SPEED_TURN; g_decision.rc_speed_r =  SPEED_TURN; g_decision.rc_active = 1; break;
                case BT_CMD_TURNR: g_decision.rc_speed_l =  SPEED_TURN; g_decision.rc_speed_r = -SPEED_TURN; g_decision.rc_active = 1; break;
                case BT_CMD_UTURN: g_decision.rc_speed_l =  SPEED_UTURN; g_decision.rc_speed_r = -SPEED_UTURN; g_decision.rc_active = 1; break;
                case BT_CMD_STOP:  g_decision.rc_speed_l = 0; g_decision.rc_speed_r = 0; g_decision.rc_active = 1;
                                   /* 2026-08-24：停止跨模式立即生效（安全优先），
                                    * 不必等下一轮规划，任何模式下收到 X 马上刹车 */
                                   g_decision.target_speed_l = 0; g_decision.target_speed_r = 0; break;
                default: break;   /* TH 查询在 TaskBt 内直接回传，不入队列 */
            }
        }

        /* ---------- 2. K230 指令（待串口驱动） ---------- */
        while (osMessageQueueGet(q_k230_cmd, &cmd, NULL, 0) == osOK) { }

        /* ---------- 3. 快照 + 分级（互斥：只保留最高级） ---------- */
        snap = g_sensor;
        g_decision.alert_level = CalcAlertLevel(&snap);

        /* 模式2 融合叠加（只升不降） */
        if ((g_decision.mode == MODE_FUSION) && (snap.fusion_valid)) {
            if ((snap.fusion_dist_cm <= TH_ALARM_CM) && (g_decision.alert_level < ALERT_LEVEL2))
                g_decision.alert_level = ALERT_LEVEL2;
            else if ((snap.fusion_dist_cm <= 300u) && (g_decision.alert_level < ALERT_LEVEL1))
                g_decision.alert_level = ALERT_LEVEL1;
        }

        /* ---------- 4. 按模式规划 ---------- */
        switch (g_decision.mode) {
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
        uint32_t now = osKernelGetTickCount();
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
 */
/* 蓝牙状态回传辅助：把固定字符串发给手机（9600 下 20 字节约 20ms，80ms 超时足够） */
static void Bt_Send(const char *s)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)s, (uint16_t)strlen(s), 80);
}

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
                if (len < (uint8_t)(sizeof(line) - 1u)) line[len++] = (char)c;
            }
        } else {
            /* 30ms 无新字节视为一帧结束 */
            if ((len > 0u) && ((osKernelGetTickCount() - last_byte_tick) >= 30u)) {
                line[len] = '\0';
                AppCmd_t cmd = {0};
                uint8_t hit = 1u;
                /* 2026-08-24 重构：
                 *  1) 每条合法指令"收到即回传"状态文本，手机端必有反馈；
                 *  2) 遥控指令(W/S/A/D/U)若当前不在模式3，先置自动切模式3
                 *     标志随指令入队，决策任务执行切模式——修复旧版"有反馈车不动、
                 *     停止要按两次"的体验 BUG（根因：遥控速度只作用于模式3）；
                 *  3) "X" 停止任何模式立即生效（决策任务内已做跨模式清零）；
                 *  4) 未识别指令回 [ERR]，便于手机端排查拼写问题。 */
                if      (strcmp(line, "MA") == 0) { cmd.cmd = BT_CMD_MODE1; Bt_Send("[OK] Mode1 Normal\r\n"); }
                else if (strcmp(line, "MB") == 0) { cmd.cmd = BT_CMD_MODE2; Bt_Send("[OK] Mode2 Fusion\r\n"); }
                else if (strcmp(line, "MC") == 0) { cmd.cmd = BT_CMD_MODE3; Bt_Send("[OK] Mode3 Remote\r\n"); }
                else if (strcmp(line, "W")  == 0) { cmd.cmd = BT_CMD_FWD;   Bt_Send("[OK] FWD\r\n"); }
                else if (strcmp(line, "S")  == 0) { cmd.cmd = BT_CMD_BACK;  Bt_Send("[OK] BACK\r\n"); }
                else if (strcmp(line, "A")  == 0) { cmd.cmd = BT_CMD_TURNL; Bt_Send("[OK] LEFT\r\n"); }
                else if (strcmp(line, "D")  == 0) { cmd.cmd = BT_CMD_TURNR; Bt_Send("[OK] RIGHT\r\n"); }
                else if (strcmp(line, "U")  == 0) { cmd.cmd = BT_CMD_UTURN; Bt_Send("[OK] UTURN\r\n"); }
                else if (strcmp(line, "X")  == 0) { cmd.cmd = BT_CMD_STOP;  Bt_Send("[OK] STOP\r\n"); }
                else if (strcmp(line, "TH") == 0) {
                    /* 温湿度查询：直接回传，不入决策队列 */
                    char rep[24];
                    int n = snprintf(rep, sizeof(rep), "T:%dC H:%u%%\r\n",
                                     g_sensor.temp_c, g_sensor.humi_pct);
                    HAL_UART_Transmit(&huart1, (uint8_t *)rep, (uint16_t)n, 80);
                }
                else { hit = 0u; Bt_Send("[ERR] Unknown\r\n"); }
                if (hit && (cmd.cmd != 0u)) {
                    /* 遥控指令且不在模式3：arg[0]=1 通知决策任务自动切模式3 */
                    if ((cmd.cmd >= BT_CMD_FWD) && (cmd.cmd <= BT_CMD_UTURN) &&
                        (g_decision.mode != MODE_BLUETOOTH)) {
                        cmd.arg[0] = 1;
                    }
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
    for (;;) {
        /* 2026-08-24 可靠性双保险（本任务 50ms 周期，是唯一稳定慢节奏任务）：
         *  1) 喂独立看门狗：任何任务死锁/跑飞导致本任务停摆，约2.7s后整机自动复位；
         *  2) OLED I2C 自检：I2C2 卡 BUSY 或 SDA 被拉死时自动恢复总线并重初始化，
         *     花屏/黑屏一帧内自愈，不再累积错位乱码。 */
        App_Watchdog_Feed();
        OLED_I2C_SelfCheck();

        uint8_t lv = g_decision.alert_level;

        /* --- LED 互斥：只点亮当前最高级对应灯（灌电流：低电平点亮） --- */
        HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, (lv == ALERT_LEVEL1) ? GPIO_PIN_RESET : GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED_Y_GPIO_Port, LED_Y_Pin,
            ((lv == ALERT_LEVEL2) || (lv == ALERT_LEVEL25)) ? GPIO_PIN_RESET : GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, (lv == ALERT_LEVEL3) ? GPIO_PIN_RESET : GPIO_PIN_SET);

        /* --- 蜂鸣器分档（频率由低到高）：1=2k间歇 2=3k间歇 2.5=2.5k滴答 3=4k长鸣 --- */
        static uint8_t tick_cnt = 0;
        tick_cnt = (uint8_t)((tick_cnt + 1u) & 0x0Fu);
        switch (lv) {
            case ALERT_LEVEL1:   /* 100ms 响 / 100ms 停（50ms 一拍×2） */
                if (tick_cnt & 0x02u) Beep_SetFreq(BEEP_FREQ_LEVEL1_HZ); else Beep_Off();
                break;
            case ALERT_LEVEL2:   /* 50ms 响 / 50ms 停 */
                if (tick_cnt & 0x01u) Beep_SetFreq(BEEP_FREQ_LEVEL2_HZ); else Beep_Off();
                break;
            case ALERT_LEVEL25:  /* 滴答变调：2.5k 与 2k 交替短音 */
                if (tick_cnt & 0x04u) {
                    Beep_SetFreq((tick_cnt & 0x02u) ? BEEP_FREQ_LEVEL25_HZ : BEEP_FREQ_LEVEL1_HZ);
                } else {
                    Beep_Off();
                }
                break;
            case ALERT_LEVEL3:   /* 长鸣 */
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
            if (df <= TH_WARN_CM)  strcat(l3, "1 ");
            if (df <= TH_ALARM_CM) strcat(l3, "2 ");
            if (g_sensor.ir_left == 0u)  strcat(l3, "2.5 ");
            if (lv == ALERT_LEVEL3)      strcat(l3, "3 ");
            snprintf(buf, sizeof(buf), "ALARM:%-13s", (l3[0] ? l3 : "--"));
            OLED_ShowString(0, 48, "                     ", OLED_6X8);
            OLED_ShowString(0, 48, buf, OLED_6X8);
        }

        OLED_Update();
        osDelay(50);
    }
}
