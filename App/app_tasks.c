/**
 ******************************************************************************
 * @file    app_tasks.c
 * @brief   毕设智能小车 FreeRTOS 任务框架 —— 七个任务的实现
 *
 * 【2026-08-28 四级预警定稿】（高级覆盖低级，声光显示互斥只显最高级）：
 *   一级预警 ：前超声波<1m 或 视觉识别到目标(人体/车辆) 或 雷达目标(1.5m,3m]
 *              → 减速；【绿灯，蜂鸣器静音】
 *   二级预警 ：前超声波<30cm 或 左/右/后超声波<20cm 或 雷达距离≤1.5m
 *              （雷达+视觉共同确认也走此档）→ 转向避让；黄灯；3kHz 50ms 间歇
 *   三级预警 ：任一红外触发(左/右电位器均调至约10cm) → 黄灯(与二级共用)；
 *              2.5kHz 滴答变调(频率/间歇与二级不同)；自动转向/死胡同掉头
 *   四级警报 ：任一超声波<3cm 或 烟雾/酒精超阈值 → 红灯 + 4kHz 长鸣 +
 *              任何模式速度强制清零
 * 职责划分：四路超声波全部联动——前=1/2/4级，左/右/后=2级(<20cm)/4级(<3cm)，
 *          并参与转向选向与死胡同掉头；红外只反馈三级；
 *          雷达/视觉=一级与二级（视觉目标或雷达(1.5m,3m]→一级；雷达≤1.5m→二级）。
 * 距离"无效值"约定：HC-SR04 超量程/无回波返回 0xFFFF，凡参与分级比较必须
 *          先判 d<=DIST_VALID_MAX_CM，避免把无效值当 0cm 误触发四级。
 *
 * 蓝牙协议（JDY-31，USART1 9600，手机串口助手发送字符串；2026-08-24 增强）：
 *   "MA"=切模式1 → 回 "[OK] Mode1 Normal"（切过去自动巡航前进）
 *   "MB"=切模式2 → 回 "[OK] Mode2 Fusion"（切过去自动巡航前进）
 *   "MC"=切模式3 → 回 "[OK] Mode3 Remote"（切过去立即停止静止）
 *   "TH"=查询温湿度 → 回 "T:26C H:55%"     "GAS"=查询气体/酒精 → 回 "MQ2:512 MQ3:480 OK"
 *   "W"=前进→回"[OK] FWD"  "S"=后退→回"[OK] BACK"  "A"=左转→回"[OK] LEFT"
 *   "D"=右转→回"[OK] RIGHT" "U"=掉头→回"[OK] UTURN" "X"=停止→回"[OK] STOP"
 *   未识别指令 → 回 "[ERR] Unknown:xxx"（附带收到的原文便于排查）。
 *   2026-08-28 容错：指令大小写不敏感（w 与 W 等效），帧内/帧尾空白自动忽略。
 *   每条指令收到即回传状态文本；W/S/A/D/U 在非模式3 下发会自动切到模式3 再执行，
 *   "X" 停止在任意模式下都立即生效——修复旧版"前进后退无反馈、停止按两次"的 BUG。
 *
 * 模式按键（2026-08-24 新增，PD0/PD1/PD2，内部上拉低有效）：
 *   按键1=模式1 / 按键2=模式2 / 按键3=模式3，与蓝牙 MA/MB/MC 等效；
 *   消抖用"按下锁定 + 三键全松解锁"，长按不会连切。
 *
 * 可靠性（2026-08-24 新增）：TaskDisplay 每帧喂独立看门狗(20s 超时)并做 OLED
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
#include <stdlib.h>   /* atoi：K230 视觉行解析（2026-08-28） */

/* ============================ 私有定义 ============================ */
#define DIST_VALID_MAX_CM   300u

/* 预警阈值（cm，2026-08-28 四级预警定稿） */
#define TH_WARN_CM          100u  /* 一级：前超声波 <1m → 减速             */
#define TH_ALARM_CM         30u   /* 二级：前超声波 <30cm → 转向避让        */
#define TH_SIDE_CM          20u   /* 二级：左/右/后超声波 <20cm → 转向避让  */
#define TH_CRITICAL_CM       3u   /* 四级：任一超声波(含后) <3cm → 强制制动 */
/* 三级由红外二值触发（硬件电位器定距：左右均约10cm），无软件阈值 */
#define TH_DEADEND_CM       20u   /* 死胡同判定：左/右/后均 <20cm → 掉头   */
/* 三级由红外二值触发（左右电位器均调至约 10cm），无软件阈值——上一行旧注释已删 */

/* 行为调试时间宏（2026-08-29 引出，单位 ms；侧向修偏已改为条件式，无需时间宏） */
#define IR_TURN_HOLD_MS     400u  /* 红外触发反向转向后，即使红外已清除也至少
                                   * 保持转向这么久再恢复直行——避免"一解除就
                                   * 回直"导致贴着障碍物/墙壁走 */
/* 侧向修正改为 0.242 版"条件式即时修偏"（无时间状态机），对应宏已删除 */
#define IR_TURN_MAX_MS      1500u /* 单次红外反向转向的最长时间（硬上限）：即使
                                   * 红外一直触发，到点也强制恢复直行——防止卡角
                                   * 时原地自转四五秒出不来 */

/* 雷达/视觉参数（2026-08-28 启用） */
#define TH_RADAR_LVL1_CM    300u  /* 一级：雷达目标 ∈ (0.3m,3m]              */
#define TH_RADAR_LVL2_CM     30u  /* 二级：雷达距离 ≤0.3m（含雷达+视觉确认） */
#define RADAR_MAX_CM        600u  /* LD2450 量程约 6m，超此值视为无效        */
#define RADAR_TIMEOUT_MS    800u  /* 超过该时长无雷达上报 → 目标视为离开     */
#define VIS_TIMEOUT_MS      800u  /* 视觉目标记忆时长（目标走出画面后渐消）   */
/* 2026-09-05：静止目标过滤阈值(cm/s)。|速度| 小于此值的目标视为静止
 * （墙/桌腿/家具等），不参与预警——解决近处静止物误报二级。
 * 调大=只报明显运动的目标（更抗误报）；调 0=完全不过滤（旧行为）。 */
#define RADAR_MIN_SPEED_CMS  5u

/* 直行航向修正：偏航超过该值(0.1°)开始差速修正（3° = 30） */
#define YAW_CORRECT_TH_DEG10  30
#define YAW_CORRECT_GAIN      5    /* 每 1° 偏航补偿 5% 差速 */
#define YAW_CORRECT_MAX       20   /* 补偿上限 ±20% */

/* 速度档（2026-08-28 统一归入 tb6612_motor.h「电机调试宏区」集中调参）：
 * 实车调速/配平只需改 tb6612_motor.h 里的 MOTOR_*_PCT，重编译即可。 */
#define SPEED_SLOW          MOTOR_SPEED_SLOW_PCT
#define SPEED_CRUISE        MOTOR_SPEED_CRUISE_PCT   /* 三模式直行统一档位（2026-08-29） */
#define SPEED_TURN          MOTOR_SPEED_TURN_PCT
#define SPEED_UTURN         MOTOR_SPEED_UTURN_PCT

/* 2026-09-04：栈溢出诊断（定义在 freertos.c 的 vApplicationStackOverflowHook）：
 * 非空=某任务栈溢出，值为该任务名；显示任务据此打故障横幅。 */
extern volatile char g_stack_overflow_task[16];

/* ============================ 蓝牙接收（中断字节 → 环形缓冲） ============================ */
static uint8_t  s_bt_rx_byte;                 /* 中断接收单字节缓冲 */
static uint8_t  s_bt_ring[64];                /* 环形缓冲 */
static volatile uint16_t s_bt_rd = 0, s_bt_wr = 0;

/* ============================ 雷达/视觉接收（同为 中断字节→环形缓冲，2026-08-28） ============================ */
static uint8_t  s_radar_rx_byte;
static uint8_t  s_radar_ring[256];            /* 【2026-09-04 修复内存越界】原为[160]，但读写
                                               * 索引用 &0xFF 在 256 处回绕 → 索引 160~255
                                               * 越界写坏数组后方的全局变量。LD2450 持续上报，
                                               * 一接雷达就随机踩内存（电机/决策状态被破坏），
                                               * 正是"接雷达后嗡鸣不转、复位时好时坏"根因。
                                               * 环形缓冲尺寸必须=掩码+1 的2的幂，故改 256。 */
static volatile uint16_t s_radar_rd = 0, s_radar_wr = 0;

static uint8_t  s_k230_rx_byte;
static uint8_t  s_k230_ring[96];              /* K230 视觉按行上报，单行很短 */
static volatile uint16_t s_k230_rd = 0, s_k230_wr = 0;

/* UART 接收完成回调（USART1/3/6 中断上下文，优先级 5 满足 FreeRTOS 要求） */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        uint16_t next = (uint16_t)((s_bt_wr + 1u) & 0x3Fu);
        if (next != s_bt_rd) {                 /* 满则丢弃最旧，防溢出崩溃 */
            s_bt_ring[s_bt_wr] = s_bt_rx_byte;
            s_bt_wr = next;
        }
        HAL_UART_Receive_IT(&huart1, &s_bt_rx_byte, 1);   /* 续接下一字节 */
    } else if (huart->Instance == USART6) {    /* LD2450 雷达 256000 */
        uint16_t next = (uint16_t)((s_radar_wr + 1u) & 0xFFu);
        if (next != s_radar_rd) {
            s_radar_ring[s_radar_wr] = s_radar_rx_byte;
            s_radar_wr = next;
        }
        HAL_UART_Receive_IT(&huart6, &s_radar_rx_byte, 1);
    } else if (huart->Instance == USART3) {    /* K230 视觉 115200 */
        uint16_t next = (uint16_t)((s_k230_wr + 1u) & 0x3Fu);
        if (next != s_k230_rd) {
            s_k230_ring[s_k230_wr] = s_k230_rx_byte;
            s_k230_wr = next;
        }
        HAL_UART_Receive_IT(&huart3, &s_k230_rx_byte, 1);
    }
}

static int Bt_RingGet(void)
{
    if (s_bt_rd == s_bt_wr) return -1;
    uint8_t c = s_bt_ring[s_bt_rd];
    s_bt_rd = (uint16_t)((s_bt_rd + 1u) & 0x3Fu);
    return (int)c;
}

static int Radar_RingGet(void)
{
    if (s_radar_rd == s_radar_wr) return -1;
    uint8_t c = s_radar_ring[s_radar_rd];
    s_radar_rd = (uint16_t)((s_radar_rd + 1u) & 0xFFu);
    return (int)c;
}

static int K230_RingGet(void)
{
    if (s_k230_rd == s_k230_wr) return -1;
    uint8_t c = s_k230_ring[s_k230_rd];
    s_k230_rd = (uint16_t)((s_k230_rd + 1u) & 0x3Fu);
    return (int)c;
}

/* ============================ 预警分级 ============================ */
/* 超声波距离有效性：0xFFFF=超量程/无回波，必须先判 <=DIST_VALID_MAX_CM 再比较，
 * 否则无效值会被当成 0cm 误触发四级。返回 1=有效且小于阈值。 */
static uint8_t DistNear(uint16_t d, uint16_t th)
{
    return (uint8_t)((d <= DIST_VALID_MAX_CM) && (d < th));
}

/**
 * @brief  四级预警计算（2026-08-28 定稿，只返回当前最高级，显示层互斥）
 *   四级：任一超声波<3cm 或 气体超标 → 红灯长鸣+强制制动
 *   三级：任一红外触发（左右均约10cm）→ 黄灯滴答+自动转向
 *   二级：前超声波<30cm 或 左/右/后<20cm 或 雷达≤1.5m → 转向避让
 *   一级：前超声波<1m 或 视觉识别到目标 或 雷达目标∈(1.5m,3m] → 减速
 */
static uint8_t CalcAlertLevel(const SensorData_t *s, uint8_t moving)
{
    /* 2026-08-29：静止状态超声波/红外/雷达/视觉一律不参与预警（方便调试）；
     * 气体超标独立于运动状态，静止时仍报四级（安全项）。 */
    if (!moving) {
        return (uint8_t)(((s->mq2_ok == 0u) || (s->mq3_ok == 0u)) ? ALERT_LEVEL4 : ALERT_NONE);
    }

    /* --- 四级警报：任一超声波 <3cm 或 烟雾/酒精超阈值 --- */
    if (DistNear(s->dist_front_cm, TH_CRITICAL_CM) ||
        DistNear(s->dist_left_cm,  TH_CRITICAL_CM) ||
        DistNear(s->dist_right_cm, TH_CRITICAL_CM) ||
        DistNear(s->dist_back_cm,  TH_CRITICAL_CM) ||
        (s->mq2_ok == 0u) || (s->mq3_ok == 0u)) {
        return ALERT_LEVEL4;
    }

    /* --- 三级：任一红外触发（仅红外反馈三级，不参与更高等级判定） --- */
    if ((s->ir_left == 0u) || (s->ir_right == 0u)) {
        return ALERT_LEVEL3;
    }

    /* --- 二级：前<30cm / 侧、后<20cm / 雷达≤1.5m（雷达+视觉共同确认同档） --- */
    if (DistNear(s->dist_front_cm, TH_ALARM_CM) ||
        DistNear(s->dist_left_cm,  TH_SIDE_CM) ||
        DistNear(s->dist_right_cm, TH_SIDE_CM) ||
        DistNear(s->dist_back_cm,  TH_SIDE_CM)) {
        return ALERT_LEVEL2;
    }
    if (s->radar_present && (s->radar_dist_cm <= TH_RADAR_LVL2_CM)) {
        return ALERT_LEVEL2;
    }

    /* --- 一级：前<1m / 视觉目标 / 雷达∈(1.5m,3m] --- */
    if (DistNear(s->dist_front_cm, TH_WARN_CM)) {
        return ALERT_LEVEL1;
    }
    if (s->vis_target_seen) {
        return ALERT_LEVEL1;
    }
    if (s->radar_present && (s->radar_dist_cm <= TH_RADAR_LVL1_CM)) {
        return ALERT_LEVEL1;
    }
    return ALERT_NONE;
}

/* ============================ 运动规划 ============================ */
/**
 * @brief  轻微修偏：以 base 速度直行，同时向更宽一侧差速修正（±10）
 * @note   2026-08-29 回退至上一代行为：前/侧超声波过近时只做小幅修正，
 *         不再原地差速转圈（旧版会一直转到前方脱离阈值，实测转圈时间过长）。
 *         真正的"原地转向"统一交给红外触发（见决策任务三级逻辑）。
 *         2026-08-30 修复：无效值(0xFFFF=无回波)改视为"最近"——旧逻辑把
 *         无效当最远，贴墙一侧读不到回波反被判为"该侧最宽"，车朝墙侧
 *         偏航直接撞墙（实测：右侧贴墙→往右偏）。现主动避开失联侧。
 */
static void PlanVeer(const SensorData_t *s, Decision_t *out, int16_t base)
{
    uint16_t dlc = (s->dist_left_cm  > DIST_VALID_MAX_CM) ? 0u : s->dist_left_cm;
    uint16_t drc = (s->dist_right_cm > DIST_VALID_MAX_CM) ? 0u : s->dist_right_cm;
    if (drc >= dlc) {   /* 右侧更宽 → 左轮快右轮慢，向右偏 */
        out->target_speed_l = (int16_t)(base + 10);
        out->target_speed_r = (int16_t)(base - 10);
    } else {            /* 左侧更宽 → 右轮快左轮慢，向左偏 */
        out->target_speed_l = (int16_t)(base - 10);
        out->target_speed_r = (int16_t)(base + 10);
    }
}

/* ---------- 侧向超声波"条件式即时修偏"（2026-08-30 移植自 0.242 版，实测符合预期） ----------
 * 语义：仅当"某一侧过近(<TH_SIDE_CM)且对侧确实更宽"时差速修偏；
 *       一旦该侧距离恢复到范围外，条件不成立 → 调用方走直行分支——
 *       "出了范围就恢复直线"，无需任何时间状态机。
 * 返回值：1=已写入偏航速度（调用方应 return 防止被直行覆盖）；0=未修偏。
 * 2026-08-30 修复两处：①调用点漏 return 导致偏航被覆盖（修偏从未生效）；
 *               ②无效值 0xFFFF 改视为"最近"，贴墙失联侧主动避开、不再朝墙偏。 */
static uint8_t ApplySideVeer(const SensorData_t *s, Decision_t *out)
{
    /* 无效值(0xFFFF)视为"最近"：贴墙侧读不到回波时主动避开（与 PlanVeer 一致，2026-08-30） */
    uint16_t dl = (s->dist_left_cm  > DIST_VALID_MAX_CM) ? 0u : s->dist_left_cm;
    uint16_t dr = (s->dist_right_cm > DIST_VALID_MAX_CM) ? 0u : s->dist_right_cm;

    /* 左侧过近且右比左宽 → 右偏 */
    if ((dl <= TH_SIDE_CM) && (dr > dl)) {
        out->target_speed_l = (int16_t)(SPEED_CRUISE + 10);
        out->target_speed_r = (int16_t)(SPEED_CRUISE - 10);
        return 1;
    }
    /* 右侧过近且左比右宽 → 左偏 */
    if ((dr <= TH_SIDE_CM) && (dl > dr)) {
        out->target_speed_l = (int16_t)(SPEED_CRUISE - 10);
        out->target_speed_r = (int16_t)(SPEED_CRUISE + 10);
        return 1;
    }
    /* 不满足条件：什么都不做，直行速度由后续分支写入 */
    return 0;
}

/**
 * @brief  直线段航向修正：偏航 >3° 时差速补偿（直行纠偏，MPU6050）
 */
static void ApplyYawCorrection(int16_t base, Decision_t *out)
{
    /* 2026-08-30 修复：MPU 未连接时旧代码直接 return，直行段速度永远=0，
     * 表现为"切模式1不动、要给超声波刺激（进入分支）才走"。
     * 现改为：无 MPU 时不修正，但 base 速度必须写入。 */
    if (!g_sensor.mpu_ok) {
        out->target_speed_l = base;
        out->target_speed_r = base;
        return;
    }
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
 * @brief  模式1：普通避障运动规划（2026-08-28 四路超声波全部联动）
 */
static void PlanModeNormal(const SensorData_t *s, Decision_t *out)
{
    uint16_t df = s->dist_front_cm;

    /* 死胡同掉头：左/右/后均 <20cm → 原地掉头（2026-08-29 从三级迁至此） */
    if (DistNear(s->dist_left_cm, TH_DEADEND_CM) &&
        DistNear(s->dist_right_cm, TH_DEADEND_CM) &&
        DistNear(s->dist_back_cm,  TH_DEADEND_CM)) {
        out->target_speed_l =  SPEED_UTURN; out->target_speed_r = -SPEED_UTURN; return;
    }

    /* 二级：前 <30cm → 向更宽一侧轻微修偏（2026-08-29 回退：不再原地转圈，
     * 原地转向统一由红外触发，见决策任务三级逻辑） */
    if (DistNear(df, TH_ALARM_CM)) { PlanVeer(s, out, SPEED_CRUISE); return; }

    /* 二级联动：左/右侧 <20cm → 条件式即时修偏（2026-08-30 移植 0.242）。
     * 命中后必须 return：否则后续直行分支会把偏航速度覆盖掉（旧 BUG）；
     * 出范围后条件不成立，自动恢复直行分支。 */
    if (ApplySideVeer(s, out)) return;

    /* 一级：前 <1m → 减速直行（带航向修正） */
    if (DistNear(df, TH_WARN_CM)) { ApplyYawCorrection(SPEED_SLOW, out); return; }

    /* 后方联动：后 <20cm → 禁止倒车（巡航直行拉开距离，倒车保护） */
    if (DistNear(s->dist_back_cm, TH_SIDE_CM)) {
        ApplyYawCorrection(SPEED_CRUISE, out); return;
    }

    /* 2026-08-29 修订：三模式直行速度统一为遥控巡航档（低速档低于部分电机启动阈值） */
    ApplyYawCorrection(SPEED_CRUISE, out);
}

/**
 * @brief  模式2：毫米波雷达+视觉融合优先，无效回退模式1（2026-08-28）
 */
static void PlanModeFusion(const SensorData_t *s, Decision_t *out)
{
    if (s->fusion_valid) {
        if (s->fusion_dist_cm <= TH_ALARM_CM)       { PlanVeer(s, out, SPEED_CRUISE); return; }
        if (s->fusion_dist_cm <= TH_RADAR_LVL1_CM)  { ApplyYawCorrection(SPEED_SLOW, out); return; } /* (0.3m,3m] 减速 */
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
        /* 1. 读上一帧触发的那只超声波（Trigger 内部已同步完成测量，此处直接取值） */
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

        /* 3. 红外（三级探测器：左右电位器均调至约10cm，0=有障碍） */
        g_sensor.ir_left  = Infrared_Read(IR_LEFT);
        g_sensor.ir_right = Infrared_Read(IR_RIGHT);

        /* 4. 气体 */
        g_sensor.mq2_raw = GasSensor_GetMQ2();
        g_sensor.mq3_raw = GasSensor_GetMQ3();
        g_sensor.mq2_ok  = (g_sensor.mq2_raw < GAS_MQ2_THRESHOLD) ? 1u : 0u;
        g_sensor.mq3_ok  = (g_sensor.mq3_raw < GAS_MQ3_THRESHOLD) ? 1u : 0u;

        /* 5. MPU6050 航向积分（2026-08-28 改异步初始化 + 失败自愈）：
         *    - 未完成初始化时：每 500ms 重试一次 MPU6050_TaskInit()（内部先做
         *      I2C1 总线自救再配置），成功前 mpu_ok=0，仅禁用航向修正，不影响其他功能；
         *    - 运行中连续失败 25 帧(≈0.5s)：视为总线挂死，先总线自救再重初始化；
         *    - 车静止（速度指令≈0）时锚定航向零点（原有逻辑保留）。 */
        {
            static uint8_t  mpu_inited   = 0u;
            static uint32_t mpu_retry_tk = 0u;
            static uint8_t  mpu_fail_cnt = 0u;
            uint32_t now_tk = osKernelGetTickCount();

            if (!mpu_inited) {
                if ((now_tk - mpu_retry_tk) >= 500u) {
                    mpu_retry_tk = now_tk;
                    if (MPU6050_TaskInit()) {
                        mpu_inited = 1u;
                        g_sensor.mpu_ok = 1u;
                    }
                }
            } else {
                int16_t yaw = 0, gz = 0;
                if (MPU6050_Update(20, &yaw, &gz)) {
                    mpu_fail_cnt = 0u;
                    g_sensor.mpu_ok = 1u;
                    g_sensor.yaw_deg10 = yaw;
                    g_sensor.gz_dps10  = gz;
                    if ((g_decision.target_speed_l == 0) && (g_decision.target_speed_r == 0)) {
                        MPU6050_ResetYaw();
                        g_sensor.yaw_deg10 = 0;
                    }
                } else {
                    g_sensor.mpu_ok = 0u;
                    if (++mpu_fail_cnt >= 25u) {
                        mpu_fail_cnt = 0u;
                        MPU6050_BusRecovery();          /* 总线自救 */
                        mpu_inited = MPU6050_Init();    /* 重配置，失败则回到重试态 */
                    }
                }
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
    uint8_t warm_cycles = 0;     /* 2026-09-04：改回 0.291N 已验证的"传感新鲜度"使能方案 */
    uint8_t prev_moving = 0u;    /* 上一周期运动标志（预警静止屏蔽用） */
    /* --- 红外反向转向状态机（2026-08-29 v2，修复"自转四五秒"缺陷）---
     * v1 缺陷：转向途中左右红外交替触发会中途换向来回摆，转不完；
     * v2 行为：只认触发侧红外、转向中忽略对侧（防换向），
     *          且单次转向有 IR_TURN_MAX_MS 硬上限（卡角也不会一直自转）。
     * ir_st：0=直行  1=右转向(左触发)  2=左转向(右触发) */
    uint8_t  ir_st        = 0u;
    uint32_t ir_turn_tk   = 0u;    /* 本次转向开始的 tick（保持窗口与硬上限共用） */
    uint8_t  ir_both      = 0u;    /* 双触发后退脱困状态（保持到两侧都清空） */
    uint8_t  ir_esc_side  = 0u;    /* 卡角逃逸时的原触发侧（1=左 2=右） */
    /* 模式切换语义宏：自主模式(1/2)=自动巡航前进；模式3(遥控)=立即停止静止。
     * 切换时完整复位转向/修偏状态机与航向基准，杜绝"上一模式残留状态带进新模式"。 */
    #define ENTER_AUTO_MODE(m) do { g_decision.mode = (m); g_decision.rc_active = 0u; \
        g_decision.rc_speed_l = 0; g_decision.rc_speed_r = 0; \
        ir_st = 0u; ir_both = 0u; ir_esc_side = 0u; \
        MPU6050_ResetYaw(); } while (0)
    #define ENTER_REMOTE_MODE() do { g_decision.mode = MODE_BLUETOOTH; g_decision.rc_active = 0u; \
        g_decision.rc_speed_l = 0; g_decision.rc_speed_r = 0; \
        g_decision.target_speed_l = 0; g_decision.target_speed_r = 0; \
        ir_st = 0u; ir_both = 0u; ir_esc_side = 0u; \
        MPU6050_ResetYaw(); } while (0)

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
                if      (k1) { ENTER_AUTO_MODE(MODE_NORMAL);  }   /* 按键1 → 模式1：自动前进 */
                else if (k2) { ENTER_AUTO_MODE(MODE_FUSION);  }   /* 按键2 → 模式2：自动前进 */
                else if (k3) { ENTER_REMOTE_MODE();           }   /* 按键3 → 模式3：停止静止 */
            } else if (!(k1 || k2 || k3)) {
                key_lock = 0u;   /* 全松解锁，允许下次按键触发 */
            }
        }

        /* ---------- 1. 蓝牙指令 ---------- */
        while (osMessageQueueGet(q_bt_cmd, &cmd, NULL, 0) == osOK) {
            /* 2026-08-24：TaskBt 对"非模式3下发的遥控指令"置 arg[0]=1，
             * 此处自动切入模式3 再执行，保证手机端"发W车就走"，与回传文本一致 */
            if ((cmd.arg[0] == 1) && (g_decision.mode != MODE_BLUETOOTH)) {
                ENTER_REMOTE_MODE();   /* 遥控指令跨模式下发 → 先切模式3（随后该指令立即执行） */
            }
            switch (cmd.cmd) {
                case BT_CMD_MODE1: ENTER_AUTO_MODE(MODE_NORMAL); break;  /* 切模式1 → 自动前进 */
                case BT_CMD_MODE2: ENTER_AUTO_MODE(MODE_FUSION); break;  /* 切模式2 → 自动前进 */
                case BT_CMD_MODE3: ENTER_REMOTE_MODE();          break;  /* 切模式3 → 停止静止 */
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
        /* 说明：K230 视觉识别结果由 TaskK230 直写 g_sensor，不经过队列，
         * 2026-08-29 已删除此处原废弃队列(q_k230_cmd)的空转代码。 */

        /* ---------- 2. 快照 + 新鲜度管理 + 分级（互斥：只保留最高级） ---------- */
        snap = g_sensor;
        {
            uint32_t now_tk = osKernelGetTickCount();
            /* 雷达：超过 RADAR_TIMEOUT_MS 无新上报 → 目标视为离开（present 清零） */
            if ((now_tk - snap.radar_tick) >= RADAR_TIMEOUT_MS) snap.radar_present = 0u;
            /* 视觉：目标记忆 VIS_TIMEOUT_MS 后渐消（目标走出画面不永久挂警） */
            if ((now_tk - snap.vis_tick) >= VIS_TIMEOUT_MS)     snap.vis_target_seen = 0u;
            /* 模式2 融合距离：雷达优先，雷达缺席用 K230 视觉上报距离兜底 */
            if (snap.radar_present) {
                snap.fusion_dist_cm = snap.radar_dist_cm; snap.fusion_valid = 1u;
            } else if ((snap.vis_dist_cm <= RADAR_MAX_CM)) {
                snap.fusion_dist_cm = snap.vis_dist_cm;   snap.fusion_valid = 1u;
            } else {
                snap.fusion_valid = 0u;
            }
            g_sensor.fusion_dist_cm = snap.fusion_dist_cm;   /* 同步回全局供显示层 */
            g_sensor.fusion_valid   = snap.fusion_valid;
        }
        /* 2026-08-29：用"上一周期是否运动"做预警屏蔽依据，
         * 保证静止→运动的第一个周期传感器就已参与 */
        g_decision.alert_level = CalcAlertLevel(&snap, prev_moving);

        /* 模式2 融合叠加（只升不降） */
        if ((g_decision.mode == MODE_FUSION) && (snap.fusion_valid)) {
            if ((snap.fusion_dist_cm <= TH_ALARM_CM) && (g_decision.alert_level < ALERT_LEVEL2))
                g_decision.alert_level = ALERT_LEVEL2;
            else if ((snap.fusion_dist_cm <= TH_RADAR_LVL1_CM) && (g_decision.alert_level < ALERT_LEVEL1))
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

        /* ---------- 5. 红外反向转向状态机（2026-08-29 v2：防换向 + 硬上限 + 卡角逃逸） ----------
         * 基本行为：运动中左红外触发→原地右转，右红外触发→原地左转；
         *          双触发→后退脱困（退到两侧都清空为止）；死胡同→原地掉头。
         * v1 缺陷：转向途中左右红外交替触发会中途换向、来回摆动永远转不完，
         *          红外一直触发时无时间上限 → 原地自转四五秒。
         * v2 修复：
         *   (1) 转向中锁定方向：只看触发侧红外是否清除，忽略对侧（防换向）；
         *   (2) 红外清除后仍保持转向 IR_TURN_HOLD_MS（防贴墙）才恢复直行；
         *   (3) 红外一直触发超过 IR_TURN_MAX_MS → 进入卡角逃逸：直行并忽略该侧
         *       红外直到它清除（前方碰撞由超声波侧偏/四级制动兜底）；
         *   (4) 静止状态整体复位，不干扰调试。
         * ir_st：0=直行  1=右转(左触发)  2=左转(右触发)  3=卡角逃逸直行 */
        {
            uint8_t il = (snap.ir_left  == 0u) ? 1u : 0u;   /* 1=左红外有障碍 */
            uint8_t ir = (snap.ir_right == 0u) ? 1u : 0u;   /* 1=右红外有障碍 */
            uint8_t cur_moving = (g_decision.target_speed_l != 0) || (g_decision.target_speed_r != 0);

            if (!cur_moving) {
                /* 静止：转向状态机整体复位（调试免打扰） */
                ir_st = 0u; ir_both = 0u; ir_esc_side = 0u;
            } else if (ir_both) {
                /* 双触发后退脱困中：保持后退直到两侧红外都清空 */
                if (!(il && ir)) ir_both = 0u;      /* 已脱困 → 下一周期重新检测 */
                else {
                    g_decision.target_speed_l = -SPEED_SLOW;
                    g_decision.target_speed_r = -SPEED_SLOW;
                }
            } else if (ir_st == 3u) {
                /* 卡角逃逸：直行，忽略原触发侧红外，该侧清除后恢复检测 */
                uint8_t side_gone = (ir_esc_side == 1u) ? (il == 0u) : (ir == 0u);
                if (side_gone) { ir_st = 0u; ir_esc_side = 0u; }
            } else if (ir_st != 0u) {
                /* 转向中：只看触发侧是否清除，忽略对侧触发（防换向摆动） */
                uint8_t  still = (ir_st == 1u) ? il : ir;
                uint32_t tk = osKernelGetTickCount();
                if (still) {
                    /* 触发侧仍有障碍：未超硬上限继续转，超了进入卡角逃逸 */
                    if ((tk - ir_turn_tk) >= IR_TURN_MAX_MS) {
                        ir_esc_side = ir_st;        /* 记住是哪侧触发的 */
                        ir_st = 3u;                 /* 强制直行逃逸 */
                    }
                } else {
                    /* 触发侧已清除：保持窗口内继续转，窗口到期恢复直行（防贴墙） */
                    if ((tk - ir_turn_tk) >= IR_TURN_HOLD_MS) ir_st = 0u;
                }
            } else if (il && ir) {
                /* 新双触发：前方贴死 → 后退脱困 */
                ir_both = 1u;
                g_decision.target_speed_l = -SPEED_SLOW;
                g_decision.target_speed_r = -SPEED_SLOW;
            } else if (il) {
                ir_st = 1u; ir_turn_tk = osKernelGetTickCount();   /* 左触发 → 原地右转 */
            } else if (ir) {
                ir_st = 2u; ir_turn_tk = osKernelGetTickCount();   /* 右触发 → 原地左转 */
            }

            /* 转向速度输出（ir_st=1/2 时覆盖模式规划的直行速度） */
            if (ir_st == 1u) {                                   /* 原地右转 */
                g_decision.target_speed_l =  SPEED_TURN;
                g_decision.target_speed_r = -SPEED_TURN;
            } else if (ir_st == 2u) {                            /* 原地左转 */
                g_decision.target_speed_l = -SPEED_TURN;
                g_decision.target_speed_r =  SPEED_TURN;
            }

            /* 死胡同：运动中左/右/后均 <20cm → 原地掉头（最高优先，覆盖红外转向） */
            if (cur_moving &&
                DistNear(snap.dist_left_cm,  TH_DEADEND_CM) &&
                DistNear(snap.dist_right_cm, TH_DEADEND_CM) &&
                DistNear(snap.dist_back_cm,  TH_DEADEND_CM)) {
                g_decision.target_speed_l =  SPEED_UTURN;
                g_decision.target_speed_r = -SPEED_UTURN;
            }

            /* ---------- 5.1 moving 标志同步（静止屏蔽用） ---------- */
            cur_moving = (g_decision.target_speed_l != 0) || (g_decision.target_speed_r != 0);
            g_decision.moving = cur_moving;
            prev_moving = cur_moving;
        }

        /* ---------- 6. 四级警报：任何模式（含蓝牙遥控）强制清零 ---------- */
        if (g_decision.alert_level == ALERT_LEVEL4) {
            g_decision.target_speed_l = 0;
            g_decision.target_speed_r = 0;
        }

        /* ---------- 7. 电机使能（2026-09-05 根因修复：放宽窗口 + 锁存） ----------
         * 【根因】超声波改同帧同步轮询后，传感任务每周期要忙等约 5~35ms
         * （等回波，期间不让出 CPU，且其优先级高于决策任务），叠加 20ms osDelay，
         * 实际采样间隔常达 100ms 上下，与旧新鲜度窗口(100ms)同量级：
         *   → fresh 经常为假 → warm_cycles 累加不到 4 → motor_enabled 恒 0
         *   → 电机任务一直走 Stop()=短路制动(IN1=IN2=1+满占空比)
         *   → 表现正是"轮子锁死不转 + 绕组通电的微弱嗡鸣"，且随回波快慢时好时坏。
         * 0.291N 同一段逻辑带着同样隐患，只是当时环境回波快、间隔恰好 <100ms 才侥幸使能。
         * 【修复】① 窗口放宽到 500ms，覆盖真实采样间隔；
         *         ② 使能改锁存：连续 4 次新鲜后永久使能，不再回退，
         *            彻底消除"驱动/制动"来回切换。传感任务真停摆(>500ms 无更新)时
         *            使能不会置位，电机保持制动（安全语义不变）。 */
        if (!g_decision.motor_enabled) {
            uint32_t now = osKernelGetTickCount();
            uint8_t fresh = (g_sensor.update_tick != 0u) && ((now - g_sensor.update_tick) < 500u);
            warm_cycles = fresh ? (uint8_t)(warm_cycles + 1u) : 0u;
            if (warm_cycles >= 4u) {
                g_decision.motor_enabled = 1u;   /* 锁存：使能后不再回退 */
            }
        }

        osDelay(20);
    }
}

/* ============================ 任务3：电机控制 ============================ */
void TaskMotor_Start(void *argument)
{
    (void)argument;
    for (;;) {
        if (g_decision.motor_enabled) {
            /* 2026-08-28：左右轮配平补偿（MOTOR_TRIM_L/R_PCT），
             * 抵消四只电机启动阈值/摩擦不一致导致的直行跑偏。
             * 2026-09-03 修正：配平只叠加在"行驶中"的轮子上——零速轮保持 0，
             * 否则停车时配平值（如右+2）会让该轮带微弱驱动/嗡嗡响。 */
            int16_t tl = g_decision.target_speed_l;
            int16_t tr = g_decision.target_speed_r;
            if (tl != 0) tl = (int16_t)(tl + MOTOR_TRIM_L_PCT);
            if (tr != 0) tr = (int16_t)(tr + MOTOR_TRIM_R_PCT);
            if (tl >  100) tl =  100;
            if (tl < -100) tl = -100;
            if (tr >  100) tr =  100;
            if (tr < -100) tr = -100;
            TB6612_Motor_SetSpeedPercent(tl, tr);
        } else {
            TB6612_Motor_Stop();   /* 未预热/未使能=刹车定住，上电即静止 */
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
                /* 2026-08-28：同时丢弃空格/制表符（手机端可能附带），
                 * 避免 "W " / " w" 这类脏帧被误判为 Unknown */
                if ((char)c == ' ' || (char)c == '\t') { /* 忽略 */ }
                else if (len < (uint8_t)(sizeof(line) - 1u)) line[len++] = (char)c;
            }
        } else {
            /* 30ms 无新字节视为一帧结束 */
            if ((len > 0u) && ((osKernelGetTickCount() - last_byte_tick) >= 30u)) {
                line[len] = '\0';
                /* 2026-08-28：容错处理（修复"只有 MC/TH 正常、其余全回 ERR"）：
                 *  1) 去掉末尾残留空白；
                 *  2) 全部转大写再匹配——手机端发 "w"/"W" 均识别，
                 *     大小写差异是单字母指令误判 Unknown 的最常见原因。 */
                while ((len > 0u) && ((line[len-1u] == ' ') || (line[len-1u] == '\t'))) { len--; line[len] = '\0'; }
                for (uint8_t ui = 0u; ui < len; ui++) {
                    if ((line[ui] >= 'a') && (line[ui] <= 'z')) line[ui] = (char)(line[ui] - 'a' + 'A');
                }
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
                else if (strcmp(line, "GAS") == 0) {
                    /* 2026-08-29 气体/酒精状态查询：与 TH 同构，直接回传不入队列。
                     * 回传 "MQ2:xxx MQ3:xxx OK/ALARM"——数值为 ADC 原始值，
                     * OK=两者均在阈值内，ALARM=任一超标（与四级预警判定一致） */
                    uint16_t m2 = GasSensor_GetMQ2(), m3 = GasSensor_GetMQ3();
                    char rep[40];
                    int n = snprintf(rep, sizeof(rep), "MQ2:%u MQ3:%u %s\r\n",
                                     m2, m3,
                                     ((m2 >= GAS_MQ2_THRESHOLD) || (m3 >= GAS_MQ3_THRESHOLD))
                                         ? "ALARM" : "OK");
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

/* ============================ 任务5：K230 视觉通信（2026-08-28 实现） ============================
 * 协议约定（ASCII 行，USART3 115200，K230 端按此格式输出即可对接）：
 *   "PERSON"            → 识别到人体（无距离）
 *   "CAR"               → 识别到车辆（无距离）
 *   "PERSON,52"         → 识别到人体，目标距离 52cm（K230 侧估计/测距）
 *   "CAR,80"            → 识别到车辆，目标距离 80cm
 *   其余行忽略。行以 '\r' 或 '\n' 结束。
 * 作用：vis_target_seen 参与一级预警；vis_dist_cm 在模式2雷达缺席时兜底融合。 */
static void K230_ParseLine(const char *line)
{
    uint8_t  is_person = (strncmp(line, "PERSON", 6u) == 0);
    uint8_t  is_car    = (strncmp(line, "CAR  ", 3u) == 0);
    uint16_t dist = 0xFFFFu;

    if (!is_person && !is_car) return;

    /* 可选 ",距离" 后缀（cm） */
    const char *comma = strchr(line, ',');
    if (comma != NULL) {
        int d = atoi(comma + 1);
        if ((d > 0) && (d <= (int)RADAR_MAX_CM)) dist = (uint16_t)d;
    }

    g_sensor.vis_target_seen = 1u;
    g_sensor.vis_tick        = osKernelGetTickCount();
    g_sensor.vis_dist_cm     = dist;
}

void TaskK230_Start(void *argument)
{
    (void)argument;
    static char line[32];
    uint8_t len = 0u;

    HAL_UART_Receive_IT(&huart3, &s_k230_rx_byte, 1);   /* 启动中断接收 */

    for (;;) {
        int c = K230_RingGet();
        if (c >= 0) {
            if ((c == '\r') || (c == '\n')) {
                if (len > 0u) { line[len] = '\0'; K230_ParseLine(line); len = 0u; }
            } else if (len < (uint8_t)(sizeof(line) - 1u)) {
                line[len++] = (char)c;
            }
        } else {
            osDelay(10);
        }
    }
}

/* ============================ 任务7：LD2450 毫米波雷达（2026-08-28 实现，09-04 重写协议） ============================
 * 协议（官方数据帧，USART6 256000 8N1，持续上报，约 30Hz）：
 *   帧头 4B = AA FF 03 00
 *   目标数据 3×8=24B：每目标 X(2B)|Y(2B)|速度(2B)|分辨率(2B)，小端；
 *     X/Y 最高位为符号位（1=正、0=负），低 7 位为数值（mm）；
 *   帧尾 2B = 55 CC
 *   距离 = sqrt(X²+Y²)（X 横向 ±300cm、Y 纵深 0~600cm）。
 * 2026-09-04 重写根因：旧状态机帧头误为 55 AA 03，永远无法同步 → R 恒 "--"。
 * 作用：取最近目标距离 radar_dist_cm；≤1.5m→二级，(1.5,3]m→一级；
 *       超过 RADAR_TIMEOUT_MS 无新帧视为目标离开（决策层清零）。 */
typedef enum {
    RD_SYNC0 = 0, RD_SYNC1, RD_SYNC2, RD_SYNC3, RD_DATA, RD_TAIL_55, RD_TAIL_CC
} RadarParseState_t;

/* 坐标解码：高 7 位为幅值(mm)，最高位 1=正、0=负 → 返回有符号 cm */
static int16_t Radar_DecodeCm(uint8_t lo, uint8_t hi)
{
    int16_t mag = (int16_t)((((int16_t)hi & 0x7F) << 8) | (int16_t)lo);
    return ((hi & 0x80u) != 0u) ? (int16_t)(mag / 10) : (int16_t)(-(mag / 10));
}

/* 整数平方根（牛顿迭代，n≥0） */
static uint16_t Radar_Isqrt(int32_t n)
{
    if (n <= 0) return 0u;
    int32_t x = n, y = (x + 1) / 2;
    while (y < x) { x = y; y = (x + n / x) / 2; }
    return (uint16_t)x;
}

static void Radar_ParseTargets(const uint8_t *data, uint16_t len)
{
    uint16_t min_dist = 0xFFFFu;
    uint8_t  has_target = 0u;
    uint16_t i;

    for (i = 0u; (i + 8u) <= len; i = (uint16_t)(i + 8u)) {
        int16_t x = Radar_DecodeCm(data[i],      data[i + 1u]);   /* 横向 cm */
        int16_t y = Radar_DecodeCm(data[i + 2u], data[i + 3u]);   /* 纵深 cm */
        /* 速度字段：编码方式与坐标相同（高 7 位幅值 + 最高位符号），单位 cm/s。
         * 2026-09-05 启用：用于过滤静止目标（墙、桌腿、家具等），解决
         * "模式3 静止时雷达报 30cm，切模式1/2 数值不变直接触发二级误报"。 */
        int16_t v = Radar_DecodeCm(data[i + 4u], data[i + 5u]);   /* 速度 cm/s */
        int32_t d2 = (int32_t)x * x + (int32_t)y * y;             /* 最大约 45 万，int32 安全 */
        uint16_t d = Radar_Isqrt(d2);

        if (d == 0u) continue;          /* 空目标（X=Y=0）跳过 */

        /* 静止目标过滤：|速度| 低于阈值视为静止（LD2450 对完全静止目标输出 0），
         * 不参与预警——静止障碍由四路超声波负责，雷达只负责"运动目标"预警。
         * 阈值见 RADAR_MIN_SPEED_CMS，调大=更抗误报，调 0=关闭过滤（旧行为）。 */
        {
            int16_t av = (v < 0) ? (int16_t)-v : v;
            if (av < (int16_t)RADAR_MIN_SPEED_CMS) continue;
        }

        has_target = 1u;
        if (d < min_dist) min_dist = d; /* 取最近目标 */
    }

    if (has_target && (min_dist <= RADAR_MAX_CM)) {
        g_sensor.radar_present = 1u;
        g_sensor.radar_dist_cm = min_dist;
        g_sensor.radar_tick    = osKernelGetTickCount();
    }
}

void TaskRadar_Start(void *argument)
{
    (void)argument;
    static uint8_t frame[24];           /* 3 目标 × 8 字节 */
    static RadarParseState_t st = RD_SYNC0;
    static uint8_t got = 0u;

    HAL_UART_Receive_IT(&huart6, &s_radar_rx_byte, 1);   /* 启动中断接收 */

    for (;;) {
        int c = Radar_RingGet();
        if (c < 0) { osDelay(5); continue; }
        uint8_t b = (uint8_t)c;

        switch (st) {
        case RD_SYNC0: if (b == 0xAAu) st = RD_SYNC1; break;                 /* 帧头第 1 字节 */
        case RD_SYNC1: st = (b == 0xFFu) ? RD_SYNC2 : ((b == 0xAAu) ? RD_SYNC1 : RD_SYNC0); break;
        case RD_SYNC2: st = (b == 0x03u) ? RD_SYNC3 : RD_SYNC0; break;
        case RD_SYNC3: st = (b == 0x00u) ? RD_DATA   : RD_SYNC0;
                       if (st == RD_DATA) got = 0u; break;
        case RD_DATA:
            frame[got++] = b;
            if (got >= 24u) st = RD_TAIL_55;
            break;
        case RD_TAIL_55: st = (b == 0x55u) ? RD_TAIL_CC : RD_SYNC0; break;
        case RD_TAIL_CC:
            if (b == 0xCCu) Radar_ParseTargets(frame, 24u);   /* 帧完整，解析 */
            st = RD_SYNC0;
            break;
        default: st = RD_SYNC0; break;
        }
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

    /* --- 开机版本横幅：上电先显示 2 秒固件版本号，一眼确认板内是否最新固件，
     * 杜绝"改了代码没重新烧录"导致的误判。版本号约定：FW_Vx.y，每次交付递增。
     * 2026-09-05：电机不转真因已确认为 LM2596 稳压模块负载不足（供电问题），
     * 排查期临时加的 RST 复位原因探针已移除，恢复纯净横幅。 */
    #define FW_VERSION_STR "FW V3.9"
    OLED_Clear();
    OLED_ShowString(0, 8,  "SmartCar",  OLED_8X16);
    OLED_ShowString(0, 24, FW_VERSION_STR, OLED_8X16);
    OLED_ShowString(0, 40, "2026-09-05", OLED_6X8);
    OLED_Update();
    osDelay(2000);
    #undef FW_VERSION_STR

    for (;;) {
        /* 可靠性双保险（本任务 50ms 周期，是唯一稳定慢节奏任务）：
         *  1) 喂独立看门狗（20s 超时，见 app_rtos.c 注释）：任何任务死锁/跑飞
         *     导致本任务停摆，看门狗自动整机复位；
         *  2) OLED I2C 自检：I2C2 卡 BUSY 或 SDA 被拉死时自动恢复总线并重初始化，
         *     花屏/黑屏一帧内自愈，不再累积错位乱码。 */
        App_Watchdog_Feed();
        OLED_I2C_SelfCheck();

        /* 2026-09-04 故障可视：若某任务栈溢出（钩子写入任务名），
         * 全屏打出 "OVF:<任务名>" 并停在本任务，直观定位哪个任务栈不够。
         * 正常运行该串为空，不进入此分支。 */
        if (g_stack_overflow_task[0] != '\0') {
            char fb[20];
            snprintf(fb, sizeof(fb), "OVF:%s", g_stack_overflow_task);
            OLED_Clear();
            OLED_ShowString(0, 0,  "STACK OVERFLOW", OLED_8X16);
            OLED_ShowString(0, 24, fb, OLED_8X16);
            OLED_ShowString(0, 48, "Fix task stack!", OLED_6X8);
            OLED_Update();
            while (1) { App_Watchdog_Feed(); osDelay(200); }  /* 停住看现场 */
        }

        /* 2026-09-04：首帧整屏清一次，擦除开机横幅("SmartCar/FW/日期")残留。
         * 原缺陷：横幅的 8x16 大字("Car")与日期(".5")落在 F/L、R/B 距离行的
         * 空白像素上，而距离行只重绘自身字符、不擦空白 → 残影混入（照片红圈处）。
         * 本循环每帧完整重绘所有行，清一次即永久干净。 */
        {
            static uint8_t first_frame = 1u;
            if (first_frame) { first_frame = 0u; OLED_Clear(); }
        }

        uint8_t lv = g_decision.alert_level;

        /* --- LED 互斥（2026-08-28 四级规则）：
         *   一级=绿灯；二级与三级共用黄灯；四级=红灯；无预警全灭（灌电流低电平点亮） --- */
        HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, (lv == ALERT_LEVEL1) ? GPIO_PIN_RESET : GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED_Y_GPIO_Port, LED_Y_Pin,
            ((lv == ALERT_LEVEL2) || (lv == ALERT_LEVEL3)) ? GPIO_PIN_RESET : GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, (lv == ALERT_LEVEL4) ? GPIO_PIN_RESET : GPIO_PIN_SET);

        /* --- 蜂鸣器分档（四级规则）：
         *   一级=静音（仅绿灯）；二级=3kHz 50ms间歇；三级=2.5kHz 滴答变调（与二级区分）；
         *   四级=4kHz 长鸣 --- */
        static uint8_t tick_cnt = 0;
        tick_cnt = (uint8_t)((tick_cnt + 1u) & 0x0Fu);
        switch (lv) {
            case ALERT_LEVEL1:   /* 一级：只亮绿灯，蜂鸣器静音 */
                Beep_Off();
                break;
            case ALERT_LEVEL2:   /* 二级：3kHz，50ms 响 / 50ms 停 */
                if (tick_cnt & 0x01u) Beep_SetFreq(BEEP_FREQ_LVL2_HZ); else Beep_Off();
                break;
            case ALERT_LEVEL3:   /* 三级：滴答变调，2.5k 与 2k 交替短音（间歇节奏与二级不同） */
                if (tick_cnt & 0x04u) {
                    Beep_SetFreq((tick_cnt & 0x02u) ? BEEP_FREQ_LVL3A_HZ : BEEP_FREQ_LOW_HZ);
                } else {
                    Beep_Off();
                }
                break;
            case ALERT_LEVEL4:   /* 四级：4kHz 长鸣 */
                Beep_SetFreq(BEEP_FREQ_LVL4_HZ);
                break;
            default:
                Beep_Off();
                break;
        }

        /* --- OLED 布局（64px 高，混排字体，5 行） ---
         * y0  (6x8)  模式+当前等级+温湿度：M:1 L:2 T:26C H:55
         * y8  (8x16) 前/左距离
         * y24 (8x16) 右/后距离
         * y40 (6x8)  气体状态+雷达距离（2026-08-28 航向角行让位给雷达，R:--=无目标）
         * y48 (6x8)  最下排：ALARM 触发源串（1/2/3/4 并排显示） */
        char buf[28];
        const char *lv_str = (lv == ALERT_NONE)   ? "L:0 " :
                             (lv == ALERT_LEVEL1) ? "L:1 " :
                             (lv == ALERT_LEVEL2) ? "L:2 " :
                             (lv == ALERT_LEVEL3) ? "L:3 " : "L:4 ";
        snprintf(buf, sizeof(buf), "M:%u %s T:%dC H:%u",
                 g_decision.mode, lv_str, (int)g_sensor.temp_c, g_sensor.humi_pct);
        OLED_ShowString(0, 0, "                       ", OLED_6X8);
        OLED_ShowString(0, 0, buf, OLED_6X8);

        OLED_ShowString(0, 8,  "F:", OLED_8X16);  OLED_ShowDistCm(16, 8,  g_sensor.dist_front_cm);
        OLED_ShowString(64, 8, "L:", OLED_8X16);  OLED_ShowDistCm(80, 8,  g_sensor.dist_left_cm);
        OLED_ShowString(0, 24, "R:", OLED_8X16);  OLED_ShowDistCm(16, 24, g_sensor.dist_right_cm);
        OLED_ShowString(64, 24,"B:", OLED_8X16);  OLED_ShowDistCm(80, 24, g_sensor.dist_back_cm);

        {
            /* 第4行：气体状态 + 雷达距离（Q2/Q3 = MQ-2/MQ-3 是否在阈值内，
             * R:xx = 雷达最近运动目标距离 cm，R:-- = 无目标）。
             * 2026-09-05：排查电机问题用的 HB/CCR 诊断探针已移除，恢复纯净显示。 */
            char rstr[8];
            if (g_sensor.radar_present) snprintf(rstr, sizeof(rstr), "%u", (unsigned)g_sensor.radar_dist_cm);
            else                        snprintf(rstr, sizeof(rstr), "--");
            snprintf(buf, sizeof(buf), "Q2:%s Q3:%s R:%s",
                     g_sensor.mq2_ok ? "OK" : "ER",
                     g_sensor.mq3_ok ? "OK" : "ER",
                     rstr);
        }
        OLED_ShowString(0, 40, "                     ", OLED_6X8);
        OLED_ShowString(0, 40, buf, OLED_6X8);

        /* 最下排：当前所有处于触发状态的预警档位并排打印（1/2/3/4） */
        {
            char l3[20];
            l3[0] = '\0';
            uint8_t mv = g_decision.moving;  /* 2026-08-29：静止不显示触发源（气体四级除外） */
            if (mv && (DistNear(g_sensor.dist_front_cm, TH_WARN_CM) || g_sensor.vis_target_seen ||
                (g_sensor.radar_present && (g_sensor.radar_dist_cm <= TH_RADAR_LVL1_CM)))) strcat(l3, "1 ");
            if (mv && (DistNear(g_sensor.dist_front_cm, TH_ALARM_CM) ||
                DistNear(g_sensor.dist_left_cm,  TH_SIDE_CM)  ||
                DistNear(g_sensor.dist_right_cm, TH_SIDE_CM)  ||
                DistNear(g_sensor.dist_back_cm,  TH_SIDE_CM)  ||
                (g_sensor.radar_present && (g_sensor.radar_dist_cm <= TH_RADAR_LVL2_CM)))) strcat(l3, "2 ");
            if (mv && ((g_sensor.ir_left == 0u) || (g_sensor.ir_right == 0u))) strcat(l3, "3 ");
            if ((mv && (DistNear(g_sensor.dist_front_cm, TH_CRITICAL_CM) ||
                DistNear(g_sensor.dist_left_cm,  TH_CRITICAL_CM) ||
                DistNear(g_sensor.dist_right_cm, TH_CRITICAL_CM) ||
                DistNear(g_sensor.dist_back_cm,  TH_CRITICAL_CM))) ||
                (g_sensor.mq2_ok == 0u) || (g_sensor.mq3_ok == 0u)) strcat(l3, "4 ");
            snprintf(buf, sizeof(buf), "ALARM:%-13s", (l3[0] ? l3 : "--"));
            OLED_ShowString(0, 48, "                     ", OLED_6X8);
            OLED_ShowString(0, 48, buf, OLED_6X8);
        }

        OLED_Update();
        osDelay(50);
    }
}
