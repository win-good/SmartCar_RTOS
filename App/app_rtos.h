/**
 ******************************************************************************
 * @file    app_rtos.h
 * @brief   毕设智能小车 FreeRTOS 任务框架 —— 公共定义（任务句柄/队列/共享数据）
 * @note    对齐《毕设引脚分配表 STM32F407VGT6（V2 定稿）》与双主控三模式架构。
 *
 * 【2026-08-28 预警体系改四级（用户定稿，原2.5级升3级、原3级升4级）】
 *   一级预警(ALERT_LEVEL1)：前超声波<1m 或 视觉识别到目标(人体/车辆)
 *                         或 雷达目标在(0.5m,2m] → 减速；【绿灯，不响蜂鸣器】
 *   二级预警(ALERT_LEVEL2)：前超声波<30cm 或 侧/后超声波<20cm
 *                         或 雷达距离≤0.5m(含雷达+视觉共同确认) → 转向避让；
 *                         黄灯；3kHz 50ms 间歇
 *   三级预警(ALERT_LEVEL3)：任一红外触发(左≈10cm/右≈1cm) → 黄灯(与2级共用)；
 *                         2.5kHz 滴答变调(频率/间歇与2级不同)；自动转向/死胡同掉头
 *   四级警报(ALERT_LEVEL4)：任一超声波<3cm 或 烟雾/酒精超阈值
 *                         → 红灯 + 4kHz 长鸣 + 任何模式速度强制清零
 *   显示互斥：同一时刻 LED/蜂鸣器/OLED 只显示当前最高级，低级不残留。
 *   四路超声波全部参与联动：前=1/2/4级；左/右/后=2级(<20cm)与4级(<3cm)，
 *   并参与转向选向、倒车保护与死胡同掉头。红外只反馈 3 级。
 *
 * 蜂鸣器硬件：无源、低电平触发模块，引脚 PD12(TIM4_CH1)——PB0 未从排针引出，
 * 按原理图改接空闲脚；静默=IO 常高，分档 2k/2.5k/3k/4kHz（beep 模块）。
 ******************************************************************************
 */
#ifndef APP_RTOS_H
#define APP_RTOS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "cmsis_os2.h"
#include <stdint.h>

/* ============================ 引脚宏映射 ============================
 * CubeMX main.h 采用板级丝印命名，此层映射为语义命名，便于任务代码阅读。
 *   HCSR04Q/Z/Y/H = 前/左/右/后 超声波 Trig（PE2~PE5）
 *   HW01Z=左前红外(三级探测器,电位器≈10cm)  HW01Y=右前红外(三级探测器,电位器≈1cm)
 *   LED2(PE14)=绿/一级  LED3(PE15)=黄/二级与三级  LED1(PB5)=红/四级
 * 若日后在 CubeMX 中把 Label 直接改成语义名并重新生成，可删除本映射块。 */
#define Trig_F_Pin        HCSR04Q_Pin
#define Trig_F_GPIO_Port  HCSR04Q_GPIO_Port
#define Trig_L_Pin        HCSR04Z_Pin
#define Trig_L_GPIO_Port  HCSR04Z_GPIO_Port
#define Trig_R_Pin        HCSR04Y_Pin
#define Trig_R_GPIO_Port  HCSR04Y_GPIO_Port
#define Trig_B_Pin        HCSR04H_Pin
#define Trig_B_GPIO_Port  HCSR04H_GPIO_Port
#define IR_L_Pin          HW01Z_Pin
#define IR_L_GPIO_Port    HW01Z_GPIO_Port
#define IR_R_Pin          HW01Y_Pin
#define IR_R_GPIO_Port    HW01Y_GPIO_Port
#define LED_G_Pin         LED2_Pin
#define LED_G_GPIO_Port   LED2_GPIO_Port
#define LED_Y_Pin         LED3_Pin
#define LED_Y_GPIO_Port   LED3_GPIO_Port
#define LED_R_Pin         LED1_Pin
#define LED_R_GPIO_Port   LED1_GPIO_Port

/* ============================ 常量定义 ============================ */
#define MODE_NORMAL      1u   /* 模式1：普通避障（红外+超声波，无视觉）          */
#define MODE_FUSION      2u   /* 模式2：高级智能融合避障（K230视觉+雷达+姿态）    */
#define MODE_BLUETOOTH   3u   /* 模式3：蓝牙遥控（人控+机警，危险工况自动预警）   */

/* 四级预警语义（2026-08-28 定稿）：数值越大等级越高，显示互斥只显最高级 */
#define ALERT_NONE       0u   /* 无预警                                          */
#define ALERT_LEVEL1     1u   /* 一级：前超声波<1m/视觉目标/雷达(0.5,2m]，绿灯静音 */
#define ALERT_LEVEL2     2u   /* 二级：前<30cm 或 侧/后<20cm 或 雷达≤0.5m，黄灯    */
#define ALERT_LEVEL3     3u   /* 三级：任一红外触发，黄灯+滴答音+自动转向          */
#define ALERT_LEVEL4     4u   /* 四级：任一超声波<3cm 或气体超标，红灯长鸣强制制动  */

#define CMD_MAX_ARGS     4    /* 每条指令最多携带的参数个数                        */

/* ============================ 共享数据结构 ============================ */

/* 传感器数据汇总（TaskSensor 周期写入，其余任务只读） */
typedef struct {
    uint16_t dist_front_cm;    /* 前超声波距离，0xFFFF = 超量程/无效 */
    uint16_t dist_left_cm;
    uint16_t dist_right_cm;
    uint16_t dist_back_cm;
    uint8_t  ir_left;          /* 左前红外(2.5级探测器)：0=障碍<10cm  1=无障碍 */
    uint8_t  ir_right;         /* 右前红外(3级探测器) ：0=障碍<1cm   1=无障碍 */
    int16_t  yaw_deg10;        /* MPU6050 累积航向角 ×10（互补滤波，°×10） */
    int16_t  gz_dps10;         /* MPU6050 Z 轴角速度 ×10（°/s ×10） */
    uint8_t  mpu_ok;           /* MPU6050 通信：1=正常 0=异常 */
    uint16_t mq2_raw;          /* MQ-2 烟雾 ADC 原始值 0~4095 */
    uint16_t mq3_raw;          /* MQ-3 酒精 ADC 原始值 0~4095 */
    uint8_t  mq2_ok;           /* MQ-2 状态：1=正常 0=超阈值（错误） */
    uint8_t  mq3_ok;           /* MQ-3 状态：同上 */
    int8_t   temp_c;           /* DHT11 温度 ℃ */
    uint8_t  humi_pct;         /* DHT11 湿度 %RH */
    uint8_t  dht_ok;           /* DHT11 最近一次读取：1=成功 0=失败 */
    /* ---- LD2450 毫米波雷达（USART6 256000，TaskRadar 写入，2026-08-28 启用） ---- */
    uint8_t  radar_present;    /* 1=雷达探测到目标（目标1在范围内且已过滤静止目标） */
    uint16_t radar_dist_cm;    /* 雷达最近目标距离（cm）；无目标时保持上次值 */
    uint32_t radar_tick;       /* 雷达最近一次有效上报的 tick（新鲜度/超时判断） */
    /* ---- K230 视觉（USART3，TaskK230 写入，2026-08-28 启用） ---- */
    uint8_t  vis_target_seen;  /* 1=视觉识别到目标（人体或车辆） */
    uint32_t vis_tick;         /* 视觉最近一次识别到目标的 tick（超时自动清零） */
    uint16_t vis_dist_cm;      /* K230 上报的目标距离（cm），0xFFFF=未提供/无效 */
    /* ---- 模式2 融合距离：雷达距离为主（K230 视觉暂不输出距离） ---- */
    uint16_t fusion_dist_cm;   /* 决策任务用雷达+视觉刷新（cm），0xFFFF=无效 */
    uint8_t  fusion_valid;     /* 融合距离有效标志（决策任务写入） */
    uint32_t update_tick;      /* 最近一次更新的系统 tick，供数据新鲜度判断 */
} SensorData_t;

/* 决策结果（TaskDecision 写入，电机/显示任务只读） */
typedef struct {
    uint8_t mode;              /* 当前工作模式 MODE_xxx */
    uint8_t alert_level;       /* 当前预警等级 ALERT_xxx（显示互斥，只此一级） */
    int16_t target_speed_l;    /* 左电机目标速度：-100 ~ +100（负值后退） */
    int16_t target_speed_r;    /* 右电机目标速度：同上 */
    uint8_t  motor_enabled;    /* 1=电机允许运行  0=紧急制动 */
    int16_t  rc_speed_l;       /* 模式3 蓝牙遥控目标速度（TaskBt/Decision 写入） */
    int16_t  rc_speed_r;
    uint8_t  rc_active;        /* 1=本周期内有遥控指令（模式3 才使用） */
} Decision_t;

/* 蓝牙/K230 解析后的指令（队列元素） */
typedef struct {
    uint8_t cmd;               /* 指令码（协议层定义） */
    int16_t arg[CMD_MAX_ARGS]; /* 参数（如目标速度、识别类别、距离等） */
} AppCmd_t;

/* 蓝牙指令码（与手机端发送字符串一一对应，详见 app_tasks.c 协议表注释） */
#define BT_CMD_MODE1      0x01u  /* "MA" → 模式1 */
#define BT_CMD_MODE2      0x02u  /* "MB" → 模式2 */
#define BT_CMD_MODE3      0x03u  /* "MC" → 模式3 */
#define BT_CMD_FWD        0x10u  /* "W"  → 遥控前进 */
#define BT_CMD_BACK       0x11u  /* "S"  → 遥控后退 */
#define BT_CMD_TURNL      0x12u  /* "A"  → 遥控左转 */
#define BT_CMD_TURNR      0x13u  /* "D"  → 遥控右转 */
#define BT_CMD_UTURN      0x14u  /* "U"  → 遥控掉头 */
#define BT_CMD_STOP       0x15u  /* "X"  → 遥控停止 */
#define BT_CMD_TH_QUERY   0x20u  /* "TH" → 查询温湿度（板端回传） */

/* ============================ 全局实例（extern） ============================ */
extern SensorData_t g_sensor;
extern Decision_t   g_decision;

extern osMessageQueueId_t q_bt_cmd;   /* 蓝牙指令队列（TaskBt  → TaskDecision） */
extern osMessageQueueId_t q_k230_cmd; /* K230 视觉消息队列（TaskK230 → TaskDecision） */

/* ============================ 任务句柄（extern） ============================ */
extern osThreadId_t t_sensor;    /* 传感采集 */
extern osThreadId_t t_decision;  /* 决策状态机 */
extern osThreadId_t t_motor;     /* 电机控制 */
extern osThreadId_t t_bt;        /* 蓝牙通信 */
extern osThreadId_t t_k230;      /* K230 通信（视觉） */
extern osThreadId_t t_radar;     /* LD2450 毫米波雷达（2026-08-28 新增） */
extern osThreadId_t t_display;   /* 显示与声光预警 */

/* ============================ 接口函数 ============================ */
void App_Init(void);   /* 创建全部队列与任务，在 MX_FREERTOS_Init() 中调用 */

/* 独立看门狗（2026-08-24 新增）：
 *   App_Watchdog_Init  在 main() 的 HAL_Init 之后尽早调用（IWDG 用 LSI 独立时钟，
 *                      不依赖系统时钟是否配置成功），超时约 2.7s，一次启动不可关闭；
 *   App_Watchdog_Feed  由 TaskDisplay 每 50ms 喂一次。任何任务死锁/死循环/跑飞
 *                      导致停喂，看门狗自动整机复位恢复——"复位后偶发无响应只能
 *                      断电恢复"从此变为自动恢复。 */
void App_Watchdog_Init(void);
void App_Watchdog_Feed(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_RTOS_H */
