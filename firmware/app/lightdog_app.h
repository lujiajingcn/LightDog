/*
 * LightDog 固件应用层：串口帧分发 + 输出决策 + 状态上报。
 *
 * 这一层和 core/ 的分工：
 *   core/lightdog_proto     帧的编解码与流式解析（不知道有电机这回事）
 *   core/lightdog_watchdog  允不允许出力（不知道有串口这回事）
 *   app/lightdog_app        把两者接起来：收到什么、这一拍给电机什么
 *
 * 为什么这一层也要能在 PC 上编译：它包含「失联时把目标速度清零」「急停立刻
 * 断使能」「上位机周期重发 ESTOP=0 不能把 hb_count 刷掉」这类判据 —— 每一条
 * 写错都表现为「上车才发现的怪现象」，而现场没有调试手段。
 * 现在用 test/hal_stub.c + test/test_app.c 在 PC 上把它们跑完。
 *
 * 上下文约定（很重要）：
 *   ld_app_rx()        在 **UART ISR** 里调用 —— 只解析与记账，不发但不阻塞
 *   ld_app_tick()      在 **1kHz 主循环 / 定时器**里调用 —— 输出与上报都在这里
 *   ld_app_on_button() 在 **急停按钮 ISR** 里调用 —— 只断使能与改状态
 * ISR 里不要调用 uart_write（会阻塞），所以上报一律走 tick。
 */
#ifndef LIGHTDOG_APP_H
#define LIGHTDOG_APP_H

#include <stdint.h>

#include "lightdog_hal.h"
#include "lightdog_proto.h"
#include "lightdog_watchdog.h"

typedef struct {
    /* 机构参数：这些必须按实车填，填错的表现是「里程计与导航对不上」，
     * 而不是报错。默认值对应 linorobot2 4wd（65mm 轮 / 300mm 轮距）。 */
    float wheel_radius_m;      /* 轮半径 */
    float wheel_base_m;        /* 左右轮中心距 */
    float ticks_per_rev;       /* 每转计数（编码器线数 x 倍频 x 减速比） */

    /* 速度限幅。上位机出 bug 发出 10m/s 时，固件不能照做。 */
    float max_linear_mps;
    float max_angular_rps;

    /* 上报周期（ms）。0 表示关闭该项。 */
    uint16_t battery_period_ms;
    uint16_t odom_period_ms;
    /* 急停/故障状态的重发周期。串口没有 DDS 的 TRANSIENT_LOCAL，
     * 丢一帧就是丢了，所以必须周期重发而不是只发变化。 */
    uint16_t state_period_ms;

    /* ---- 安全阈值（F-010：把协议里「过流/欠压/过温保护关 PWM」真正接上）----
     * OVERCURRENT / UNDERVOLTAGE / OVERTEMP 命中时 tick 会强制断使能（等于一次
     * 内部急停），故障恢复并经迟滞解除后，仍需重新攒够 3 帧心跳才恢复出力。
     * ENCODER 只置位上报、不断电（编码器坏了车还能停，不至于因一次计数异常锁死）。
     * 阈值 <=0 表示该项检查关闭；safety_period_ms=0 表示整体关闭。 */
    uint16_t safety_period_ms;   /* 安全评估周期（建议 ~100ms） */
    float    max_current_a;      /* 电流超过即过流 */
    float    min_voltage_v;      /* 电压低于即欠压 */
    float    max_temp_c;         /* 温度高于即过温（需要 hal.temp_read） */
} ld_app_cfg_t;

typedef struct {
    const ld_hal_t *hal;
    ld_app_cfg_t cfg;

    ld_parser_t parser;
    ld_wd_t wd;

    /* 最近一条合法 CMD_VEL（已限幅）。失联时会被清零。 */
    float cmd_linear;
    float cmd_angular;

    float led_brightness;
    /* 这三个字段由 UART RX ISR / 按钮 EXTI ISR 写、由 1kHz tick 读。
     * volatile 只保证编译器每次真的去内存取，互斥仍靠 app_enter_critical。 */
    volatile uint8_t estop_from_host;   /* 上位机最近一次下发的要求 */
    volatile uint8_t button_pressed;    /* 物理按钮当前状态（按下时上位机不能解除） */
    volatile uint8_t state_dirty;       /* 急停/故障有变化，下一拍立刻上报 */

    /* 安全故障（F-010）运行时状态 */
    uint8_t  safety_lock;      /* 电气/热故障触发的内部断使能闭锁 */
    uint8_t  oc_tripped;       /* 过流迟滞锁存：置位后需降到释放阈值才解除 */
    uint8_t  uv_tripped;       /* 欠压迟滞锁存 */
    uint8_t  ot_tripped;       /* 过温迟滞锁存 */
    uint32_t last_safety_ms;   /* 上次安全评估时刻 */

    /* 里程计 */
    float odom_x;
    float odom_y;
    float odom_yaw;
    float odom_v;
    float odom_w;
    int32_t last_enc_l;
    int32_t last_enc_r;
    uint8_t odom_started;

    uint32_t last_tick_ms;
    uint32_t last_battery_ms;
    uint32_t last_odom_ms;
    uint32_t last_state_ms;

    /* 统计（供 FAULT 上报与现场排查） */
    uint32_t frames_ok;
    uint32_t frames_bad;
    uint32_t cmd_frames;
} ld_app_t;

void ld_app_cfg_defaults(ld_app_cfg_t *cfg);

void ld_app_init(ld_app_t *app, const ld_hal_t *hal, const ld_app_cfg_t *cfg);

/* UART 收到一段字节（ISR 上下文）。 */
void ld_app_rx(ld_app_t *app, const uint8_t *buf, uint32_t len);

/* 1kHz 主循环。负责输出决策与周期上报。 */
void ld_app_tick(ld_app_t *app, uint32_t now_ms);

/* 物理急停按钮（ISR 上下文）。按下立即断使能，不等通信。
 * 约定用**自锁**按钮：旋出（pressed=0）即解除；解除后仍需 3 帧心跳才恢复出力。
 * 若装的是点动按钮，把解除那一段删掉、只保留上位机 clear。 */
void ld_app_on_button(ld_app_t *app, int pressed, uint32_t now_ms);

#endif /* LIGHTDOG_APP_H */
