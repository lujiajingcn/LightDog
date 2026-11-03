/*
 * LightDog 固件应用层实现（纯 C99，只依赖 app/ 与 core/）。
 *
 * 这里没有一行 HAL 寄存器操作：所有外设动作都通过 ld_hal_t 的函数指针出去。
 * 所以这个文件连同 core/ 一起能在 PC 上编译、在 PC 上测 —— 上车前就能把
 * 「失联会不会停」「急停会不会被心跳刷掉」这些判据跑完。
 */
#include "lightdog_app.h"

#include <math.h>
#include <string.h>

#define LD_PI        3.14159265358979f
#define LD_TWO_PI    6.28318530717959f

/* 协议约定小端（docs/firmware_protocol.md §六）。
 * 目标平台（ARM Cortex-M / x86）都是小端，所以直接 memcpy；
 * 移植到大端机器时改这两个函数，别在调用点各写一遍。 */
static void put_f32(uint8_t *p, float v)
{
    memcpy(p, &v, sizeof(v));
}

static float get_f32(const uint8_t *p)
{
    float v = 0.0f;
    memcpy(&v, p, sizeof(v));
    return v;
}

static float clampf(float v, float lo, float hi)
{
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

static float wrap_pi(float a)
{
    while (a > LD_PI) {
        a -= LD_TWO_PI;
    }
    while (a < -LD_PI) {
        a += LD_TWO_PI;
    }
    return a;
}

/* ---------------------------------------------------------------- 配置 / 初始化 */

void ld_app_cfg_defaults(ld_app_cfg_t *cfg)
{
    if (cfg == NULL) {
        return;
    }
    cfg->wheel_radius_m = 0.0325f;
    cfg->wheel_base_m = 0.300f;
    cfg->ticks_per_rev = 1560.0f;
    cfg->max_linear_mps = 0.50f;
    cfg->max_angular_rps = 1.50f;
    cfg->battery_period_ms = 1000u;
    cfg->odom_period_ms = 50u;
    cfg->state_period_ms = 500u;
}

void ld_app_init(ld_app_t *app, const ld_hal_t *hal, const ld_app_cfg_t *cfg)
{
    uint32_t now = 0u;

    if (app == NULL || hal == NULL) {
        return;
    }
    memset(app, 0, sizeof(*app));
    app->hal = hal;

    if (cfg != NULL) {
        app->cfg = *cfg;
    } else {
        ld_app_cfg_defaults(&app->cfg);
    }

    /* 配置兜底：除零会直接 HardFault，而 HardFault 在车上是「突然不动」 */
    if (app->cfg.wheel_base_m <= 0.0f) {
        app->cfg.wheel_base_m = 0.300f;
    }
    if (app->cfg.ticks_per_rev <= 0.0f) {
        app->cfg.ticks_per_rev = 1560.0f;
    }
    if (app->cfg.wheel_radius_m <= 0.0f) {
        app->cfg.wheel_radius_m = 0.0325f;
    }

    ld_parser_init(&app->parser);

    if (hal->now_ms != NULL) {
        now = hal->now_ms();
    }
    ld_wd_init(&app->wd, now);
    app->last_tick_ms = now;
    app->last_battery_ms = now;
    app->last_odom_ms = now;
    app->last_state_ms = now;

    /* 上电默认断使能：宁可多等 3 帧心跳，也不要「一上电就使能、
     * 等上位机来告诉它停」（README 铁律 3）。 */
    if (hal->motor_enable != NULL) {
        hal->motor_enable(0);
    }
    app->state_dirty = 1;
}

/* ---------------------------------------------------------------- 发送 */

static void send_frame(ld_app_t *app, uint8_t type,
                       const uint8_t *payload, uint8_t plen)
{
    uint8_t buf[LD_FRAME_MAX];
    size_t n;

    if (app->hal->uart_write == NULL) {
        return;
    }
    n = ld_encode(type, payload, plen, buf, sizeof(buf));
    if (n > 0u) {
        (void)app->hal->uart_write(buf, (uint32_t)n);
    }
}

static void send_state(ld_app_t *app)
{
    uint8_t on[1];
    uint8_t fault[2];
    uint16_t f = app->wd.fault;

    on[0] = (uint8_t)(app->wd.estop ? 1u : 0u);
    send_frame(app, LD_T_ESTOP_STATE, on, 1u);

    /* 小端 u16：低字节在前 */
    fault[0] = (uint8_t)(f & 0xFFu);
    fault[1] = (uint8_t)((f >> 8) & 0xFFu);
    send_frame(app, LD_T_FAULT, fault, 2u);
}

static void send_battery(ld_app_t *app)
{
    uint8_t pl[12];
    float pct = 0.0f;
    float volt = 0.0f;
    float amp = 0.0f;

    if (app->hal->battery_read == NULL) {
        return;
    }
    if (app->hal->battery_read(&pct, &volt, &amp) == 0) {
        /* 读失败就这一拍不发。上报 0% 会让 dock_manager 以为电量耗尽、
         * 触发一次没必要的回充，比不发更糟。 */
        return;
    }
    put_f32(&pl[0], pct);
    put_f32(&pl[4], volt);
    put_f32(&pl[8], amp);
    send_frame(app, LD_T_BATTERY, pl, 12u);
}

static void send_odom(ld_app_t *app)
{
    uint8_t pl[20];

    put_f32(&pl[0], app->odom_x);
    put_f32(&pl[4], app->odom_y);
    put_f32(&pl[8], app->odom_yaw);
    put_f32(&pl[12], app->odom_v);
    put_f32(&pl[16], app->odom_w);
    send_frame(app, LD_T_ODOM, pl, 20u);
}

/* ---------------------------------------------------------------- 里程计 */

static void odom_step(ld_app_t *app, uint32_t now)
{
    int32_t el;
    int32_t er;
    uint32_t dt_ms;
    float m_per_tick;
    float dl;
    float dr;
    float d;
    float dyaw;
    float dt_s;

    dt_ms = (uint32_t)(now - app->last_odom_ms);
    app->last_odom_ms = now;

    if (app->hal->encoder_read == NULL) {
        return;
    }
    el = app->hal->encoder_read(0u);
    er = app->hal->encoder_read(1u);

    if (!app->odom_started) {
        /* 第一拍只取基准值，不积分：否则开机瞬间会把编码器的绝对值
         * 当成位移，odom 直接跳出去几十米。 */
        app->last_enc_l = el;
        app->last_enc_r = er;
        app->odom_started = 1u;
        return;
    }

    m_per_tick = LD_TWO_PI * app->cfg.wheel_radius_m / app->cfg.ticks_per_rev;
    /* 有符号差：编码器计数回绕时仍然正确（模运算） */
    dl = (float)(el - app->last_enc_l) * m_per_tick;
    dr = (float)(er - app->last_enc_r) * m_per_tick;
    app->last_enc_l = el;
    app->last_enc_r = er;

    d = 0.5f * (dl + dr);
    dyaw = (dr - dl) / app->cfg.wheel_base_m;
    /* 中点积分：一次走两个 50ms 周期的量时，用起点朝向积分会有可观误差 */
    app->odom_x += d * cosf(app->odom_yaw + 0.5f * dyaw);
    app->odom_y += d * sinf(app->odom_yaw + 0.5f * dyaw);
    app->odom_yaw = wrap_pi(app->odom_yaw + dyaw);

    dt_s = (float)dt_ms / 1000.0f;
    if (dt_s > 1e-4f) {
        app->odom_v = d / dt_s;
        app->odom_w = dyaw / dt_s;
    }
}

/* ---------------------------------------------------------------- 收帧 */

static void handle_frame(ld_app_t *app, const ld_frame_t *f, uint32_t now)
{
    switch (f->type) {
    case LD_T_HEARTBEAT:
        ld_wd_heartbeat(&app->wd, now);
        /* 唯一的喂狗点。放进 while(1) 里喂，主循环卡死时照样喂 —— 等于没有看门狗。 */
        if (app->hal->iwdg_feed != NULL) {
            app->hal->iwdg_feed();
        }
        break;

    case LD_T_CMD_VEL:
        if (f->len >= 8u) {
            app->cmd_linear = clampf(get_f32(&f->payload[0]),
                                     -app->cfg.max_linear_mps,
                                     app->cfg.max_linear_mps);
            app->cmd_angular = clampf(get_f32(&f->payload[4]),
                                      -app->cfg.max_angular_rps,
                                      app->cfg.max_angular_rps);
            app->cmd_frames++;
        }
        break;

    case LD_T_ESTOP: {
        int on;
        if (f->len < 1u) {
            break;
        }
        on = f->payload[0] ? 1 : 0;

        /* 物理按钮按下期间，上位机不能把它解除：按钮是最后一道人工手段，
         * 不能因为上位机「以为已经 clear 过了」就失效。 */
        if (!on && app->button_pressed) {
            break;
        }

        /* **只在变化时**调用。ld_wd_set_estop 会把 hb_count 清零，
         * 而上位机可能周期性重发 ESTOP=0 来保活 —— 每帧都调的话
         * 恢复帧数永远攒不够，车会「心跳正常但永远不动」。 */
        if ((uint8_t)on != app->estop_from_host) {
            app->estop_from_host = (uint8_t)on;
            ld_wd_set_estop(&app->wd, on, 0);
            app->state_dirty = 1u;
            if (on && app->hal->motor_enable != NULL) {
                /* 急停要立刻断使能，不等下一个 tick */
                app->hal->motor_enable(0);
            }
        }
        break;
    }

    case LD_T_LED:
        if (f->len >= 4u) {
            app->led_brightness = clampf(get_f32(&f->payload[0]), 0.0f, 1.0f);
            if (app->hal->led_set != NULL) {
                app->hal->led_set(app->led_brightness);
            }
        }
        break;

    default:
        /* 未知类型直接忽略：协议允许后续扩展，不能因为不认识就把链路断了。 */
        break;
    }
}

void ld_app_rx(ld_app_t *app, const uint8_t *buf, uint32_t len)
{
    ld_frame_t frames[4];
    int n;
    int i;
    uint32_t now;

    if (app == NULL || app->hal == NULL || buf == NULL || len == 0u) {
        return;
    }
    now = (app->hal->now_ms != NULL) ? app->hal->now_ms() : app->last_tick_ms;

    n = ld_parser_push(&app->parser, buf, (size_t)len, frames, 4);
    for (i = 0; i < n; i++) {
        app->frames_ok++;
        handle_frame(app, &frames[i], now);
    }
    app->frames_bad = app->parser.crc_errors + app->parser.len_errors;
}

/* ---------------------------------------------------------------- 输出决策 */

void ld_app_tick(ld_app_t *app, uint32_t now_ms)
{
    int st;
    float l;
    float r;

    if (app == NULL || app->hal == NULL) {
        return;
    }
    app->last_tick_ms = now_ms;

    ld_wd_tick(&app->wd, now_ms);

    /* 三态决策，别自己组合 output_enabled + hold_zero（会死锁，
     * 见 lightdog_watchdog.h 里那段说明）。 */
    st = ld_wd_output_state(&app->wd);
    if (st == LD_WD_OUT_DISABLE) {
        if (app->hal->motor_enable != NULL) {
            app->hal->motor_enable(0);
        }
        /* 失联/急停时把目标清零：恢复后必须从零重新开始，
         * 不能带着失联前那条速度指令冲出去。 */
        app->cmd_linear = 0.0f;
        app->cmd_angular = 0.0f;
    } else if (st == LD_WD_OUT_ZERO) {
        if (app->hal->motor_enable != NULL) {
            app->hal->motor_enable(1);
        }
        if (app->hal->motor_set != NULL) {
            app->hal->motor_set(0.0f, 0.0f);
        }
        ld_wd_zero_applied(&app->wd);
    } else {
        float half = 0.5f * app->cmd_angular * app->cfg.wheel_base_m;
        float cap = app->cfg.max_linear_mps + 0.5f * app->cfg.max_angular_rps
                    * app->cfg.wheel_base_m;
        l = clampf(app->cmd_linear - half, -cap, cap);
        r = clampf(app->cmd_linear + half, -cap, cap);
        if (app->hal->motor_enable != NULL) {
            app->hal->motor_enable(1);
        }
        if (app->hal->motor_set != NULL) {
            app->hal->motor_set(l, r);
        }
    }

    if (app->cfg.odom_period_ms != 0u &&
        (uint32_t)(now_ms - app->last_odom_ms) >= app->cfg.odom_period_ms) {
        odom_step(app, now_ms);
        if (app->odom_started) {
            send_odom(app);
        }
    }

    if (app->cfg.battery_period_ms != 0u &&
        (uint32_t)(now_ms - app->last_battery_ms) >= app->cfg.battery_period_ms) {
        app->last_battery_ms = now_ms;
        send_battery(app);
    }

    if (app->state_dirty ||
        (app->cfg.state_period_ms != 0u &&
         (uint32_t)(now_ms - app->last_state_ms) >= app->cfg.state_period_ms)) {
        app->state_dirty = 0u;
        app->last_state_ms = now_ms;
        send_state(app);
    }
}

void ld_app_on_button(ld_app_t *app, int pressed, uint32_t now_ms)
{
    if (app == NULL || app->hal == NULL) {
        return;
    }
    (void)now_ms;

    app->button_pressed = pressed ? 1u : 0u;

    if (pressed) {
        /* 先硬件断使能，再改状态：中断里不能等通信，也不能指望主循环还活着 */
        if (app->hal->motor_enable != NULL) {
            app->hal->motor_enable(0);
        }
        ld_wd_set_estop(&app->wd, 1, 1);
    } else {
        /* 自锁按钮旋出 -> 解除。ld_wd_set_estop(0) 会把 hb_count 清零，
         * 所以解除之后仍然要 3 帧心跳才恢复出力 —— 防止松开按钮的瞬间
         * 按失联前的旧指令冲出去。 */
        ld_wd_set_estop(&app->wd, 0, 1);
    }
    app->state_dirty = 1u;
}
