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

/* 临界区包装（F-003）：UART RX ISR、急停按钮 ISR、1kHz tick 三个上下文都会
 * 读-改-写共享的看门狗状态（estop 与 fault 掩码）。Cortex-M 上同级/高优先级
 * 中断能在 RMW 中间打断，导致漏置/漏清故障位、或急停与使能判定互相覆盖。这里
 * 用 HAL 提供的开关中断把这**几条**语句保护起来。HAL 没实现时退化为本上下文
 * 顺序执行（PC 单线程自检下等价，且各用例仍是确定性的）。
 * 铁律：临界区里只放寄存器级快操作，绝不做 ADC / 三角函数 / uart 写这类慢活。 */
static void app_enter_critical(ld_app_t *app)
{
    if (app->hal->enter_critical != NULL) {
        app->hal->enter_critical();
    }
}

static void app_exit_critical(ld_app_t *app)
{
    if (app->hal->exit_critical != NULL) {
        app->hal->exit_critical();
    }
}

/* 会触发断电保护的电气/热故障。ENCODER 只置位上报，不在此列。 */
#define LD_F_SAFETY_CUTOFF \
    ((uint16_t)(LD_F_OVERCURRENT | LD_F_UNDERVOLTAGE | LD_F_OVERTEMP))

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
    /* 安全阈值占位默认：务必按实机电机/电池 datasheet 再调，这里只求「接上但
     * 不误触发」。max_current_a 取堵转裕度之外，min_voltage_v 取低于正常工作
     * 下限的硬截止，max_temp_c 取功率级/电机的常见热截止。 */
    cfg->safety_period_ms = 100u;
    cfg->max_current_a = 8.0f;
    cfg->min_voltage_v = 9.0f;
    cfg->max_temp_c = 85.0f;
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
    app->last_safety_ms = now;

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
    uint16_t f;
    uint8_t estop;

    /* 临界区里只做一次一致性快照；串口写在临界区外（慢，绝不能关着中断做）。 */
    app_enter_critical(app);
    estop = app->wd.estop;
    f = app->wd.fault;
    app_exit_critical(app);

    on[0] = (uint8_t)(estop ? 1u : 0u);
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

/* ---------------------------------------------------------------- 安全故障评估 */

/*
 * F-010：把协议里承诺的「过流 / 欠压 / 过温保护关 PWM」真正接到故障位与断电上。
 * 之前 LD_F_OVERCURRENT/UNDERVOLTAGE/OVERTEMP/ENCODER 定义了却没有任何地方 set，
 * 上位机永远只能看到 LINK_LOST 和按钮位，电气/热失控完全不上报也不处理。
 *
 * 只在 tick 上下文调用（它是这几个电气故障位的唯一写者；estop/按钮位在
 * sync_estop_state 的临界区里写，二者用同一把临界区互斥）。慢操作（ADC 读）
 * 全部留在临界区外，只有最后写 wd.fault / safety_lock 的几条语句进临界区。
 */
static void update_safety_faults(ld_app_t *app, uint32_t now)
{
    float pct = 0.0f;
    float volt = 0.0f;
    float amp = 0.0f;
    float temp = 0.0f;
    int have_batt = 0;
    int have_temp = 0;
    int enc_err = 0;
    uint16_t trip;
    uint16_t mask;
    uint16_t old;

    if (app->cfg.safety_period_ms == 0u) {
        return;
    }
    if ((uint32_t)(now - app->last_safety_ms) < app->cfg.safety_period_ms) {
        return;
    }
    app->last_safety_ms = now;

    /* 电池：只在要做过流/欠压判断时读；读失败则本轮不据此改变任何判定 */
    if (app->hal->battery_read != NULL &&
        (app->cfg.max_current_a > 0.0f || app->cfg.min_voltage_v > 0.0f)) {
        if (app->hal->battery_read(&pct, &volt, &amp) != 0) {
            have_batt = 1;
        }
    }
    /* 温度：没有该传感器就永不置 OVERTEMP（不能因为没接就误刹车） */
    if (app->hal->temp_read != NULL && app->cfg.max_temp_c > 0.0f) {
        if (app->hal->temp_read(&temp) != 0) {
            have_temp = 1;
        }
    }
    /* 编码器通道健康 */
    if (app->hal->encoder_error != NULL) {
        if (app->hal->encoder_error(0u) != 0 || app->hal->encoder_error(1u) != 0) {
            enc_err = 1;
        }
    }

    /* 过流：命中阈值置位，降到阈值 70% 以下才解除（防在临界点反复抖） */
    if (have_batt && app->cfg.max_current_a > 0.0f) {
        if (!app->oc_tripped) {
            if (amp > app->cfg.max_current_a) {
                app->oc_tripped = 1u;
            }
        } else if (amp < app->cfg.max_current_a * 0.7f) {
            app->oc_tripped = 0u;
        }
    }
    /* 欠压：只采信合理区间的电压（<0.5V 多半是毛刺或没接，不据此断电） */
    if (have_batt && app->cfg.min_voltage_v > 0.0f && volt > 0.5f) {
        if (!app->uv_tripped) {
            if (volt < app->cfg.min_voltage_v) {
                app->uv_tripped = 1u;
            }
        } else if (volt > app->cfg.min_voltage_v + 0.5f) {
            app->uv_tripped = 0u;
        }
    }
    /* 过温：高于上限置位，回落 5℃ 以上才解除 */
    if (have_temp && app->cfg.max_temp_c > 0.0f) {
        if (!app->ot_tripped) {
            if (temp > app->cfg.max_temp_c) {
                app->ot_tripped = 1u;
            }
        } else if (temp < app->cfg.max_temp_c - 5.0f) {
            app->ot_tripped = 0u;
        }
    }

    trip = 0u;
    if (app->oc_tripped) {
        trip |= LD_F_OVERCURRENT;
    }
    if (app->uv_tripped) {
        trip |= LD_F_UNDERVOLTAGE;
    }
    if (app->ot_tripped) {
        trip |= LD_F_OVERTEMP;
    }
    mask = (uint16_t)(LD_F_OVERCURRENT | LD_F_UNDERVOLTAGE | LD_F_OVERTEMP);

    /* 只有下面这几条进临界区 */
    app_enter_critical(app);
    old = app->wd.fault;
    ld_wd_set_fault(&app->wd, trip, 1);
    ld_wd_set_fault(&app->wd, (uint16_t)(mask & (uint16_t)~trip), 0);
    if (app->hal->encoder_error != NULL) {
        ld_wd_set_fault(&app->wd, LD_F_ENCODER, enc_err);
    }
    app->safety_lock = (uint8_t)((app->wd.fault & LD_F_SAFETY_CUTOFF) != 0u);
    if (app->wd.fault != old) {
        app->state_dirty = 1u;
    }
    app_exit_critical(app);
}

/* ---------------------------------------------------------------- 收帧 */

/* 把两个急停来源合并成看门狗的实际 estop。
 *
 * estop 位 = 上位机锁存 OR 物理按钮按下。这样任何一方单独解除都不会误开另一方：
 * 原实现里按钮松开会无条件把 estop 清 0，导致「碰一下再松开物理按钮」就把上位机
 * 的锁存急停破掉；而 handle_frame 只在「变化」时重设 estop，宿主后续重发
 * ESTOP=1 又因为 estop_from_host 仍是 1 被判无变化，永远补不回来 —— 表现为
 * 「急停状态下电机照样转」，是最危险的一类失效。
 *
 * 只在合并结果真的变化时才调 ld_wd_set_estop：它会清 hb_count，而宿主可能周期性
 * 重发 ESTOP=0 保活，每帧都调会让恢复心跳永远攒不够（车「心跳正常但永远不动」）。
 * 按钮故障位则独立反映「按钮此刻是否按下」，与 estop 由谁锁存无关。 */
static void sync_estop_state(ld_app_t *app)
{
    uint8_t want = (uint8_t)(app->estop_from_host || app->button_pressed);

    app_enter_critical(app);
    if (want != app->wd.estop) {
        ld_wd_set_estop(&app->wd, want ? 1 : 0, 0);
        app->state_dirty = 1u;
        if (want && app->hal->motor_enable != NULL) {
            /* 急停要立刻断使能，不等下一个 tick */
            app->hal->motor_enable(0);
        }
    }
    ld_wd_set_button_fault(&app->wd, app->button_pressed ? 1 : 0);
    app_exit_critical(app);
}

static void handle_frame(ld_app_t *app, const ld_frame_t *f, uint32_t now)
{
    switch (f->type) {
    case LD_T_HEARTBEAT:
        app_enter_critical(app);
        ld_wd_heartbeat(&app->wd, now);
        app_exit_critical(app);
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

        app->estop_from_host = (uint8_t)on;
        /* 合并两个来源后再落到看门狗，避免「按钮松开把宿主锁存一起清掉」。 */
        sync_estop_state(app);
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

    /* F-010：先评估电气/热故障。慢的 ADC 读全在临界区外，只有末尾写 wd.fault /
     * safety_lock 的几条进临界区。它决定 safety_lock，下面的输出决策据此强制断使能。 */
    update_safety_faults(app, now_ms);

    /* F-003：ld_wd_tick 会对 wd.fault / estop / enabled 做读-改-写，而按钮 EXTI ISR
     * （sync_estop_state）和 UART RX ISR（心跳 ld_wd_heartbeat）也在同一批字段上写。
     * 把「推进看门狗 + 三态决策 + 落到电机寄存器」整段关进一个临界区，避免 ISR 在
     * tick 判定「可以使能」与实际 motor_enable(1) 之间插入并断使能、随后又被 tick
     * 翻回来（或漏置/漏清故障位）。临界区里全是寄存器级快操作，没有任何慢活；真正
     * 耗时的 send_* 帧（uart_write）都留在这段之外。 */
    app_enter_critical(app);

    if (app->safety_lock) {
        /* 过流/欠压/过温闭锁：作废使能判定并持续清零恢复计数。故障没解除期间心跳
         * 也攒不起来，杜绝阈值临界抖动时电机一抽一抽。 */
        ld_wd_invalidate(&app->wd);
    }

    ld_wd_tick(&app->wd, now_ms);

    /* 三态决策，别自己组合 output_enabled + hold_zero（会死锁，
     * 见 lightdog_watchdog.h 里那段说明）。 */
    st = ld_wd_output_state(&app->wd);
    if (st == LD_WD_OUT_DISABLE) {
        if (app->hal->motor_enable != NULL) {
            app->hal->motor_enable(0);
        }
        /* 失联/急停/安全闭锁时把目标清零：恢复后必须从零重新开始，
         * 不能带着断使能前那条速度指令冲出去。 */
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

    app_exit_critical(app);

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
    }
    /* 按钮松开只解除「按钮这一路」。若上位机仍锁存急停，estop 必须保持为 1，
     * 否则碰一下按钮就把宿主的急停破了，且宿主重发 ESTOP=1 也补不回来。
     * 解除后仍需重新攒 3 帧心跳才恢复出力 —— 防止松开瞬间带着旧指令冲出去。 */
    sync_estop_state(app);
    app->state_dirty = 1u;
}
