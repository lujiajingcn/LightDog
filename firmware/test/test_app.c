/*
 * 应用层自检（在 PC 上跑，不需要 STM32）。
 *
 *     make -C firmware test
 *
 * 这一组测的不是「能不能编译」，而是**每一拍给电机什么**：
 *   - 上电、失联、急停、按钮，四种情况下是不是都往「不动」的方向倒
 *   - 恢复供电/恢复心跳之后，会不会带着旧指令冲出去
 *   - 上位机周期重发 ESTOP=0 时，会不会把恢复帧数刷掉（会 -> 永远不动）
 *
 * 这些都是「上车才发现的怪现象」的原产地，而现场没有调试手段。
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "hal_stub.h"
#include "lightdog_app.h"

static int g_checks = 0;
static int g_failed = 0;

#define CHECK(cond, ...)                                                    \
    do {                                                                    \
        g_checks++;                                                         \
        if (!(cond)) {                                                      \
            g_failed++;                                                     \
            printf("FAIL %s:%d  ", __FILE__, __LINE__);                      \
            printf(__VA_ARGS__);                                            \
            printf("\n");                                                   \
        }                                                                   \
    } while (0)

static ld_app_t g_app;
static uint32_t g_t;
static const uint8_t g_empty[1] = { 0 };

static void putf(uint8_t *p, float v)
{
    memcpy(p, &v, sizeof(v));
}

static void putf2(uint8_t *p, float a, float b)
{
    putf(&p[0], a);
    putf(&p[4], b);
}

static void boot(void)
{
    hal_stub_reset();
    ld_app_init(&g_app, hal_stub_table(), NULL);
    g_t = 0;
}

/* 喂一段原始字节进解析器 */
static void raw(const uint8_t *buf, uint32_t len)
{
    ld_app_rx(&g_app, buf, len);
}

static void feed(uint8_t type, const uint8_t *pl, uint8_t len)
{
    uint8_t buf[LD_FRAME_MAX];
    size_t n = ld_encode(type, pl, len, buf, sizeof(buf));
    CHECK(n > 0, "ld_encode 失败 type=0x%02X len=%u", type, (unsigned)len);
    raw(buf, (uint32_t)n);
}

static void hb(void)
{
    feed(LD_T_HEARTBEAT, g_empty, 0);
}

static void cmd(float lin, float ang)
{
    uint8_t pl[8];
    putf2(pl, lin, ang);
    feed(LD_T_CMD_VEL, pl, 8);
}

static void estop(uint8_t on)
{
    feed(LD_T_ESTOP, &on, 1);
}

static void led(float b)
{
    uint8_t pl[4];
    putf(pl, b);
    feed(LD_T_LED, pl, 4);
}

/* 推进时间并跑一拍 tick。
 * 必须同时更新时间源 g_hal.now：rx 里读 hal->now_ms() 给心跳打时间戳，
 * 如果只有 tick 用 g_t、rx 还在读旧值，心跳会被判成「刚收到就已经过期」。 */
static void tick_by(uint32_t ms)
{
    g_t += ms;
    g_hal.now = g_t;
    ld_app_tick(&g_app, g_t);
}

/* 收一帧心跳 + 跑一拍 tick，间隔 10ms */
static void hb_tick(void)
{
    hb();
    tick_by(10);
}

/* 攒够恢复帧数并跑到「可以出力」。返回时 hold_zero 已被 tick 处理掉。 */
static void warmup(void)
{
    hb_tick();
    hb_tick();
    hb_tick();
}

/*
 * 同上，但每一拍前重发一次 CMD_VEL。
 *
 * 为什么必须这样：攒心跳期间是 DISABLE 状态，而 DISABLE 会把目标速度清零
 * （失联/急停后不能带着旧指令冲出去）。真机上上位机是 10Hz 持续发的，
 * 清掉下一帧又填上；测试里只发一次就会被清成 0，那不是实现的问题。
 */
static void warmup_cmd(float lin, float ang)
{
    int i;

    for (i = 0; i < 3; i++) {
        hb();
        cmd(lin, ang);
        tick_by(10);
    }
}

static int tx_has(uint8_t type)
{
    uint32_t i = 0;

    while (i + 6u <= g_hal.tx_len) {
        if (g_hal.tx[i] == LD_SYNC0 && g_hal.tx[i + 1u] == LD_SYNC1) {
            uint8_t len = g_hal.tx[i + 2u];
            if (g_hal.tx[i + 3u] == type) {
                return 1;
            }
            i += 6u + (uint32_t)len;
        } else {
            i++;
        }
    }
    return 0;
}

/* ---------------------------------------------------------------- 用例 */

static void test_power_on_is_disabled(void)
{
    boot();
    CHECK(g_hal.motor_on == 0, "上电必须断使能");
    CHECK(g_hal.disable_calls >= 1, "init 必须真的拉低一次 EN，而不只是变量为 0");
}

static void test_no_heartbeat_never_enables(void)
{
    int i;

    boot();
    for (i = 0; i < 200; i++) {
        tick_by(10);
    }
    CHECK(g_hal.motor_on == 0, "2 秒没有心跳却使能了");
    CHECK((g_app.wd.fault & LD_F_LINK_LOST) != 0, "失联必须置 LINK_LOST");
}

static void test_three_heartbeats_then_zero_first(void)
{
    boot();
    /* 每拍前重发：攒心跳期间目标会被清零，真机上是 10Hz 持续发的 */
    hb();
    cmd(0.3f, 0.0f);
    tick_by(10);
    CHECK(g_hal.motor_on == 0, "第 1 帧心跳后不该使能");

    hb();
    cmd(0.3f, 0.0f);
    tick_by(10);
    CHECK(g_hal.motor_on == 0, "第 2 帧心跳后不该使能");

    hb();
    cmd(0.3f, 0.0f);
    tick_by(10);
    CHECK(g_hal.motor_on == 1, "第 3 帧心跳后应该使能");
    /* 恢复后的第一拍必须是零速：不能带着失联前的旧指令冲出去 */
    CHECK(fabsf(g_hal.left) < 1e-6f && fabsf(g_hal.right) < 1e-6f,
          "恢复第一拍必须是零速，实际 L=%.3f R=%.3f", g_hal.left, g_hal.right);

    tick_by(10);
    CHECK(fabsf(g_hal.left - 0.3f) < 1e-4f,
          "第二拍应恢复跟目标 0.3，实际 L=%.3f", g_hal.left);
}

static void test_link_loss_stops_and_clears_target(void)
{
    boot();
    warmup_cmd(0.3f, 0.0f);
    tick_by(10);
    CHECK(fabsf(g_hal.left - 0.3f) < 1e-4f,
          "热身之后应该能动，实际 L=%.3f", g_hal.left);

    /* 心跳断了 300ms 以上 */
    tick_by(350);
    CHECK(g_hal.motor_on == 0, "失联必须断使能");
    CHECK(fabsf(g_app.cmd_linear) < 1e-6f, "失联必须把目标速度清零");

    /* 心跳回来：要重新攒 3 帧，且恢复后第一拍仍是零速 */
    hb_tick();
    CHECK(g_hal.motor_on == 0, "恢复时不能立刻使能");
    hb_tick();
    hb_tick();
    CHECK(g_hal.motor_on == 1, "3 帧后应使能");
    CHECK(fabsf(g_hal.left) < 1e-6f,
          "目标已被清零，恢复后应保持零速，实际 L=%.3f", g_hal.left);
}

static void test_estop_frame_cuts_enable_immediately(void)
{
    boot();
    warmup_cmd(0.3f, 0.0f);
    tick_by(10);
    CHECK(g_hal.motor_on == 1, "前提：应该能动");

    estop(1);
    /* 没有 tick —— 急停必须在下一拍之前就生效 */
    CHECK(g_hal.motor_on == 0, "急停帧必须立刻断使能，不能等下一个 tick");
}

static void test_estop_clear_needs_fresh_heartbeats(void)
{
    boot();
    cmd(0.3f, 0.0f);
    warmup();
    estop(1);

    estop(0);
    tick_by(10);
    CHECK(g_hal.motor_on == 0, "解除急停后不能立刻恢复出力");

    hb_tick();
    hb_tick();
    hb_tick();
    CHECK(g_hal.motor_on == 1, "解除后攒够 3 帧应恢复");
}

static void test_periodic_estop_zero_does_not_block_enable(void)
{
    /* 真实的 mcu_bridge 会周期性重发 ESTOP（锁存语义），而 ld_wd_set_estop
     * 会把恢复帧数清零。如果每帧都调，车就永远不动 —— 这里钉住它。 */
    boot();
    cmd(0.2f, 0.0f);
    hb();
    estop(0);
    tick_by(10);
    hb();
    estop(0);
    tick_by(10);
    hb();
    estop(0);
    tick_by(10);

    CHECK(g_hal.motor_on == 1,
          "周期性重发 ESTOP=0 不应阻止恢复（hb_count 被反复清零了？）");
}

static void test_button_cuts_enable_and_sets_fault(void)
{
    boot();
    warmup_cmd(0.3f, 0.0f);
    tick_by(10);
    CHECK(g_hal.motor_on == 1, "前提：应该能动");

    ld_app_on_button(&g_app, 1, g_t);
    CHECK(g_hal.motor_on == 0, "按下按钮必须立刻断使能");
    CHECK((g_app.wd.fault & LD_F_ESTOP_BUTTON) != 0,
          "按钮急停必须置 F_ESTOP_BUTTON 供上报");
    CHECK(g_app.wd.estop == 1, "按钮必须进入闭锁");
}

static void test_host_cannot_clear_while_button_held(void)
{
    boot();
    warmup();
    ld_app_on_button(&g_app, 1, g_t);

    estop(0);
    CHECK(g_app.wd.estop == 1,
          "按钮还按着的时候上位机 clear 必须无效（人工手段优先）");
}

static void test_button_release_unlocks_but_needs_heartbeats(void)
{
    boot();
    cmd(0.3f, 0.0f);
    warmup();
    ld_app_on_button(&g_app, 1, g_t);

    ld_app_on_button(&g_app, 0, g_t);
    CHECK(g_app.wd.estop == 0, "自锁按钮旋出应解除闭锁");
    tick_by(10);
    CHECK(g_hal.motor_on == 0, "解除后不能立刻出力");

    hb_tick();
    hb_tick();
    hb_tick();
    CHECK(g_hal.motor_on == 1, "解除后攒够 3 帧应恢复");
}

static void test_cmd_vel_is_clamped(void)
{
    boot();
    cmd(99.0f, -99.0f);
    CHECK(fabsf(g_app.cmd_linear - g_app.cfg.max_linear_mps) < 1e-6f,
          "线速度必须限到 %.2f，实际 %.3f",
          g_app.cfg.max_linear_mps, g_app.cmd_linear);
    CHECK(fabsf(g_app.cmd_angular + g_app.cfg.max_angular_rps) < 1e-6f,
          "角速度必须限到 -%.2f，实际 %.3f",
          g_app.cfg.max_angular_rps, g_app.cmd_angular);
}

static void test_led_brightness_is_clamped(void)
{
    boot();
    led(1.7f);
    CHECK(fabsf(g_app.led_brightness - 1.0f) < 1e-6f, "亮度必须钳到 1.0");
    CHECK(fabsf(g_hal.led - 1.0f) < 1e-6f, "HAL 收到的亮度也要钳过");
    led(-0.5f);
    CHECK(fabsf(g_app.led_brightness) < 1e-6f, "亮度必须钳到 0");
}

static void test_diff_drive_signs(void)
{
    /* 正角速度 = 左转 = 右轮快、左轮慢（ROS 约定 z 逆时针） */
    boot();
    warmup_cmd(0.0f, 0.5f);
    tick_by(10);
    CHECK(g_hal.right > g_hal.left,
          "左转时右轮应更快，实际 L=%.3f R=%.3f", g_hal.left, g_hal.right);
    CHECK(g_hal.left < 0.0f, "原地左转时左轮应反转");
}

static void test_odom_straight_line(void)
{
    float expect;

    boot();
    warmup();
    tick_by(100);              /* 第一次 odom_step 只取基准 */
    g_hal.enc[0] = 1560;
    g_hal.enc[1] = 1560;
    tick_by(1000);

    expect = 6.2831853f * g_app.cfg.wheel_radius_m;   /* 一圈的周长 */
    CHECK(fabsf(g_app.odom_x - expect) < 1e-3f,
          "直行一圈后 x 应为 %.4f，实际 %.4f", expect, g_app.odom_x);
    CHECK(fabsf(g_app.odom_y) < 1e-4f, "直行不该有横向位移");
    CHECK(fabsf(g_app.odom_yaw) < 1e-4f, "直行不该有偏航");
    CHECK(fabsf(g_app.odom_v - expect) < 1e-2f,
          "速度应为 %.3f m/s，实际 %.3f", expect, g_app.odom_v);
}

static void test_odom_turn_in_place(void)
{
    float expect_yaw;

    boot();
    warmup();
    tick_by(100);
    g_hal.enc[0] = -1560;
    g_hal.enc[1] = 1560;
    tick_by(1000);

    expect_yaw = 2.0f * 6.2831853f * g_app.cfg.wheel_radius_m
                 / g_app.cfg.wheel_base_m;
    CHECK(fabsf(g_app.odom_yaw - expect_yaw) < 1e-3f,
          "原地转一圈的 yaw 应为 %.4f，实际 %.4f", expect_yaw, g_app.odom_yaw);
    CHECK(fabsf(g_app.odom_x) < 1e-4f, "原地转不该有前向位移");
}

static void test_battery_failure_sends_nothing(void)
{
    boot();
    g_hal.battery_ok = 0;
    g_hal.tx_len = 0;
    tick_by(1100);
    CHECK(!tx_has(LD_T_BATTERY),
          "电池读失败时不能上报（报 0%% 会误触发回充）");

    g_hal.battery_ok = 1;
    tick_by(1100);
    CHECK(tx_has(LD_T_BATTERY), "读成功时应上报 BATTERY");
}

static void test_odom_and_battery_periods(void)
{
    boot();
    warmup();
    g_hal.tx_len = 0;
    g_hal.tx_frames = 0;
    tick_by(60);                       /* 超过 odom 周期 50ms */
    CHECK(tx_has(LD_T_ODOM), "到周期应上报 ODOM");
    CHECK(!tx_has(LD_T_BATTERY), "电池 1Hz，不该在 60ms 内上报");
}

static void test_state_is_resent_periodically(void)
{
    boot();
    g_hal.tx_len = 0;
    tick_by(10);
    CHECK(tx_has(LD_T_ESTOP_STATE), "启动后应立即上报一次急停状态");
    /* 串口没有 DDS 的 TRANSIENT_LOCAL，丢一帧就是丢了，所以要周期重发 */
    CHECK(g_app.cfg.state_period_ms > 0u, "重发周期必须非 0");
}

static void test_unknown_frame_type_is_ignored(void)
{
    uint8_t pl[2] = { 0x11, 0x22 };
    uint32_t before = g_app.frames_ok;

    boot();
    feed(0x7Fu, pl, 2);
    CHECK(g_app.frames_ok == before + 1u, "未知类型也应算作收到的合法帧");
    CHECK(g_app.wd.estop == 0, "未知类型不能改变急停状态");
}

static void test_bad_crc_does_not_feed_watchdog(void)
{
    uint8_t buf[LD_FRAME_MAX];
    size_t n;
    int feeds;

    boot();
    n = ld_encode(LD_T_HEARTBEAT, g_empty, 0, buf, sizeof(buf));
    CHECK(n > 0, "编码失败");
    buf[n - 1u] ^= 0xFFu;              /* 破坏 CRC */

    feeds = g_hal.iwdg_feeds;
    raw(buf, (uint32_t)n);
    CHECK(g_hal.iwdg_feeds == feeds,
          "CRC 错的帧不能喂 IWDG（否则链路坏了看门狗还以为一切正常）");
    CHECK(g_app.frames_bad >= 1u, "CRC 错要计入统计");
}

static void test_heartbeat_is_the_only_iwdg_feed(void)
{
    int feeds;

    boot();
    feeds = g_hal.iwdg_feeds;
    tick_by(1000);
    CHECK(g_hal.iwdg_feeds == feeds, "tick 里不能喂 IWDG（主循环卡死等于没看门狗）");

    hb();
    CHECK(g_hal.iwdg_feeds == feeds + 1, "每帧合法心跳喂一次");
}

int main(void)
{
    test_power_on_is_disabled();
    test_no_heartbeat_never_enables();
    test_three_heartbeats_then_zero_first();
    test_link_loss_stops_and_clears_target();
    test_estop_frame_cuts_enable_immediately();
    test_estop_clear_needs_fresh_heartbeats();
    test_periodic_estop_zero_does_not_block_enable();
    test_button_cuts_enable_and_sets_fault();
    test_host_cannot_clear_while_button_held();
    test_button_release_unlocks_but_needs_heartbeats();
    test_cmd_vel_is_clamped();
    test_led_brightness_is_clamped();
    test_diff_drive_signs();
    test_odom_straight_line();
    test_odom_turn_in_place();
    test_battery_failure_sends_nothing();
    test_odom_and_battery_periods();
    test_state_is_resent_periodically();
    test_unknown_frame_type_is_ignored();
    test_bad_crc_does_not_feed_watchdog();
    test_heartbeat_is_the_only_iwdg_feed();

    printf("%d 项检查，%d 项失败\n", g_checks, g_failed);
    return g_failed ? 1 : 0;
}
