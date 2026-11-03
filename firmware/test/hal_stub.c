/*
 * host 编译用的假 HAL 实现。
 */
#include "hal_stub.h"

#include <string.h>

hal_stub_t g_hal;

static uint32_t stub_now_ms(void)
{
    return g_hal.now;
}

static void stub_motor_enable(int on)
{
    g_hal.motor_on = on ? 1 : 0;
    if (on) {
        g_hal.enable_calls++;
    } else {
        g_hal.disable_calls++;
    }
}

static void stub_motor_set(float left, float right)
{
    g_hal.left = left;
    g_hal.right = right;
}

static void stub_led_set(float b)
{
    g_hal.led = b;
}

static int stub_uart_write(const uint8_t *buf, uint32_t len)
{
    if (len > (HAL_STUB_TX_MAX - g_hal.tx_len)) {
        return 0;
    }
    memcpy(&g_hal.tx[g_hal.tx_len], buf, len);
    g_hal.tx_len += len;
    g_hal.tx_frames++;
    return (int)len;
}

static int stub_battery_read(float *pct, float *volt, float *amp)
{
    if (!g_hal.battery_ok) {
        return 0;
    }
    if (pct != NULL) {
        *pct = g_hal.pct;
    }
    if (volt != NULL) {
        *volt = g_hal.volt;
    }
    if (amp != NULL) {
        *amp = g_hal.amp;
    }
    return 1;
}

static int32_t stub_encoder_read(uint8_t side)
{
    g_hal.enc_reads++;
    return g_hal.enc[side ? 1 : 0];
}

static void stub_iwdg_feed(void)
{
    g_hal.iwdg_feeds++;
}

static const ld_hal_t g_table = {
    stub_now_ms,
    stub_motor_enable,
    stub_motor_set,
    stub_led_set,
    stub_uart_write,
    stub_battery_read,
    stub_encoder_read,
    stub_iwdg_feed
};

void hal_stub_reset(void)
{
    memset(&g_hal, 0, sizeof(g_hal));
    /* 默认：电池读成功、电压正常，避免每条用例都要先配一遍 */
    g_hal.battery_ok = 1;
    g_hal.pct = 80.0f;
    g_hal.volt = 12.4f;
    g_hal.amp = 0.5f;
    /* 上电默认断使能（与真实硬件下拉一致） */
    g_hal.motor_on = 0;
}

const ld_hal_t *hal_stub_table(void)
{
    return &g_table;
}
