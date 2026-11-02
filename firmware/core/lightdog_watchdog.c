/*
 * LightDog 看门狗判定实现（纯 C99）。
 *
 * 时间比较一律用无符号减法：`(uint32_t)(now - last)`。ms 计数器在 49 天后回绕，
 * 用 `now > last + 300` 的写法会在回绕那一刻把「刚收到心跳」判成「失联 49 天」。
 */
#include "lightdog_watchdog.h"

#include "lightdog_proto.h"

void ld_wd_init(ld_wd_t *w, uint32_t now_ms)
{
    if (w == NULL) {
        return;
    }
    w->last_hb_ms = now_ms;
    w->have_hb = 0;
    w->hb_count = 0;
    w->estop = 0;
    w->enabled = 0;
    w->hold_zero = 0;
    w->fault = 0;
}

void ld_wd_heartbeat(ld_wd_t *w, uint32_t now_ms)
{
    if (w == NULL) {
        return;
    }
    w->last_hb_ms = now_ms;
    w->have_hb = 1;
    if (w->hb_count < LD_WD_RECOVER_FRAMES) {
        w->hb_count++;
    }
}

void ld_wd_set_estop(ld_wd_t *w, int on, int from_button)
{
    if (w == NULL) {
        return;
    }
    if (on) {
        w->estop = 1;
        w->enabled = 0;
        w->hold_zero = 0;
        /* 急停要立刻生效，不等下一次 tick 判定 */
        w->hb_count = 0;
        if (from_button) {
            w->fault |= LD_F_ESTOP_BUTTON;
        }
    } else {
        w->estop = 0;
        /* 解除之后必须重新攒够 RECOVER_FRAMES 帧心跳才能再动 */
        w->hb_count = 0;
        if (from_button) {
            w->fault &= (uint16_t)~LD_F_ESTOP_BUTTON;
        }
    }
}

void ld_wd_tick(ld_wd_t *w, uint32_t now_ms)
{
    uint8_t fresh;
    uint8_t was_enabled;

    if (w == NULL) {
        return;
    }
    fresh = (uint8_t)(w->have_hb &&
                      ((uint32_t)(now_ms - w->last_hb_ms) < LD_WD_TIMEOUT_MS));
    if (!fresh) {
        w->enabled = 0;
        w->hb_count = 0;
        w->hold_zero = 0;
        w->fault |= LD_F_LINK_LOST;
        return;
    }
    w->fault &= (uint16_t)~LD_F_LINK_LOST;

    was_enabled = w->enabled;
    w->enabled = (uint8_t)(!w->estop && w->hb_count >= LD_WD_RECOVER_FRAMES);
    if (w->enabled && !was_enabled) {
        w->hold_zero = 1;      /* 恢复后的第一拍必须先出零速 */
    }
    if (!w->enabled) {
        w->hold_zero = 0;
    }
}

int ld_wd_output_enabled(const ld_wd_t *w)
{
    if (w == NULL) {
        return 0;
    }
    return (int)(w->enabled && !w->hold_zero);
}

void ld_wd_zero_applied(ld_wd_t *w)
{
    if (w == NULL) {
        return;
    }
    w->hold_zero = 0;
}
