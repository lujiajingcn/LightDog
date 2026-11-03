/*
 * LightDog 看门狗判定（纯逻辑，无 HAL）—— 整条安全链的最后一环。
 *
 * 它回答的是一个是非题：**现在允不允许让电机出力？**
 *
 * 规则（docs/firmware_protocol.md §2.1）：
 *   - 300ms 没收到心跳          -> 立即失能
 *   - 急停（软件或物理按钮）    -> 立即失能，且清除「已恢复」进度
 *   - 重新使能要连续 3 帧心跳   -> 防止单帧抖动把车「救活」
 *   - 恢复后第一拍必须先出零速  -> 丢掉失联前残留的旧指令
 *
 * 为什么这条逻辑要单独成文件：它是**唯一**在上位机 `kill -9` 之后还能刹住车的
 * 东西，所以必须能在 PC 上把 300ms 边界、恢复帧数、急停优先级全部跑一遍。
 * 写在 HAL 的 while(1) 里就只能靠「上车试一下」验证，而这类 bug 上车试一次
 * 的代价可能是撞一次墙。
 */
#ifndef LIGHTDOG_WATCHDOG_H
#define LIGHTDOG_WATCHDOG_H

#include <stdint.h>

#define LD_WD_TIMEOUT_MS      300u
#define LD_WD_RECOVER_FRAMES  3u

typedef struct {
    uint32_t last_hb_ms;
    uint8_t  have_hb;      /* 开机以来收到过心跳 */
    uint8_t  hb_count;     /* 失联后连续收到的心跳数，饱和在 RECOVER_FRAMES */
    uint8_t  estop;        /* 闭锁：只能显式 set_estop(0) 解 */
    uint8_t  enabled;      /* 心跳新鲜 && 无急停 && 恢复帧数够 */
    uint8_t  hold_zero;    /* 刚恢复使能：本拍必须先输出零速 */
    uint16_t fault;        /* 位掩码，见 lightdog_proto.h */
} ld_wd_t;

void ld_wd_init(ld_wd_t *w, uint32_t now_ms);

/* 收到一帧合法心跳时调用（解析器确认 CRC 之后再调，别在 ISR 里凭字节数就调）。 */
void ld_wd_heartbeat(ld_wd_t *w, uint32_t now_ms);

/* 急停开关。from_button 为真时同时置位 F_ESTOP_BUTTON 故障位。
 * 仅用于**上报**：物理按钮还必须在硬件上直接断开驱动使能。 */
void ld_wd_set_estop(ld_wd_t *w, int on, int from_button);

/* 主循环里每轮调一次（建议 100Hz 以上，否则 300ms 判定会有 10ms 级抖动） */
void ld_wd_tick(ld_wd_t *w, uint32_t now_ms);

/* 允许输出 PWM 吗？（hold_zero 期间返回 0：调用方必须先发一拍零速） */
int ld_wd_output_enabled(const ld_wd_t *w);

/*
 * 这一拍该给电机什么 —— 三态，调用方 switch 即可。
 *
 * **不要自己组合 `output_enabled` 和 `hold_zero`**：
 *     if (!ld_wd_output_enabled(&w)) motor_disable();
 *     else if (w.hold_zero) { motor_set(0,0); ld_wd_zero_applied(&w); }
 * 看起来对，实际会死锁 —— hold_zero 期间 output_enabled 返回 0，于是走进
 * 断使能分支，zero_applied 永远不被调用，hold_zero 永远不退，车永远不动。
 * （实测：enabled=1 hold_zero=1 motor_enable=0，心跳计数已经到 3。）
 *
 * hold_zero 的正确含义是「**可以使能，但这一拍必须给零速**」，
 * 它和「不许使能」是两件事，`output_enabled` 那个布尔量表达不了。
 */
#define LD_WD_OUT_DISABLE 0   /* 断使能（EN/STBY 拉低），不是输出 0 占空比 */
#define LD_WD_OUT_ZERO    1   /* 使能，但本拍必须输出零速，下发后调 zero_applied */
#define LD_WD_OUT_RUN     2   /* 使能，按目标速度输出 */

int ld_wd_output_state(const ld_wd_t *w);

/* 已经输出过那一拍零速后调用，解除 hold_zero */
void ld_wd_zero_applied(ld_wd_t *w);

#endif /* LIGHTDOG_WATCHDOG_H */
