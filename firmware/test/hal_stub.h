/*
 * host 编译用的假 HAL：把所有外设调用记下来，供 test_app.c 断言。
 *
 * 它不属于固件产物，只在 `make test` / `make app` 的 PC 构建里参与编译。
 */
#ifndef LIGHTDOG_HAL_STUB_H
#define LIGHTDOG_HAL_STUB_H

#include <stdint.h>

#include "lightdog_hal.h"

#define HAL_STUB_TX_MAX 1024

typedef struct {
    uint32_t now;
    int motor_on;
    float left;
    float right;
    float led;
    int32_t enc[2];
    int battery_ok;
    float pct;
    float volt;
    float amp;

    /* 可选能力（F-010）：温度与编码器健康，默认关闭 */
    int temp_ok;          /* 0=没有该传感器/读失败 → 永不置 OVERTEMP */
    float temp_c;
    int enc_err[2];       /* 非 0 = 该侧编码器故障 */

    /* 临界区可控开关（F-003）：enter/exit 是否接了实现 */
    int critical_enabled;

    /* 记录 */
    int enable_calls;
    int disable_calls;
    int iwdg_feeds;
    int enc_reads;
    int critical_enter;
    int critical_exit;
    int critical_depth;   /* 当前嵌套深度，正常应在 0/1 之间 */
    int critical_nested;  /* 曾出现过 >1 的嵌套（app 层刻意不嵌套，应为 0） */
    uint8_t tx[HAL_STUB_TX_MAX];
    uint32_t tx_len;
    uint32_t tx_frames;
} hal_stub_t;

/* 全局唯一实例：函数指针的签名没有 user 参数，所以只能这样接 */
extern hal_stub_t g_hal;

void hal_stub_reset(void);

/* 取填充好的函数表（指向 g_hal 的实现） */
const ld_hal_t *hal_stub_table(void);

#endif /* LIGHTDOG_HAL_STUB_H */
