/*
 * LightDog HAL 的 STM32F4 参考实现。
 *
 * 这不是「拷进去就能跑」的固件：引脚、定时器、ADC 通道都按你的原理图改，
 * 电机/编码器的换算要按实际减速比和轮径重新算。它给出的是**每个函数的
 * 职责边界和几个必须避开的坑**（见每个函数上方的注释）。
 *
 * 不参与 firmware/Makefile 的 host 构建：那个构建只验纯逻辑。
 * 真正编译它要用 CubeMX 生成的工程（USE_HAL_DRIVER + 芯片宏）。
 *
 * 接线与参数说明见同目录的 README.md。
 */
#if defined(USE_HAL_DRIVER) && (defined(STM32F405xx) || defined(STM32F407xx))

#include "lightdog_hal.h"
#include "lightdog_proto.h"

#include <string.h>

#include "main.h"          /* CubeMX 生成：huart2 / htim1 / htim3 / hiwdg ... */
#include "stm32f4xx_hal.h"

/* ---- 按你的原理图改这几项 ---- */
#define LD_TX_QUEUE_FRAMES   4u
#define LD_MOTOR_MAX_ACCEL   1.5f      /* m/s^2 斜坡，防止打滑丢里程 */
#define LD_PWM_PERIOD        8400u     /* TIM1 ARR：20kHz @ 168MHz */
/* 单轮线速度上限（对应满占空比）。必须 >= ld_app_cfg_t 里
 * max_linear + max_angular*wheel_base/2，否则「限速」只是纸上写的。 */
#define LD_MAX_WHEEL_MPS     0.75f

extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef  htim1;       /* 4 路互补 PWM */
extern TIM_HandleTypeDef  htim3;       /* 左编码器 */
extern TIM_HandleTypeDef  htim4;       /* 右编码器 */
extern ADC_HandleTypeDef  hadc1;
extern IWDG_HandleTypeDef hiwdg;

/* ---------------------------------------------------------------- 时间 */

static uint32_t hal_now_ms(void)
{
    /* HAL_GetTick 是 1ms 递增的 uint32，49 天回绕；上层全部用无符号减法，
     * 所以这里不需要额外处理。 */
    return HAL_GetTick();
}

/* ---------------------------------------------------------------- 电机 */

static int motor_on_state = 0;
static float wheel_target[2] = { 0.0f, 0.0f };
static float wheel_now[2] = { 0.0f, 0.0f };
static uint32_t ramp_ms = 0;

static void hal_motor_enable(int on)
{
    motor_on_state = on ? 1 : 0;

    if (on) {
        /* **先给零占空比再拉 EN**：反过来会有一次「EN 已高、占空比还是
         * 上一次的残留值」的窗口，表现为上电瞬间轮子抽一下。 */
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
        HAL_GPIO_WritePin(MOTOR_EN_GPIO_Port, MOTOR_EN_Pin, GPIO_PIN_SET);
    } else {
        /* 断使能，不是输出 0 占空比（README 铁律 4）。
         * 驱动过热复位、线松、MCU 复位 —— 只有断使能三种情况都停得住。 */
        HAL_GPIO_WritePin(MOTOR_EN_GPIO_Port, MOTOR_EN_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
        wheel_now[0] = 0.0f;
        wheel_now[1] = 0.0f;
    }
}

/*
 * 把 m/s 写成 PWM ± 方向脚。
 *
 * 这里做斜坡限加速：从 0 跳到 0.5m/s 会打滑，而打滑之后编码器读数与实际
 * 位移脱节，里程计立刻漂 —— 表现为「导航越走越偏」，很难归因到固件。
 */
static void hal_motor_set(float left_mps, float right_mps)
{
    uint32_t now = HAL_GetTick();
    uint32_t dt_ms = now - ramp_ms;
    float dt_s;
    float max_step;
    float ratio;
    int i;

    ramp_ms = now;
    if (!motor_on_state) {
        /* 没使能就别更新目标，否则下次使能会直接从旧目标开始 */
        return;
    }

    wheel_target[0] = left_mps;
    wheel_target[1] = right_mps;

    dt_s = (float)dt_ms / 1000.0f;
    if (dt_s > 0.2f) {
        dt_s = 0.2f;               /* 主循环被卡住过，别一次补太多 */
    }
    max_step = LD_MOTOR_MAX_ACCEL * dt_s;

    for (i = 0; i < 2; i++) {
        float d = wheel_target[i] - wheel_now[i];
        if (d > max_step) {
            d = max_step;
        } else if (d < -max_step) {
            d = -max_step;
        }
        wheel_now[i] += d;
    }

    /* m/s -> 占空比。WHEEL_CIRCUMFERENCE_M 与 MAX_WHEEL_MPS 必须和
     * ld_app_cfg_t 的 wheel_radius_m 保持一致，否则「限速」是假的。 */
    for (i = 0; i < 2; i++) {
        float duty;
        float v = wheel_now[i];
        int forward = (v >= 0.0f);

        if (v < 0.0f) {
            v = -v;
        }
        ratio = v / LD_MAX_WHEEL_MPS;
        if (ratio > 1.0f) {
            ratio = 1.0f;
        }
        duty = ratio * (float)LD_PWM_PERIOD;

        if (i == 0) {
            HAL_GPIO_WritePin(LEFT_DIR_GPIO_Port, LEFT_DIR_Pin,
                              forward ? GPIO_PIN_SET : GPIO_PIN_RESET);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)duty);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)duty);
        } else {
            HAL_GPIO_WritePin(RIGHT_DIR_GPIO_Port, RIGHT_DIR_Pin,
                              forward ? GPIO_PIN_SET : GPIO_PIN_RESET);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint32_t)duty);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, (uint32_t)duty);
        }
    }
}

/* ---------------------------------------------------------------- LED */

static void hal_led_set(float b)
{
    /* 用主 LED 的 PWM 通道；亮度曲线是线性的，感知上的非线性交给
     * 上位机（lightdog_light_auto 已经在做时间积分渐变）。 */
    uint32_t duty = (uint32_t)(b * (float)LD_PWM_PERIOD);
    if (duty > LD_PWM_PERIOD) {
        duty = LD_PWM_PERIOD;
    }
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, duty);
}

/* ---------------------------------------------------------------- 串口 */

/*
 * 发送队列。
 *
 * **必须拷贝一份**：app 层传给 uart_write 的是栈上的 buf，而 DMA 是异步的
 * ——不拷贝的话函数返回时栈被复用，发出去的是垃圾帧（表现为「上位机偶尔
 * 收到 CRC 错的帧」）。这类 bug 在台面上很难复现。
 */
typedef struct {
    uint8_t  buf[LD_FRAME_MAX];
    uint16_t len;
} tx_slot_t;

static tx_slot_t tx_q[LD_TX_QUEUE_FRAMES];
static uint8_t tx_head = 0;
static uint8_t tx_tail = 0;
static uint8_t tx_busy = 0;

static int hal_uart_write(const uint8_t *buf, uint32_t len)
{
    uint8_t next;
    tx_slot_t *slot;

    if (buf == NULL || len == 0u || len > LD_FRAME_MAX) {
        return 0;
    }

    if (tx_busy) {
        next = (uint8_t)((tx_head + 1u) % LD_TX_QUEUE_FRAMES);
        if (next == tx_tail) {
            /* 队列满：丢**当前**这一帧而不是覆盖旧帧。旧帧已经在排队的
             * 大概率是上次的状态上报，丢了更难查。 */
            return 0;
        }
    }

    slot = &tx_q[tx_head];
    memcpy(slot->buf, buf, len);
    slot->len = (uint16_t)len;
    tx_head = (uint8_t)((tx_head + 1u) % LD_TX_QUEUE_FRAMES);

    if (!tx_busy) {
        tx_busy = 1;
        (void)HAL_UART_Transmit_DMA(&huart2, tx_q[tx_tail].buf,
                                    tx_q[tx_tail].len);
    }
    return (int)len;
}

/* 由 HAL 的发送完成回调调用（在 stm32f4xx_it.c 的 HAL_UART_TxCpltCallback
 * 里转进来，或者直接在 CubeMX 的 USER CODE 段里调这个函数）。 */
void lightdog_hal_tx_complete(void)
{
    if (!tx_busy) {
        return;
    }
    tx_tail = (uint8_t)((tx_tail + 1u) % LD_TX_QUEUE_FRAMES);
    if (tx_tail == tx_head) {
        tx_busy = 0;
        return;
    }
    (void)HAL_UART_Transmit_DMA(&huart2, tx_q[tx_tail].buf, tx_q[tx_tail].len);
}

/* ---------------------------------------------------------------- 电池 */

static int hal_battery_read(float *pct, float *volt, float *amp)
{
    /* ADC 读分压；电流用 INA219/INA226 的话走 I2C，这里给的是 ADC 版本。
     * 换算系数必须按实际分压电阻算，写错的表现是「电量永远 100%」
     * 或「刚充满就报 0%」——后者会误触发回充。 */
    uint32_t raw = 0;
    float v_ref = 3.3f;
    float v_bat;

    (void)amp;
    if (HAL_ADC_Start(&hadc1) != HAL_OK) {
        return 0;
    }
    if (HAL_ADC_PollForConversion(&hadc1, 2) != HAL_OK) {
        HAL_ADC_Stop(&hadc1);
        return 0;
    }
    raw = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    /* 分压比 4:1 -> 电池电压 = adc 电压 x 4 */
    v_bat = ((float)raw / 4095.0f) * v_ref * 4.0f;
    if (volt != NULL) {
        *volt = v_bat;
    }
    if (pct != NULL) {
        /* 4S 锂电：16.8V 满 / 13.2V 空，线性内插只用于兜底；
         * 真机建议按实测放电曲线做查表。 */
        float p = (v_bat - 13.2f) / (16.8f - 13.2f) * 100.0f;
        if (p < 0.0f) {
            p = 0.0f;
        }
        if (p > 100.0f) {
            p = 100.0f;
        }
        *pct = p;
    }
    if (amp != NULL) {
        *amp = 0.0f;               /* 没有电流采样就报 0，别瞎猜 */
    }
    return 1;
}

/* ---------------------------------------------------------------- 编码器 */

/*
 * 16 位定时器计数 -> 32 位累计。
 *
 * 关键：用**有符号**的 16 位差处理回绕：
 *     int16_t delta = (int16_t)(now - prev);
 * 直接 `total += (now - prev)` 在回绕那一下会加 65000 多，里程计瞬间跳几十米。
 *
 * 为什么 50ms 一次读不会溢出：1560 计数/转，int16 半量程 32767 约 21 转，
 * 折算到 0.0325m 轮径是 4.3m —— 50ms 走 4.3m 意味着 86m/s，不会发生。
 * 但如果以后把上报周期改到秒级，这里就要改成用定时器溢出中断累计。
 */
static int32_t enc_total[2] = { 0, 0 };
static uint16_t enc_prev[2] = { 0, 0 };

static int32_t hal_encoder_read(uint8_t side)
{
    uint8_t i = side ? 1u : 0u;
    TIM_HandleTypeDef *htim = (i == 0u) ? &htim3 : &htim4;
    uint16_t now = (uint16_t)__HAL_TIM_GET_COUNTER(htim);
    int16_t delta = (int16_t)(now - enc_prev[i]);

    enc_total[i] += (int32_t)delta;
    enc_prev[i] = now;
    return enc_total[i];
}

/* ---------------------------------------------------------------- 看门狗 */

static void hal_iwdg_feed(void)
{
    /* 只在「收到合法心跳」时被调用（lightdog_app.c 的 handle_frame）。
     * 千万不要把它挪到 while(1) 里 —— 主循环卡死时那行照样执行，
     * 看门狗就完全失去意义。 */
    (void)HAL_IWDG_Refresh(&hiwdg);
}

/* ---------------------------------------------------------------- 导出 */

static const ld_hal_t g_lightdog_hal = {
    hal_now_ms,
    hal_motor_enable,
    hal_motor_set,
    hal_led_set,
    hal_uart_write,
    hal_battery_read,
    hal_encoder_read,
    hal_iwdg_feed
};

const ld_hal_t *lightdog_hal_stm32f4(void)
{
    return &g_lightdog_hal;
}

#endif /* USE_HAL_DRIVER && STM32F4 */
