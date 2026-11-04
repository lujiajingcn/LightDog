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

/* ---------------------------------------------------------------- 临界区（F-003）*/

/*
 * Cortex-M 上屏蔽/恢复中断的最小实现，供 app 层把「共享看门狗状态(estop/fault)
 * 的读-改-写」和 UART RX ISR / 按钮 EXTI ISR 互斥。app 层刻意不嵌套临界区，但这里
 * 仍做成 PRIMASK 保存/恢复的小栈：既支持偶发嵌套，又保证「进来时若本已关中断，
 * 出去不会误把中断打开」。临界区只包几条寄存器级语句，绝不涵盖 ADC/DMA/串口写。
 */
#define LD_CRIT_MAX 4u
static volatile uint32_t crit_pmask[LD_CRIT_MAX];
static volatile uint32_t crit_depth = 0;

static void hal_enter_critical(void)
{
    uint32_t save = __get_PRIMASK();
    __disable_irq();
    if (crit_depth < LD_CRIT_MAX) {
        crit_pmask[crit_depth] = save;
    }
    crit_depth++;
}

static void hal_exit_critical(void)
{
    uint32_t save = 0u;

    if (crit_depth == 0u) {
        /* 不配对保护：宁可维持现状，也不要凭一个多余的 exit 打开中断 */
        return;
    }
    crit_depth--;
    if (crit_depth < LD_CRIT_MAX) {
        save = crit_pmask[crit_depth];
    }
    __set_PRIMASK(save);
}

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
/* F-009：head/tail/busy 被两个上下文共享 —— 主循环的 hal_uart_write 改 head，
 * UART 发送完成 DMA 中断的 lightdog_hal_tx_complete 改 tail/busy。不加 volatile，
 * 编译器可能把 busy 判定缓存在寄存器、或把 `tx_busy=1` 重排到 HAL_UART_Transmit_DMA
 * 之后，导致「入队了但没人启动发送」或「完成中断读到陈旧 head 误判队列空而丢帧」。
 * 单字节队列指针在 Cortex-M 上是原子的读写，临界区只用于把 head/busy 两者保持一致。 */
static volatile uint8_t tx_head = 0;
static volatile uint8_t tx_tail = 0;
static volatile uint8_t tx_busy = 0;

static int hal_uart_write(const uint8_t *buf, uint32_t len)
{
    uint8_t next;
    uint8_t head;
    tx_slot_t *slot;
    int start = 0;

    if (buf == NULL || len == 0u || len > LD_FRAME_MAX) {
        return 0;
    }

    /* 入队整段做在临界区里：head/busy 的一致性不能被发送完成中断从中间打断。
     * 里面只有 memcpy 到固定槽 + 两个字节指针运算，几微秒级，可接受。 */
    hal_enter_critical();

    if (tx_busy) {
        next = (uint8_t)((tx_head + 1u) % LD_TX_QUEUE_FRAMES);
        if (next == tx_tail) {
            /* 队列满：丢**当前**这一帧而不是覆盖旧帧。旧帧已经在排队的
             * 大概率是上次的状态上报，丢了更难查。 */
            hal_exit_critical();
            return 0;
        }
    }

    head = tx_head;
    slot = &tx_q[head];
    memcpy(slot->buf, buf, len);
    slot->len = (uint16_t)len;
    tx_head = (uint8_t)((head + 1u) % LD_TX_QUEUE_FRAMES);

    if (!tx_busy) {
        tx_busy = 1;
        start = 1;              /* DMA 启动放到临界区外，但 busy 已在锁内置位 */
    }

    hal_exit_critical();

    if (start) {
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

/* ---------------------------------------------------------------- 电池 / 温度 */

/*
 * F-002：绝不在 tick（1kHz 上下文）里做阻塞式 ADC。
 *
 * 旧写法 HAL_ADC_Start + HAL_ADC_PollForConversion(2ms) 是忙等：单次转换+轮询
 * 最坏把 CPU 顶住约 2ms，控制环抖动直接到 ms 级，还会挤掉 UART 收到的字节
 * （而 F-010 之后电池读被安全评估以 ~10Hz 频繁调用，等于把这个坑放大 10 倍）。
 * 正确做法：hadc1 配成 Scan + ContinuousConvMode + DMA 循环搬运，一开播就持续
 * 把三个通道刷进 adc_raw，读的时候只取最新一拍的缓存 —— 全程零等待。
 */
#define LD_ADC_CH_NUM 3u
/* [0]=电池分压 [1]=电流 [2]=温度 NTC；顺序必须与 CubeMX 里的扫描序列一致 */
static volatile uint16_t adc_raw[LD_ADC_CH_NUM] __attribute__((aligned(4)));
static uint8_t adc_started = 0;

static void adc_ensure_started(void)
{
    if (adc_started) {
        return;
    }
    /* 需要 hadc1 在 CubeMX 里配好：多通道 Scan、ContinuousConvMode、DMA 循环。 */
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_raw, LD_ADC_CH_NUM) == HAL_OK) {
        adc_started = 1;
    }
}

/* 换算系数按实际分压电阻/分流器/热敏电阻改，写错的表现是「电量永远 100%」
 * 或「刚充满就报 0%」（后者会误触发回充）。 */
#define LD_VREF          3.3f
#define LD_ADC_FULLSCALE 4095.0f
#define LD_BAT_DIV       4.0f                 /* 分压比 4:1 -> x4 */
#define LD_AMPS_PER_VOLT 0.0f                 /* 电流通道增益；没接保持 0=不判过流 */

static int hal_battery_read(float *pct, float *volt, float *amp)
{
    uint16_t rv;
    uint16_t rc;
    float v_bat;

    adc_ensure_started();
    if (!adc_started) {
        /* ADC 起不来就这拍不报：上层会跳过，绝不报 0% 去误触发回充 */
        return 0;
    }

    rv = adc_raw[0];              /* 单次 16 位读，DMA 只会整体刷新，取到的是某拍的快照 */
    rc = adc_raw[1];

    v_bat = ((float)rv / LD_ADC_FULLSCALE) * LD_VREF * LD_BAT_DIV;
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
        /* 有电流采样时按增益换算；没接（增益 0）就报 0，OVERCURRENT 不会据此误触发 */
        *amp = ((float)rc / LD_ADC_FULLSCALE) * LD_VREF * LD_AMPS_PER_VOLT;
    }
    return 1;
}

static int hal_temp_read(float *celsius)
{
    uint16_t rt;

    if (celsius == NULL) {
        return 0;
    }
    adc_ensure_started();
    if (!adc_started) {
        return 0;
    }
    rt = adc_raw[2];
    if (rt == 0u) {
        /* 通道没接（恒 0）当作「没有该传感器」：返回 0，上层不会据此置 OVERTEMP，
         * 避免「没装热敏电阻却误刹车」。 */
        return 0;
    }
    /* NTC 换算高度依赖具体热敏电阻 B 值与分压，这里给线性示意，务必按 datasheet 替换 */
    *celsius = ((float)rt / LD_ADC_FULLSCALE) * LD_VREF * 100.0f;
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

/*
 * 编码器通道健康（F-010 的 ENCODER 位来源）：本函数只读一个由外部置位的标志。
 * 谁置位由板级决定 —— 定时器溢出/换相错误中断、看门狗式「指令在动但计数长期不动」
 * 的堵转检测、SSI/增量线的信号丢失诊断等，检测到异常时调用
 * lightdog_hal_report_encoder_error()。没有这类硬件检测时标志恒 0，ENCODER 位不会
 * 被误置（也就不会因为一次计数抖动把车锁死）。 */
static volatile uint8_t enc_err_flag[2] = { 0, 0 };

void lightdog_hal_report_encoder_error(uint8_t side, int err)
{
    enc_err_flag[side ? 1u : 0u] = err ? 1u : 0u;
}

static int hal_encoder_error(uint8_t side)
{
    return (int)enc_err_flag[side ? 1u : 0u];
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
    hal_iwdg_feed,
    /* 可选段（顺序必须与 lightdog_hal.h 一致）：接上后 F-003 的临界区与
     * F-010 的过温/编码器健康在真机上才真正生效。 */
    hal_enter_critical,
    hal_exit_critical,
    hal_temp_read,
    hal_encoder_error
};

const ld_hal_t *lightdog_hal_stm32f4(void)
{
    return &g_lightdog_hal;
}

#endif /* USE_HAL_DRIVER && STM32F4 */
