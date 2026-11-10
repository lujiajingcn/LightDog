# STM32F4 接线与集成

这个目录是 [`app/lightdog_hal.h`](../../app/lightdog_hal.h) 的一个参考实现。
**它不参与 `make -C firmware test` 的构建**（那个构建只验纯逻辑，宿主机没有
STM32 HAL）。真正编译它要用 CubeMX 生成的工程。

`#if defined(USE_HAL_DRIVER) && (STM32F405xx || STM32F407xx)` 之外的代码不会被
编译，所以误加进 host 构建也不会报错。

---

## 一、CubeMX 配置清单

| 外设 | 配置 | 为什么 |
|---|---|---|
| **UART2** | 115200 8N1，**DMA TX（Normal，不是 Circular）**，开 UART 全局中断 | 开中断才能拿到 DMA 发送完成回调；Circular 会掩盖「发到哪了」 |
| **TIM1** | 4 路 PWM（CH1/CH1N、CH2/CH2N、CH3/CH3N、CH4/CH4N），**20kHz**，ARR=8400 | 20kHz 在可听域外；ARR 要和 `LD_PWM_PERIOD` 一致 |
| **TIM2** | CH1 PWM，主灯亮度 | **不能和 TIM1_CH4 共用**：那是右轮 IN2，两者会互相覆盖比较寄存器 |
| **TIM3 / TIM4** | Encoder Mode（TI1 and TI2），**4 倍频** | 每转计数 = 线数 × 4 × 减速比，必须与 `ticks_per_rev` 一致 |
| **ADC1** | **3 通道 Scan + ContinuousConvMode + 循环 DMA**（顺序：电池分压→电流→温度 NTC），12 位 | `ld_app_tick` 在 1kHz 中断里跑，绝不能再忙等 ADC；DMA 持续把最新值刷进缓存，读只取快照（见代码里 F-002）。分压/增益写错的表现是「电量永远 100%」或「刚充满就报 0%」 |
| **IWDG** | 预分频 + 重载，**超时 1s** | 见下面「IWDG 超时不要设太短」 |
| **GPIO 输出** | `MOTOR_EN`、`LEFT_DIR`、`RIGHT_DIR`、`LED` | `MOTOR_EN` **硬件上必须下拉** |
| **GPIO 输入** | 急停按钮，**上升沿 + 下降沿都触发** | 自锁按钮：按下锁住、旋出解除，两个沿都要知道 |
| **定时器中断** | 1kHz（TIM6 或 SysTick） | `ld_app_tick` 的调用源 |

### IWDG 超时不要设太短

心跳是 10Hz（100ms 一帧），IWDG 由 `ld_app_tick` 在**控制环完整跑完一拍、
且心跳仍新鲜**时喂（`firmware/app/lightdog_app.c` 里唯一的喂狗点）。所以：

- 超时设 200ms → 丢几帧心跳就复位 MCU，车会莫名其妙重启；
- 超时设 1s → 连丢 10 帧才复位，正常丢包不受影响。

**绝不能在 UART RX ISR 里喂狗**：`ld_app_rx` 跑在接收中断里，主循环/1kHz
tick 卡死时它照样被心跳唤醒。那样 IWDG 永远不会超时，而电机保持最后一次
EN=1 和占空比一直出力 —— 这是最危险的一类失效。

复位是**第二层**兜底：300ms 失联已经由 `lightdog_watchdog` 断使能了，
IWDG 只在「控制环整个卡死、连 watchdog 判定都跑不到」时才动手。
急停 / 过流闭锁期间只要心跳还在就继续喂狗，是故意的：反复复位会清掉 RAM
里的闭锁状态，而上电默认断使能并不能替代「谁锁的、什么时候解」。

---

## 二、接线表（示例，按自己的原理图改）

| 信号 | 引脚（示例） | 说明 |
|---|---|---|
| UART2 TX / RX | PA2 / PA3 | 与树莓派 40pin 的 8/10 脚交叉连接，**3.3V 直连**，共地，串 22Ω |
| MOTOR_EN | PB0 | 接 H 桥的 EN/STBY，**外部 10k 下拉到 GND** |
| TIM1_CH1/CH2 | PE9 / PE11 | 左轮 H 桥 IN1/IN2 |
| TIM1_CH3/CH4 | PE13 / PE14 | 右轮 H 桥 IN1/IN2 |
| LEFT_DIR / RIGHT_DIR | PB1 / PB2 | 方向脚（若 H 桥用 IN1/IN2 正交驱动则不需要） |
| TIM3_CH1/CH2 | PA6 / PA7 | 左编码器 A/B |
| TIM4_CH1/CH2 | PB6 / PB7 | 右编码器 A/B |
| LED PWM | PA5（`TIM2_CH1`） | 主灯亮度，**必须独立定时器**；TIM1 四个通道已全给轮子，共用比较寄存器会让开灯把右轮拉满 |
| 电池 ADC | PA0 | 分压后输入，**不超过 3.3V** |
| 急停按钮 | PC13 | 一端接 GND，另一端上拉；按下为低 |

> PWM 布线远离编码器线与 UART：编码器计数乱跳的表现是 odom 原地漂移，
> 通常会被误判成「轮距没标定」。

---

## 三、集成步骤

```c
/* main.c，CubeMX 的 USER CODE 段 */

#include "lightdog_app.h"

extern const ld_hal_t *lightdog_hal_stm32f4(void);   /* port 层导出 */
static ld_app_t g_app;

/* UART 接收：DMA + 空闲中断（HAL_UARTEx_ReceiveToIdle_DMA） */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart->Instance == USART2) {
        /* 直接在回调里喂解析器：ld_app_rx 只做解析与记账，不阻塞 */
        ld_app_rx(&g_app, g_rx_buf, size);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, g_rx_buf, sizeof(g_rx_buf));
    }
}

/* DMA 发送完成：把 port 层队列推进一步 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        lightdog_hal_tx_complete();
    }
}

/* 急停按钮：两个沿都要触发 */
void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
    if (pin == ESTOP_BTN_Pin) {
        int pressed = (HAL_GPIO_ReadPin(ESTOP_BTN_GPIO_Port, ESTOP_BTN_Pin)
                       == GPIO_PIN_RESET);
        ld_app_on_button(&g_app, pressed, HAL_GetTick());
    }
}

/* 1kHz 定时器中断 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) {
        ld_app_tick(&g_app, HAL_GetTick());
    }
}

int main(void)
{
    /* ... CubeMX 初始化 ... */

    ld_app_cfg_t cfg;
    ld_app_cfg_defaults(&cfg);
    cfg.wheel_radius_m = 0.0325f;    /* 用卡尺量，别抄 */
    cfg.wheel_base_m   = 0.300f;
    cfg.ticks_per_rev  = 1560.0f;    /* 线数 x4 x 减速比 */

    /* 安全阈值（F-010）：务必按实机电机/电池/功率级 datasheet 填，不要抄默认。
     * 命中即断使能（内部闭锁），恢复要跨过迟滞阈值并重新攒 3 帧心跳。
     * 设成 <=0 关闭对应单项；max_temp_c 需要有温度通道（ADC 第 3 路）才有意义。 */
    cfg.safety_period_ms = 100u;     /* 安全评估周期 */
    cfg.max_current_a    = 8.0f;     /* 堵转裕度之外的过流阈值 */
    cfg.min_voltage_v    = 9.0f;     /* 欠压硬截止 */
    cfg.max_temp_c       = 85.0f;    /* 功率级/电机热截止 */

    ld_app_init(&g_app, lightdog_hal_stm32f4(), &cfg);

    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, g_rx_buf, sizeof(g_rx_buf));
    HAL_TIM_Base_Start_IT(&htim6);

    while (1) {
        /* 主循环不要做控制：控制全在 1kHz 中断里。
         * 这里可以放状态灯、调试串口之类的慢活。 */
    }
}
```

---

## 四、和 ROS 侧参数的对应关系

树莓派那边这几个值必须和这里一致，否则「限速」和「里程计」会静默地错：

| 固件侧 | ROS 侧 | 不一致的表现 |
|---|---|---|
| `cfg.wheel_radius_m` | URDF 的轮半径 | 里程计按比例缩放，导航越走越偏 |
| `cfg.wheel_base_m` | URDF 的轮距 | 转弯角度系统性偏大/偏小 |
| `cfg.ticks_per_rev` | 编码器线数 × 倍频 × 减速比 | 距离整体差一个倍数 |
| `cfg.max_linear_mps` | `mcu_params.yaml` 的 `max_linear` | 固件比 ROS 限得松 = 限速形同虚设 |
| `LD_MAX_WHEEL_MPS` | 电机空载转速 × 轮周长 | 占空比到 100% 也跑不到目标速度 |

建议上车第一件事就是这段标定：让车直行 2m，量实际距离，反推
`wheel_radius_m`；原地转 10 圈，量实际角度，反推 `wheel_base_m`。

---

## 五、安全链路在真机上怎么落地

1. **F-003（临界区）**：`ld_app_tick`（1kHz 定时器中断）、`ld_app_rx`（UART RX
   中断）、`ld_app_on_button`（按钮 EXTI 中断）三个上下文都会读-改-写同一份看门狗
   状态（`estop` / `fault` 掩码 / `enabled`）。app 层现在把这些 RMW 包进 HAL 的
   `enter_critical`/`exit_critical`；本文件用 PRIMASK 保存/恢复的小栈实现它们，
   临界区里只有寄存器级快操作，绝不含 ADC / 三角函数 / uart 写。

2. **F-009（TX ring）**：`tx_head`/`tx_tail`/`tx_busy` 已被声明为 `volatile`，入队
   那段（head 快照 → memcpy → 推进 head → 置 busy）整段进临界区，防止发送完成中断
   从中间打断，看到半更新的指针或把 `busy=1` 重排到 `HAL_UART_Transmit_DMA` 之后。

3. **F-010（安全故障位）**：过流 / 欠压 / 过温现在真的会被置位并强制断使能（等于
   一次内部急停），带迟滞解除，解除后仍要重新攒 3 帧心跳才出力。温度来自 ADC 第 3
   路经 `temp_read` 取；通道没接（恒 0）时 `temp_read` 返回 0，绝不据此误置 OVERTEMP。
   ENCODER 位只上报、**不断电**（一次计数异常不该把车锁死在原地）。

4. **编码器健康由板级喂**：port 暴露一个非静态钩子，任何检测手段检测到异常就调它：

   ```c
   /* side: 0=左 1=右；err 非 0=故障。定时器溢出/换相错误中断、
    * 「指令在动但计数长期不动」的堵转看门狗、SSI/增量线信号丢失诊断都可以调。 */
   void lightdog_hal_report_encoder_error(uint8_t side, int err);
   ```

   没有这类检测时标志恒 0，`hal_encoder_error` 返回 0，ENCODER 位不会被误置。

---

## 六、这个参考实现没验证什么

如实列出，别当成已验收：

1. **没有在真实板子上编译过**（仓库里没有 HAL 头文件）。可能有拼写/宏名错误，
   要按 CubeMX 生成的 `main.h` 改。
2. **电机、编码器、ADC 的换算系数是示例值**。必须按实车标定，见第四节。
3. **DMA 发送队列没有实测过丢帧率**。115200 下 10Hz 心跳 + 20Hz 里程计
   （20 字节）+ 1Hz 电池，带宽占用约 5%，队列 4 帧足够；但这是估算。
4. **物理急停的响应时间是设计值，不是实测值**。要求是「中断里直接断使能」，
   实际延迟取决于 EXTI 优先级和 H 桥的 EN 关断时间，要用示波器量。

第 1、2 条在 `docs/hardware_readiness.md` 的验收清单里对应第 1、2、4 项
（轮子物理停转 / EN 脚电平 / 按钮），那三项永远是「待人工」。
