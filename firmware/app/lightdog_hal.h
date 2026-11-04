/*
 * LightDog 固件平台接口（HAL 边界）。
 *
 * 这一层是「应用逻辑」和「具体芯片」之间唯一的接缝。约定两条：
 *   1. 本文件**只依赖 stdint.h**，不 include 任何厂商头文件。
 *   2. 实现里不许阻塞：now_ms / motor_set / uart_write 都在 1kHz 或 ISR
 *      上下文被调用。要在里面做阻塞操作（比如等 UART 发送完成），
 *      请自己改成 DMA + 队列，别在这层加锁。
 *
 * 移植到新板子 = 实现这 8 个函数，其它文件一行不用改。
 * 参考实现见 port/stm32f4/lightdog_hal_stm32f4.c（含接线说明）。
 */
#ifndef LIGHTDOG_HAL_H
#define LIGHTDOG_HAL_H

#include <stdint.h>

typedef struct {
    /* 单调递增的毫秒计数。可以任意起点，但必须**永不倒退**。
     * 上层所有超时判定都用无符号减法，49 天回绕是安全的。 */
    uint32_t (*now_ms)(void);

    /* 拉高/拉低驱动使能（EN 或 STBY）。
     * 关的时候必须是**真的断使能**，不是输出 0 占空比 —— 见 firmware/README.md
     * 铁律 4。上电时硬件必须默认下拉，由固件主动拉高。 */
    void (*motor_enable)(int on);

    /* 左右轮目标线速度，单位 m/s，已由上层限幅。正 = 前进。
     * 实现负责换算成 PWM 与方向，并且**必须自己做斜坡限加速**：
     * 从 0 跳到 0.5m/s 会打滑，而打滑的代价是里程计立刻漂掉。 */
    void (*motor_set)(float left_mps, float right_mps);

    /* LED 亮度 0.0~1.0（已钳位）。 */
    void (*led_set)(float brightness);

    /* 发一段字节。返回实际发出的字节数，0 表示失败。
     * **必须非阻塞**（DMA 或环形缓冲），否则 10Hz 的上报会拖慢控制环。 */
    int (*uart_write)(const uint8_t *buf, uint32_t len);

    /* 读电池：百分比 0~100、电压 V、电流 A。返回 0 表示这次读失败
     * （上层会跳过这一拍，不会上报 0% 这种会让上位机误判的值）。 */
    int (*battery_read)(float *percent, float *volt, float *amp);

    /* 读编码器累计计数。side: 0=左 1=右。允许回绕（上层用有符号差）。
     * 返回失败要**保持上次的值**，不要返回 0 —— 那会让里程计一次跳很远。 */
    int32_t (*encoder_read)(uint8_t side);

    /* 喂独立看门狗 IWDG。
     * **只允许在收到合法心跳时调用**（见 lightdog_app.c 的 handle_frame）。
     * 放进 while(1) 里喂，主循环卡死时照样喂，等于没有看门狗。 */
    void (*iwdg_feed)(void);

    /* ---- 以下为可选项，NULL 表示该能力不存在，上层自动跳过 ---- */

    /* 临界区：短暂屏蔽中断，用来把「共享看门狗状态（estop / fault 掩码）」的
     * 读-改-写做成原子，防止 UART ISR / 按钮 ISR / 1kHz tick 三个上下文互相
     * 打断（见 F-003）。务必**成对、且只包住最少的几条语句**——里面不要做
     * ADC、三角函数、 uart 写这类慢操作，否则等于把中断关了几十微秒丢串口字节。
     * 允许被嵌套调用前先自查：app 层刻意不嵌套。 */
    void (*enter_critical)(void);
    void (*exit_critical)(void);

    /* 读板载温度（摄氏度）。返回 0 表示这次读失败或没有该传感器
     * （上层不会据此置 OVERTEMP，避免传感器没接就误刹车）。 */
    int (*temp_read)(float *celsius);

    /* 编码器通道健康检查。side: 0=左 1=右。返回非 0 表示该侧编码器故障
     * （如定时器溢出、信号丢失）。没有该能力时置 NULL，ENCODER 位不会被自动置位。 */
    int (*encoder_error)(uint8_t side);
} ld_hal_t;

#endif /* LIGHTDOG_HAL_H */
