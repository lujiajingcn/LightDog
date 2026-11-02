# 固件参考实现（STM32）

这里**不是**能烧进板子的完整固件：没有 HAL、没有电机驱动、没有编码器 ISR。
这里是**安全关键的那一段逻辑**，写成了纯 C99，所以能在 PC 上编译、在 PC 上测。

为什么值得单独抽出来：这一层是唯一能在上位机 `kill -9` 之后刹住车的东西。
写在 `while(1)` 里就只能「上车试一下」验证，而这类 bug 试一次的代价可能是撞墙。

## 目录

```
core/lightdog_proto.{h,c}     帧编解码 + CRC + 流式解析器（无 HAL）
core/lightdog_watchdog.{h,c}  300ms 看门狗判定（无 HAL）
test/test_proto.c             自检：黄金向量 + 解析边界 + 看门狗时序
test/vectors.txt              黄金向量，与 lightdog/lightdog_mcu_proto.py 共读
```

## 跑自检

```bash
make -C firmware test
# cc -std=c99 -Wall -Wextra -Wpedantic -Werror -O2 -Icore ...
#   向量：10 行
# 94 项检查，0 项失败
```

仿真容器里只有 ROS 工作空间，没有这个仓库的 `firmware/` 目录（挂载点只到包目录），
所以要么在宿主机上装个 gcc，要么拷进去：

```bash
docker cp firmware sim:/tmp/fw
docker exec sim bash -lc "cd /tmp/fw && make test"
```

`-Werror -Wpedantic` 是刻意留着的：固件代码要过得了最严格的编译，才有资格
去管「300ms 内必须断 PWM」这种事。

## 集成到真实的 STM32 工程

三处调用点，别的都不用改：

```c
/* 1) UART 收到字节（DMA 半满/空闲中断里都行） */
ld_frame_t frames[4];
int n = ld_parser_push(&g_parser, rx_buf, rx_len, frames, 4);
for (int i = 0; i < n; ++i) {
    switch (frames[i].type) {
    case LD_T_HEARTBEAT:
        ld_wd_heartbeat(&g_wd, HAL_GetTick());   /* 只有这一处喂狗 */
        break;
    case LD_T_CMD_VEL:  /* 8 字节 -> 两个 float */ g_target = decode(frames[i]); break;
    case LD_T_ESTOP:    ld_wd_set_estop(&g_wd, frames[i].payload[0] != 0, 0); break;
    case LD_T_LED:      g_brightness = ...; break;
    }
}

/* 2) 主循环 / 1kHz 定时器里 */
ld_wd_tick(&g_wd, HAL_GetTick());
if (!ld_wd_output_enabled(&g_wd)) {
    motor_disable();                 /* 拉低 H 桥 EN，不是「输出 0 占空比」 */
} else {
    motor_enable();
    if (g_wd.hold_zero) { motor_set(0, 0); ld_wd_zero_applied(&g_wd); }
    else                { motor_set(clamp(g_target)); }
}

/* 3) 物理急停按钮的中断里（先硬件断使能，再上报） */
motor_disable();
ld_wd_set_estop(&g_wd, 1, 1);
send_frame(LD_T_ESTOP_STATE, &on, 1);
```

### 四条必须遵守的

1. **IWDG 只在收到心跳时喂。** 放到 `while(1)` 里喂的话，主循环卡死时
   喂狗照跑 —— 等于没有看门狗。这是本节最重要的一条。
2. **`HAL_GetTick()` 的溢出由 `ld_wd_tick` 用无符号减法处理了**（49 天回绕），
   调用方不要自己写 `now > last + 300`。
3. **上电时驱动使能脚必须是低**（硬件下拉），由固件主动拉高：
   宁可多等 300ms 收心跳，也不要「一上电就使能、等上位机来告诉它停」。
4. **`motor_disable()` 要断使能（EN/STBY），不是输出 0 占空比。**
   驱动过热复位、线松了、MCU 自己复位 —— 只有断使能三种情况都停得住。

### 时序预算

| 事件 | 目标 | 由谁保证 |
|---|---|---|
| 上位机进程死 | ≤ 400ms 轮子停 | 心跳 300ms 超时 + 主循环 1kHz |
| 物理按钮按下 | 立即 | 中断里直接断使能，不等通信 |
| 解急停之后重新能动 | ≥ 3 帧心跳（约 300ms） | `LD_WD_RECOVER_FRAMES` |
| 恢复后第一拍 | 先出零速 | `hold_zero` + `ld_wd_zero_applied` |

验收入口是 `deploy/acceptance.py` 的第 1、2、4 条（会 kill 掉 twist_mux，
看轮子是否物理停转、EN 脚是否变低）。

## 与 ROS 侧的对应关系

| 帧 | ROS 侧谁发/谁收 |
|---|---|
| `CMD_VEL` | `lightdog_twist_mux` → `/cmd_vel_muxed` → `lightdog_mcu_bridge` |
| `HEARTBEAT` | `lightdog_twist_mux`（同一 tick）→ 桥接节点转发 |
| `ESTOP` | `/safety/estop`（语音 / 手机 / 物理按钮）→ 桥接节点转发 |
| `LED` | `/led/brightness`（`lightdog_light_auto` 已做渐变） |
| `BATTERY` | 桥接节点 → `/battery_state` |
| `ODOM` | 桥接节点 → `/odom` |
| `ESTOP_STATE` | 桥接节点 → `/safety/estop`（`SAFETY_QOS`，闭锁可继承） |
| `FAULT` | 桥接节点 → `/mcu/fault`（JSON）+ 日志 |

帧格式、TYPE 表、故障位、解析策略见 `docs/firmware_protocol.md` §六。
改协议时**三处一起改**（文档、Python 常量、C 宏）+ 补一行 `test/vectors.txt`。
