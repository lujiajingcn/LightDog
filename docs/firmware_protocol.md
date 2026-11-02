# 固件 / MCU 协议（ROS 侧 ⇄ 下位机）

这份文档定义 **ROS 软件层已经做完的那半** 与 **必须由 MCU 固件实现的那半** 之间
的接口。写它的原因很直接：`lightdog_twist_mux.py` 的安全闸、`lightdog_mission`
的急停让路、`lightdog_led` 的亮度输出，逻辑上都已经完成并在仿真里验收过，但
**真正让轮子停下来的最后一道保险长在 MCU 上** —— 那段代码不在本仓库里，如果不把
契约写清楚，上车时会变成「双方都以为对方负责」。

一句话总结分工：

```
ROS 侧负责「决定该走多快、以及该不该走」
MCU 侧负责「一旦 ROS 侧失联，立刻自己停」   ← 只有 MCU 能保证这一条
```

MCU 的硬件选型（STM32/ESP32/…）、电机驱动型号都还没定，所以下面只写**语义和时序**，
不写具体寄存器。选型定下来后，把「帧格式」那一节替换成具体的串口/CAN 布局即可。

---

## 一、ROS → MCU 的三条下行话题

| 话题 | 类型 | 频率 | 语义 |
|---|---|---|---|
| `/cmd_vel_muxed` | `geometry_msgs/msg/Twist` | 由 `twist_mux` 的 `publish_rate` 决定（默认 10Hz） | 仲裁后的最终速度指令 |
| `/led/brightness` | `std_msgs/msg/Float32` | 以变化为主（渐变时约 10~20Hz） | 主灯亮度 0.0 ~ 1.0 |
| `/safety/heartbeat` | `std_msgs/msg/Bool` | 与 `/cmd_vel_muxed` 同频（同一 tick 里发） | 见第二节 |

### 1.1 `/cmd_vel_muxed`

- 单位：`linear.x` m/s，`angular.z` rad/s（差速/滑移转向，`linear.y` **恒为 0**，
  固件可以忽略，若不为 0 说明上游接错了，建议上报一次告警后忽略）。
- 已限幅：`twist_mux` 会把 `linear.x` 钳到 `max_linear`（默认 0.25）、
  `angular.z` 钳到 `max_angular`（默认 0.70）。**固件仍必须自己再钳一次** ——
  上位机参数改了、或者上位机被换成了别的程序，MCU 不该跟着一起越界。
- 发零速 ≠ 停发：两者语义不同，见下一节。
- 死区：`|linear.x| < 0.02` 且 `|angular.z| < 0.05` 时按 0 处理（避免电机嗡嗡叫）。
  这个死区是**固件侧**的，不要指望 ROS 侧做。

### 1.2 为什么速度指令和心跳要分成两条话题

看起来 `twist_mux` 每个周期都在发 `/cmd_vel_muxed`，只要「多久没收到 cmd_vel 就停车」
就够了，为什么还要一条独立的 `/safety/heartbeat`？

因为**停发本身就是一种合法的指令**。`lightdog_follower` 被调度层收回控制权时，
代码里有明确注释的规则：**先发一条零速，然后彻底停发**（否则 `twist_mux` 的 follow
源会永远处于活跃状态，Nav2 永远拿不到方向盘 —— 见 `docs/P2_person_follow.md` §7.8）。
真机上还会出现「上游节点全部正常、只是当前没人需要动」的长时间静默。

如果 MCU 把「没收到 cmd_vel」当成故障，那它会在系统正常待命时反复触发保护；
如果当成正常，那它在上位机真的崩了时也不会停。**同一个信号承担不了两种含义**，
所以拆开：

- `/cmd_vel_muxed` 断了 = 没必要动；
- `/safety/heartbeat` 断了 = **上位机死了**。

`twist_mux` 的实现保证了「只要它这个进程在跑、tick 在走，心跳就一定在发」——
心跳不受仲裁结果影响，所以它是干净的存活信号。

---

## 二、MCU 侧必须实现的看门狗（这一节是本文档的核心）

### 2.1 判据与时序

```
心跳超时          watchdog_ms = 300
执行动作          硬件层面断 PWM（H 桥 EN 拉低 / 驱动使能脚拉低），
                  不是「发一条零速指令」—— 零速指令依赖驱动还活着
恢复条件          连续收到 N=3 帧心跳（300ms）后才允许重新使能
恢复动作          重新使能后先保持零速 1 个周期，不收残留的旧指令
```

为什么是 300ms：`twist_mux` 默认 10Hz，300ms = 连续丢 3 帧才动作，能过滤单帧
抖动；同时 0.3s 在 0.25m/s 下只走 7.5cm，对人来说仍是「一碰就停」的量级。
**这个值必须和 `p2_params.yaml` 里的 `publish_rate` 一起调**：如果为了省 CPU 把
`publish_rate` 降到 5Hz，300ms 就只剩 1.5 帧，必须同步放大到 500~600ms。

### 2.2 断的是 PWM，不是指令

仿真里「停」= 发 `Twist()` 全零。真机上这两件事完全不同：

| 做法 | 上位机崩了 | 驱动过热/看门狗复位 | 线松了 |
|---|---|---|---|
| 发零速指令 | 零速也发不出去 | 不确定 | 不确定 |
| 断 PWM 使能 | **停** | **停** | **停**（默认失能） |

所以：**PWM 使能脚默认低电平，必须由固件主动拉高**。上电时序里宁可多花 100ms
等待 ROS 侧连通，也不要「一上电就使能、等上位机来告诉它停」。

### 2.3 失联期间的行为矩阵

| 情况 | ROS 侧 | MCU 侧期望行为 |
|---|---|---|
| 正常待命（无人可跟） | 无 `/cmd_vel_muxed`，有心跳 | 使能保持，速度 0。**不要**因为没速度就断使能 |
| 用户急停 | `/safety/estop = true` | 断 PWM + 点亮急停指示，且**忽略所有后续速度指令**直到 estop=false |
| 上位机进程崩溃 | 心跳停 | 300ms 后断 PWM，上报 `FAULT_LINK_LOST` |
| 上位机整机掉电 | 心跳停 | 同上（依赖 MCU 独立供电或至少维持到停车） |
| 本机驱动过流 | — | 立即断 PWM，上报 `FAULT_OVERCURRENT`，**不自动恢复**，需上位机显式清 |
| 电池过放 | 电量低于阈值 | 允许行驶到桩前，但拒绝新的大电流动作（由 ROS 侧 `dock_manager` 决策，MCU 只做最后兜底） |

---

## 三、`/safety/estop`：闭锁，不是事件

| 属性 | 值 |
|---|---|
| 类型 | `std_msgs/msg/Bool` |
| QoS | **RELIABLE + TRANSIENT_LOCAL + depth 1**（代码里是 `lightdog_qos.SAFETY_QOS`） |
| 语义 | `true` = 闭锁，**只有显式的 `false` 能解**，超时/重启都不自动解除 |
| 发布者 | `lightdog_voice`（语音「急停」）、`lightdog_server`（`POST /api/v1/estop`）、**物理按钮（固件，待实现）** |
| 订阅者 | `lightdog_twist_mux`（执闸）、`lightdog_mission`（让路，取消导航目标） |

### 3.1 为什么是 TRANSIENT_LOCAL

实测过一次真实后果：按下急停后，`lightdog_mission` 因为改参数被重启，它从
`estop=False` 开始初始化 —— 内存里「已经急停」这个事实跟着旧进程一起没了。
车还有 `twist_mux` 挡着，但调度层会以为自己可以跟随，于是放开 `/follow/enable`、
去抢 Nav2 的目标。

改成 `TRANSIENT_LOCAL(depth=1)` 之后，发布端保留「最后一次」状态，后启动的
订阅者握手时直接收到它。**实测：kill 掉 `lightdog_mission` 再拉起，0.8s 内就打出
「收到急停闭锁」，模式直接是 ESTOP。**

### 3.2 物理按钮（固件侧，待实现）

急停按钮必须是**硬件常闭/自锁**的：按下时机械断开驱动使能，**同时**（或之后）
通过 ROS 话题上报 `true`。顺序不能反 —— 不能设计成「先上报、等 ROS 确认再断」。

按钮上报时**必须用 `SAFETY_QOS`**，否则重启后的节点拿不到这个状态：

```python
# 固件桥接节点（示意）
from lightdog_qos import SAFETY_QOS
self.pub = self.create_publisher(Bool, '/safety/estop', SAFETY_QOS)
self.pub.publish(Bool(data=True))    # 按下
```

### 3.3 上位机侧的已知限制

`ros2 topic pub`（CLI）发急停**不可靠**：它是发完就退的短命进程，RELIABLE 的样本
还没被所有订阅者 ACK 就析构了，丢包甚至**按订阅者**发生 —— 实测出现过 mux 收到了、
mission 没收到。所以：

- 生产路径上的急停发布者必须是**常驻节点**（语音 / server / 固件桥接）。
- 调试时用 `bash sim/system.sh estop`，它做了闭环确认（读回 `/cmd_vel_mux_state`
  和 `/mission/mode`，两边都到位才报成功，否则重发最多 6 轮）。

---

## 四、`/led/brightness`：一个浮点，别在 MCU 里做动画

| 属性 | 值 |
|---|---|
| 类型 | `std_msgs/msg/Float32` |
| 范围 | 0.0 ~ 1.0（已经做过渐变，见 `lightdog_light_auto` 的 `fade_time`） |
| 频率 | 变化时约 10~20Hz，稳定时不发或低频重发 |

固件侧只做两件事：

1. **映射到 PWM**：`duty = round(brightness ** gamma * pwm_max)`，
   `gamma ≈ 2.2` 让低亮度段的观感线性（人眼对亮度是非线性的）。
   先按实测调，不要把 gamma 写死成 1.0 —— 那会让「10% 亮度」看起来像 40%。
2. **限幅与保护**：`brightness` 钳到 `[0, 1]`；`NaN` 按 0 处理并报错一次。

不要在 MCU 里做呼吸/渐变 —— 渐变已经在 ROS 侧按时间积分算好了
（`docs/P1_led_battery.md` 里解释过为什么不能用「每 tick 固定一步」）。
两边各做一半会导致亮度曲线变成台阶。

**真机上必须停掉 `lightdog_led` 节点**：它把亮度翻译成 Gazebo 材质自发光，
在真机上只会往一个没人转发的话题发消息，现象是「灯不亮、也没有任何报错」。

---

## 五、电池与底盘状态上报（MCU → ROS）

| 话题 | 类型 | 频率 | 说明 |
|---|---|---|---|
| `/battery_state` | `sensor_msgs/msg/BatteryState` | 1Hz 足够 | 只用 `percentage`（0.0~1.0）和 `voltage`；`percentage` 必须归一化 |
| `/odom` | `nav_msgs/msg/Odometry` | ≥ 20Hz | 见下方关于 yaw 的警告 |

### 5.1 `percentage` 的坑

`lightdog_dock_manager` 的低电判据直接比较 `percentage`。两个必须注意的点：

- **`percentage` 为 NaN 时不要当 0**：代码里会忽略 NaN 而不是判定「没电了」，
  但如果固件一直发 NaN，低电回充就永远不会触发。固件要么给真值，要么**不发**。
- **不要用电压反推百分比**：锂电池的电压-SOC 曲线在 3.6~3.9V 之间几乎是平的，
  反推出来会在 40%~70% 之间乱跳。要库仑计，或者干脆只上报原始电压、让
  `dock_manager` 用 `voltage` 的阈值判（默认参数就是按 ratio 做的，改之前先读
  `docs/P3_auto_dock.md`）。

### 5.2 `odom` 的 yaw

linorobot2 四驱 skid steer 的里程计 **yaw 高估约 35%**（仿真里就能测出来）。
所以：

- P3 停靠不需要依赖 odom 朝向，用 ArUco 视觉对准（`align_before_dock`）。
- 如果固件侧能拿到更准的轮速（编码器分辨率、轮距标定），把标定结果写进固件，
  不要在 ROS 侧用一个魔法系数去乘 —— 那个系数会随地面材质变。

---

## 六、串口帧格式（建议，待选型后定稿）

ROS 侧目前没有实现串口驱动，这一节是给固件作者的**建议**，不是既有实现：

```
帧 = SYNC(0xAA 0x55) | LEN(1B) | TYPE(1B) | PAYLOAD(N) | CRC16(2B, CCITT-FALSE)
```

- 所有多字节字段用小端。
- **CRC 错了就整帧丢弃并计数**，不要「尽力恢复」——半截的速度指令比没有更危险。
- 下行（ROS→MCU）与上行（MCU→ROS）各用独立 TYPE，不要复用。
- 心跳在总线上如果和速度指令同帧，那等于又回到「一个信号两种含义」的问题：
  **心跳必须能单独发**（可以是同一个 LEN/TYPE 下的空 PAYLOAD 帧，但必须能在
  不发速度的情况下发）。

---

## 七、上车验收清单（逐条可测）

| # | 测什么 | 怎么测 | 通过标准 |
|---|---|---|---|
| 1 | 看门狗真的断 PWM | 拉高使能让车低速前进，用 `kill -9` 杀掉 `twist_mux` | ≤ 400ms 内轮子物理停转，指示灯报 `FAULT_LINK_LOST` |
| 2 | 断的是使能不是指令 | 同上，用万用表/示波器看 H 桥 EN 脚 | EN 变低；不是「仍在输出 0 占空比」 |
| 3 | 待命不会误触发 | 正常待命 10 分钟（无速度、有心跳） | 不触发保护，不闪故障灯 |
| 4 | 急停闭锁 | 按下按钮 → 松开按钮 | 松开后仍不走；必须显式 `clear_estop` |
| 5 | 急停可继承 | 急停状态下 `kill` 掉 `lightdog_mission` 再拉起 | 新进程直接进 ESTOP（日志有「收到急停闭锁」） |
| 6 | 语音急停 | 对麦克风说「急停」 | 500ms 内断 PWM |
| 7 | 语音解锁 | 说「解除急停」 | 恢复行驶；**不能**被理解成再加一次锁 |
| 8 | 亮度映射 | `system.sh light on/off`，照度计测 | 0.0 全灭、1.0 满亮，中间无台阶 |
| 9 | 电量不跳变 | 静置 30 分钟看 `/battery_state` | 相邻两次差值 < 2%，无 NaN |
| 10 | 低电回充 | 注入 30% 电量 | `mode=DOCK_APPROACH` → 导航到桩前 → `DOCKING` |

第 4、5、6、7 条是本轮新增的安全语义，仿真里已经全部跑通；上车时**必须重测**，
因为它们的最后一环（真断 PWM）在仿真里不存在。

---

## 八、和现有文档的关系

| 想知道 | 看 |
|---|---|
| 为什么 max_linear=0.25、跟随距离怎么定的 | `docs/P2_person_follow.md` |
| 回充状态机、`/dock/state` 的取值 | `docs/P3_auto_dock.md` |
| LED 亮度曲线、电量判据 | `docs/P1_led_battery.md` |
| 哪些参数上真机必须改 | `docs/hardware_readiness.md` §一 |
| 各节点失效时的降级行为 | `docs/hardware_readiness.md` §三 |
