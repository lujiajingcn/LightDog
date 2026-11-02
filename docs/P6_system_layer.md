# P6：系统层（任务调度 / 语音链路 / 软件安全闸）

P1~P5 各自把一条功能做通了：灯会亮、车会跟、会回充、能录像、会报警。但它们
**互不知道对方存在** —— 每验证一个阶段，都要先把别的阶段手动停掉。真车上不能这样。

这一层补的是「谁说了算」和「出事怎么办」，不改任何一条已有的功能逻辑：

```
          ┌──────────────────────────────────────────────┐
          │  lightdog_mission   模式仲裁（只决策，不产生速度）│
          └──────────────────────────────────────────────┘
              │ /follow/enable            │ /mission/hold_motion
              ▼                           ▼
   lightdog_follower              lightdog_twist_mux ──► /cmd_vel_muxed ──► 底盘
   （放行才发速度）                （急停 / 按住 / 心跳）
                                        ▲
              lightdog_voice ──/safety/estop──┘
              /voice/command ──┘
```

## 一、为什么必须有这一层

### 缺口 1：跟随会把 Nav2 压死

`twist_mux` 的仲裁规则是「优先级高的源只要还在发，就它说了算」。而
`lightdog_follower` 一旦 `enabled=true`，即使人已经走了、它只是在维持零速，
它**仍然在发** —— 于是 `follow` 源永远活跃，Nav2（prio 20，低于 follow）
永远拿不到方向盘。

同一个坑还有一个更隐蔽的版本：如果为了「让出方向盘」干脆**彻底停发**，那么
下游 `twist_mux` 会在 `hold` 超时后把输出归零 —— 看起来对，但此时 Nav2 发来的
速度**也没人转发**（因为 mux 只在有活跃源时才输出那个源的速度）。所以「让位」
必须是「先发一条零速、再停发」这个精确动作。

现在这件事由 `/follow/enable` 负责：调度层说 `true`，跟随器才发速度；说
`false`，跟随器**先发一条零速然后彻底停发**（实现在 `_follow_gate()`）。

### 缺口 2：没人负责「从远处回桩」

`lightdog_dock_manager` 只会在「桩已经出现在视野里」之后接手。可从客厅另一头
回到桩前这段路属于导航，谁来决定什么时候该走这段？告警来了、人还在旁边、
电量还够 —— 这些条件横跨 P2/P3/P5，谁都不该单独决策。

所以有了 `lightdog_mission`：它**订阅所有状态、只发布决策**，一行速度代码都没有。
这种「决策和产出分离」不是为了好看，是为了它能在离线测试里被穷举
（`test_system.py` 里 30 多个用例全是纯函数调用，不需要 ROS）。

## 二、模式仲裁

优先级从高到低，**不要随意重排**：

| 优先级 | 模式 | 触发条件 | 跟随放行 | 按住轮子 | 导航 |
|---|---|---|---|---|---|
| 1 | `ESTOP` | 收到 `/safety/estop=true`（闭锁） | ✗ | ✓ | 取消 |
| 2 | `ALERT_HOLD` | `/alert/state ∈ {CONFIRMING, ALARM, ACKED}` 且 5s 内新鲜 | ✗ | ✓ | 取消 |
| 3 | `DOCKING` | `/dock/state ∈ {ALIGNING, DOCKING}` 且 5s 内新鲜 | ✗ | ✗ | — |
| 4 | `DOCKED` | `/dock/state == DOCKED` 且新鲜 | ✗ | ✗ | — |
| 5 | `MANUAL` | 命令 `manual` | ✓（跟随器自己决定） | ✗ | 取消 |
| 6 | `DOCK_APPROACH` | 电量低于 `approach_ratio`(0.35) 且已过冷却 | ✗ | ✗ | 到桩前 1m |
| 7 | `FOLLOW` | `follow_enabled` 且 present 新鲜 | ✓ | ✗ | 取消 |
| 8 | `PATROL` | `patrol_enabled` 且有路点 | ✗ | ✗ | 下一个路点 |
| 9 | `IDLE` | 以上都不满足 | ✗ | ✗ | 取消 |

几条顺序的理由：

- **ESTOP 在最上面**，而且它不走「按住」通道 —— 按住会过期，急停不会。两者是
  不同的东西，`twist_mux` 里也是分开两个变量。
- **ALERT 高于 DOCKING**：人可能倒在回桩的路径上。宁可停下等，也不要在
  「充电」这个理由下继续走。
- **DOCK_APPROACH 低于 MANUAL**：人工接管时不要跟它抢导航目标。
- **FOLLOW 低于 DOCK_APPROACH**：电量低到 35% 时，跟着人走没有意义；
  但要能走完回桩这段路，所以采用**迟滞**：0.35 触发、0.45 才解除，
  避免电量在阈值附近抖动时反复切换。

`decide()` 是纯函数（不产生副作用、不碰时间以外的状态），`tick()` 才落地。
所有时间判据一律用 `time.monotonic()` 墙钟，**不用 ROS 时钟** ——
`use_sim_time` 一旦没接上 `/clock`，ROS 时钟会停走，所有超时永久失效。

## 三、话题契约

```
订阅  /person/present       Bool       P2 检测器
订阅  /dock/state           String     P3 回充状态机
订阅  /dock/battery_ratio   Float32    电量比（已做过迟滞处理前的原值）
订阅  /alert/state          String     P5 告警状态
订阅  /safety/estop         Bool       人工急停（SAFETY_QOS，见第五节）
订阅  /mission/command      String     命令入口（语音 / 手机 / CLI）

发布  /mission/state        String     JSON：mode / reason / since / age /
                                       follow_enabled / hold / manual / estop ...
发布  /mission/mode         String     纯字符串，方便 `ros2 topic echo`
发布  /follow/enable        Bool       跟随器放行门
发布  /mission/hold_motion  Bool       按住轮子（1Hz 刷新，3s 不刷新自动放开）
发布  /dock/command         String     dock / undock / cancel
```

命令集合：`follow / follow_off / auto / manual / patrol_on / patrol_off /
dock / undock / cancel / status`。未知命令只记一条 warn 并返回，**不猜**。

### 关于 `/mission/state` 的 `reason`

它描述的是「为什么是现在这个模式」，所以 `IDLE` 也必须有自己的一句
（`待命（无事可做）`）。少了这一支，状态里会出现 `mode=IDLE, reason=检测到人`
这种自相矛盾的快照 —— 人走了、车停着，状态却写着看见人。手机端和现场排查
都会被它带偏，`test_idle_reason_does_not_stay_stale` 专门钉住了这条。

## 四、语音链路

```
外部 ASR ──/voice/command──► lightdog_voice ──/mission/command──► 调度层
                                          ├──/light/mode───────► 自动灯
                                          └──/safety/estop─────► 安全闸

告警状态机 ──/alert/voice──► lightdog_voice ──/voice/say──► 外部 TTS
                                          └─(可选) tts_command 直接起子进程
```

麦克风、唤醒词、ASR 模型全都依赖算力平台。平台已定（**树莓派 5**，无 CUDA），
引擎随之定为 openWakeWord + whisper.cpp/sherpa-onnx + Piper，全部 CPU 实时
（`docs/hardware_readiness.md` §8.5/8.6）。所以这里先把**不依赖硬件的那一半**
做成契约：真机上唯一要做的是「把 ASR 的输出灌进 `/voice/command`、让某个 TTS
引擎订阅 `/voice/say`」，本节点一行都不用改。

`/voice/command` 三种格式都收：

```json
{"text": "把灯打开", "intent": "light_on", "confidence": 0.92}
{"intent": "light_on"}
把灯打开
```

`intent` 字段优先：ASR/NLU 那边已经分好类了，就不该再用关键词猜一遍。

### 意图表与「宁可漏、不可错」

| 意图 | 关键词（节选） | 动作 |
|---|---|---|
| `clear_estop` | 解除急停 / 取消急停 / 解除刹车 | `/safety/estop=false` |
| `estop` | 急停 / 紧急停止 / 刹车 | `/safety/estop=true` |
| `stop_follow` | 别跟 / 不要跟 / 不要再跟 / 别跟着 / 停止跟随 | `follow_off` |
| `follow` | 过来 / 跟着我 / 跟我走 / 跟上 | `follow` |
| `stop` | 停下 / 停止 / 别动 / 站住 | `hold_motion` |
| `undock` / `dock` | 离桩 / 充电 / 回充 | `undock` / `dock` |
| `light_on` / `light_off` / `light_auto` | 开灯 / 关灯 / 灯光自动 | `/light/mode` |
| `patrol_on` / `patrol_off` | 开始巡逻 / 停止巡逻 | `patrol_on` / `patrol_off` |
| `status` | 状态 / 报告情况 | `status` |

三条硬规则：

1. **只做包含匹配**，不做模糊/拼音/编辑距离。远场噪声下那类匹配错得离谱。
2. **顺序即优先级**，而且「停」必须排在「跟」前面。「别跟着我」同时含「跟」
   和「别跟」，先匹配到 `follow` 的话，车会朝说话的人开过去 —— 这是这套逻辑里
   最危险的一条歧义。
3. **解锁排在加锁前面**。「解除急停」里含「急停」，顺序反了，唯一的解锁口令
   会被执行成「再加一次锁」，现场表现是喊多少遍都解不开。反方向不会误判
   （「急停」不含「解除急停」）。

这两条顺序错误都是被测试抓出来的，不是靠人肉审表 ——
`test_negative_phrase_does_not_mean_come` 和
`test_clear_estop_is_not_swallowed_by_estop`。

匹配不到就**什么都不做**，只记一条 info。绝不猜。

### TTS 侧

- 相同文本在 1.5s 内不重复播（告警状态机 1Hz 刷新，否则同一句话念 20 遍）。
- 文本截断 200 字符、去换行 —— 它最终会进子进程参数，长度必须封顶。
- `tts_command` 用 `shlex.split` 切分后直接 `exec`，**不走 `shell=True`**，
  避免语音文本里的引号变成命令。

## 五、软件安全闸

`twist_mux` 在原有的「优先级 + 超时」之上，加了两级闸和一条心跳：

| 机制 | 话题 | 语义 | 会过期吗 |
|---|---|---|---|
| 急停 | `/safety/estop` | `true` 闭锁，**只有显式 `false` 能解** | **不会** |
| 按住 | `/mission/hold_motion` | 调度层 1Hz 刷新，3s 不刷新自动放开 | 会 |
| 心跳 | `/safety/heartbeat` | 每个 tick 都发，给 MCU 看门狗 | — |

为什么「按住」要会过期：它的发布者是调度层，调度层崩了以后如果按住永久生效，
车就再也动不了了 —— 一次 OOM 变成永久瘫痪。而急停的发布者是人，人没说解除
就不能自己解除。

心跳存在的理由是 `docs/firmware_protocol.md` §1.2：`/cmd_vel` 断流本身是合法状态
（待命），不能拿它当「上位机死了」的判据，否则正常待命会反复触发保护。

### 急停是「状态」不是「事件」

`/safety/estop` 用 `lightdog_qos.SAFETY_QOS`（RELIABLE + TRANSIENT_LOCAL +
depth 1），不是默认的 VOLATILE。

这不是过度设计。实测过一次真实后果：按下急停后 `lightdog_mission` 由于改参数
被重启，它从 `estop=False` 初始化 —— 内存里「已经急停」这个事实跟着旧进程
一起没了。车还有 `twist_mux` 挡着，但调度层会以为自己可以跟随，于是放开
`/follow/enable`、去抢 Nav2 的目标。哪天 `twist_mux` 也重启一次，这洞就真漏了。

改成 `TRANSIENT_LOCAL(depth=1)` 后，发布端保留「最后一次」状态，后启动的
订阅者握手时直接收到它。**实测**（2026-11-02，容器 `sim`）：

```
$ bash sim/system.sh listen 急停           # 由常驻的 voice 节点发布
  mission: mode=ESTOP, hold=true, follow_enabled=false
  mux: estop

$ kill -9 <mission_pid> && 重新拉起 mission
  [WARN] 收到急停闭锁，调度层让路      ← 启动后 0.8s，没有任何人重发
  [INFO] 模式 IDLE -> ESTOP

$ bash sim/system.sh listen 解除急停
  mission: mode=FOLLOW（人还在，正常恢复跟随）
  mux: follow
```

同时把 `transient_local` 的兼容性也考虑进去了：发布端用 SAFETY_QOS、订阅端用
默认 VOLATILE 仍然**能收到**（只是拿不到历史），所以外部脚本不改也能用。

## 六、参数

`lightdog/config/system_params.yaml`：

| 参数 | 默认 | 说明 |
|---|---|---|
| `auto_mode` | `true` | `false` 时只响应显式命令，不自动跟随/不自动回充 |
| `follow_enabled` | `true` | 开机是否允许跟随 |
| `patrol_enabled` | `false` | 开机是否巡游 |
| `dock_approach` | `true` | 是否启用低电自动接近 |
| `approach_ratio` | `0.35` | 低电触发阈值 |
| `clear_ratio` | `0.45` | 迟滞解除阈值（必须 > `approach_ratio`） |
| `dock_pose_x/y/yaw` | `-0.50 / 0.00 / 0.00` | 充电桩在 map 里的位姿 |
| `staging_distance` | `1.00` | 导航到「桩前多远」后交给 dock_manager |
| `patrol_waypoints` | `''` | `"x,y,yaw; x,y,yaw"`，空 = 不巡游 |
| `person_timeout` | `2.0` | present 断流多久算没人 |
| `dock_state_timeout` | `5.0` | `/dock/state` 断流多久算没在回充 |
| `alert_timeout` | `5.0` | `/alert/state` 断流多久算没告警 |

`patrol_waypoints` 特意用**字符串**而不是数组参数：rclpy 里
`declare_parameter('x', [])` 之后再 `get_parameter` 会抛
`ParameterUninitializedException`（descriptor 声明的类型被忽略），
容器里实测过两种写法都绕不过去。字符串 + 自己解析虽然土，但它可测、可报错，
坏数据会被**计数并跳过**而不是猜一个默认值。

## 七、验收记录（2026-11-02，容器 `sim`）

前置：`docker compose up -d` → `bash sim/system.sh start` → `bash sim/p2.sh spawn`
→ `bash sim/p2.sh start`。

| 步骤 | 命令 | 实测结果 |
|---|---|---|
| 系统层起来 | `system.sh status` | `mode=IDLE`，`follow=false`，`mux=idle` |
| 人出现 | `system.sh status` | `mode=FOLLOW`，`follow=true`，`mux=follow`，亮度 0.85 |
| 手动关灯 | `system.sh light off` | `/led/brightness=0.0`（有人也不亮，覆盖优先）✅ |
| 语音开灯 | `system.sh listen 把灯打开` | 0.0 → 0.85 ✅ |
| 语音停跟随 | `system.sh listen 不要再跟着我了` | `mode=IDLE`，`reason=命令：停止跟随`，`mux=idle` ✅（关键：没有变成「过来」） |
| 语音急停 | `system.sh listen 急停` | `mode=ESTOP`，`hold=true`，`mux=estop` ✅ |
| 急停继承 | kill mission 后重启 | 0.8s 内 `收到急停闭锁` → `ESTOP` ✅ |
| 语音解锁 | `system.sh listen 解除急停` | `mode=FOLLOW` ✅（没被理解成再加锁） |
| CLI 闭环急停 | `system.sh estop` × 5 轮 | 5/5 确认（部分轮次需要重发 1~2 次）✅ |
| 离线回归 | `python lightdog/tests/run.py` | 0 失败（项数以脚本末尾打印为准；宿主机上会跳过需要 `/proc`、gcc、`firmware/` 的几组） |

## 八、已知限制

1. **`ros2 topic pub` 的短突发不可靠**，而且丢包是**按订阅者**发生的。
   实测 `-r 10 -t 6`（0.6s）连发 5 轮，`/safety/estop` 只到 1 轮；更糟的是出现过
   `twist_mux` 收到了、`lightdog_mission` 没收到 —— 一个锁上、一个还自以为能动。
   根因是 CLI 发布进程发完就退，RELIABLE 的样本还没被所有订阅者 ACK 就析构了。
   对策：`sim/system.sh` 改成「发 → 读回 → 不行再发」，并把确认标准定成
   **两个消费者都到位**才报成功。生产路径上的急停必须由常驻节点发。
2. **`/mission/hold_motion` 只有 3s 记忆**。调度层卡住超过 3s（比如 GC、CPU
   被占满）会让按住提前失效。急停不受影响，但「语音说停下」这种软停车不是硬保险。
3. **巡游没有避障策略**，只依赖 Nav2 自身的 `collision_monitor`。真机上建议在
   路点之间保持 `max_linear` ≤ 0.25m/s。
4. **模式仲裁对输入是「最后一条说了算」**，没有做多传感器融合。如果有人用
   `ros2 topic pub` 手动灌 `/dock/state=DOCKED`，调度层会信 —— 这是调试便利换来的
   代价，生产环境不要把 ROS 图暴露给不可信网络。

## 九、相关文档

| 想知道 | 看 |
|---|---|
| P1~P5 各自做了什么 | `docs/P1_led_battery.md` … `docs/P5_fall_alert.md` |
| 上真机要改哪些参数、失效怎么降级 | `docs/hardware_readiness.md` |
| MCU 看门狗、LED PWM、急停按钮 | `docs/firmware_protocol.md` |
