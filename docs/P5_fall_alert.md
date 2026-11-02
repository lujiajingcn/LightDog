# P5：跌倒检测 + 告警链路

> 阶段目标一句话：**识别到"人躺在地上起不来"，才叫救护**。
> 蹲下系鞋带、坐地上、弯腰捡东西，一律不打扰任何人。

---

## 一、目标

P1~P4 解决的是"机器人看得见、走得动、记得下、传得出"，被看护的人出事时仍然
只能靠人盯着屏幕。P5 要把这条链路补成闭环：

1. **看得懂姿态**：从 RGB-D 图像判断人是站着、躺着、还是看不见。
2. **分得清误报**：躺倒 ≠ 跌倒，必须有第二道确认。
3. **叫得动人**：确认无误后 —— 机器人语音问询、手机端全屏告警、按顺序联系
   紧急联系人、并**自动留下一段事发前后的视频**。

第 3 条里"留视频"这件事由 P4 完成，P5 只发一条消息。这个分工是刻意的，见 §3.3。

---

## 二、拓扑

```
        /camera/color/image_raw   /camera/depth/image_rect_raw   /camera/color/camera_info
                    │                          │                          │
                    └──────────────┬───────────┴──────────────────────────┘
                                   ▼
                    ┌──────────────────────────────┐
                    │  lightdog_fall_detector      │  感知层
                    │  颜色掩膜 → 反投影 → 离地高度 │
                    └──────────────┬───────────────┘
                       /fall/state │ /fall/z_p95 │ /fall/lying │ /fall/debug_image
                                   ▼
                    ┌──────────────────────────────┐
                    │  lightdog_alert_manager      │  决策层（两级确认状态机）
                    └──┬────────────┬───────────┬──┘
       /alert/state   │            │           │  /recorder/event
       /alert/voice   │            │           ▼
                       ▼            ▼      ┌────────────────────┐
              lightdog_server   (TTS)     │ lightdog_recorder  │ P4
              /api/v1/alarms               └─────────┬──────────┘
                    │                                ▼
                    ▼                          events/<id>/video.mp4
              手机全屏告警 ←──── /recorder/event 已经切好了视频
```

`/alert/command`（`ack` / `reset` / `test`）由后端从 HTTP 转成 ROS 消息回灌给
状态机 —— **状态机的唯一所有者只能是 `lightdog_alert_manager`**，
后端和手机端都只是它的遥控器，不各记一份状态。

---

## 三、四个关键设计

### 3.1 判据用「离地高度」，不用「宽高比」

最直观的写法是看包围盒：站着是竖条、躺着是横条。但宽高比会被距离毁掉 ——
人走到 0.8m 时相机只能看到躯干中段，bbox 接近正方形；蹲下系鞋带也是横的。
靠 bbox 做跌倒检测，误报率会高到没人愿意开这个功能。

这里用一个**物理量**：人体点云最高点离地多少米。

相机装在 `base_link` 前 0.065m、高 0.255m，`base_link` 离地 0.1195m，
所以光心离地 `0.3745m`，光轴水平朝前。已知每个像素的深度 `d`：

```
光学系 (x 右 / y 下 / z 前)：x = (u-cx)/fx*d ,  y = (v-cy)/fy*d ,  z = d
base_link (x 前 / y 左 / z 上)：x_b = z ,  y_b = -x ,  z_b = -y
离地高度：                        h = 0.3745 + z_b
```

代码里就一行：

```python
z_floor = self.cam_z_floor - (vs.astype(np.float32) - self.cy) / self.fy * d
z_p95 = float(np.percentile(z_floor, 95))
```

取 **95 分位**而不是最大值：深度图边缘有飞点，`max` 会被一两个坏点顶到天上。

阈值 `lying_top_max: 0.50` / `stand_top_min: 0.68` **不是拍脑袋划的**，
它们落在一片真的空档里：

| 姿态 | 橙色躯干（0.72~1.47m）在画面上的可见范围 | 实测 `z_p95` |
|---|---|---|
| 站立 | 躯干从 0.72 起向上铺开 | **1.435 m** |
| 蹲着 | 只露上沿，落在两阈值之间 | → `unknown`，**不报警** |
| 躺倒 | 整根压在 0~0.36 | **0.340 m** |

`unknown` 这个状态是刻意留的：蹲下、坐在矮凳上、画面被家具挡掉一半，
都落在这里。它对状态机的含义是"非躺倒"，也就是**不推进告警**。

颜色判据（橙色上衣）整套沿用 P2 检测器，但**去掉** `aspect_min` 约束 ——
P2 那个约束是"只认竖着的人"（为了跟随不跟错目标），而 P5 恰恰要能看见躺着的人。
两个节点订阅同一路图像，互不影响。

### 3.2 两级确认：姿态持续性 + 交互无响应

「检测到人躺在地上」和「人摔倒了需要叫救护车」之间差着十万八千里：
蹲下系鞋带、坐地上看电视、弯腰捡东西、趴着擦地，视觉上全是"很低的一团"。
只靠一级判定就打电话，一周能误报二十次，家人第一时间就会把设备关掉。

```
NORMAL ──(躺倒)──> SUSPECTED ──(持续 suspect_hold=5s)──> CONFIRMING
                       │                                    │
                       │(站起)                    ┌─────────┴─────────┐
                       ↓                          │(站起)             │(超时仍躺倒 15s)
                    NORMAL <──────────────────────┘                   ↓
                       ↑                                            ALARM ──(ack)──> ACKED
                       │                                              │                 │
                       └──────(站起持续 recover_hold=3s)──── RESOLVED <┴─────────────────┘
```

- **一级（时间）**：连续躺倒 5 秒。排除"弯腰一下又起来"。
- **二级（交互）**：机器人语音问「您还好吗？需要帮忙请回答我。」，
  再等 15 秒。这段时间里只要站起来就取消；一直躺着才报警。

二级的价值在于它引入了**时间**这个维度：真正的跌倒者往往起不来，
而误报者一定会起来。这是纯视觉判据永远拿不到的信息。

`lying_gap: 1.0` 是判据抖动的容忍窗口：1 秒内还看到过 `lying` 就仍然算躺着。
否则人躺着时翻个身、手挡住上衣，颜色掩膜断一帧就会把计时器清零。

### 3.3 视频切片归 P4，P5 只发一条消息

`alert_manager` 告警时只做一件事：

```python
req = {'type': 'fall', 'label': '检测到跌倒',
       'pre': 25.0, 'post': 10.0,
       'note': 'alert_manager:%s' % self.alarm_id}
self.pub_event.publish(String(data=json.dumps(req, ensure_ascii=False)))
```

剩下的（从环形缓冲里挑分段、解码、按时间裁、重编码、写 thumb）全在
`lightdog_recorder` 里。这样 P5 不需要知道任何视频细节，P4 也不依赖 P5 存在
—— 换掉任何一个都不会连带崩。

`note` 这个字段是两边唯一的耦合点，也是**历史记录能找回自己那段视频**的关键：
后端 `event_of_alarm()` 拿 `note == 'alert_manager:<alarm_id>'` 精确匹配。
不能只按时间猜 —— 那样翻旧记录会串到后面那次告警的视频上去（踩过，见 §6.6）。

`pre` 为什么是 25 而不是 15：告警是在「躺倒 5 秒 + 问询 15 秒」之后才触发的，
`pre=15` 的视频起点正好卡在问询开始那一刻，人已经躺平了，
**最关键的"怎么倒的"反而没拍到**（踩过，见 §6.1）。

### 3.4 「打电话」：自建 SIP

选型已定（D2）：**自建 SIP**。Asterisk 跑在本机，`lightdog_call.SipDialer` 通过
AMI 让 Asterisk 外呼家属分机，接听后播放预录的告警语音。

```
alert_manager ──AMI 5038(明文,只绑 127.0.0.1)──> Asterisk ──SIP──> 家属手机
```

三种模式（`call_mode`，默认 `auto`）：

| 模式 | 触发条件 | 能知道什么 |
|---|---|---|
| `sip` | `sip_ami_secret` 非空 | 接通 / 无人接听 / 忙 / 拥塞 / 失败（`OriginateResponse`） |
| `command` | `call_command` 非空（兜底通道，外部语音 API） | 只知道命令是否成功退出 → 记 `placed`「已发出外呼」 |
| `log` | 都没有（仿真默认） | 只写记录，不拨号 |

`auto` 是为了**同一份 yaml 同时用于仿真和真机**：真机填上 secret 就自动走 SIP，
仿真里什么都不配就是 log。

四条设计约束，每条都对应一个会真的发生的故障：

1. **拨号不能在 `_tick` 里同步做。** 原实现是 `subprocess.run(timeout=15)` 直接
   在 5Hz 的 tick 里跑：一通电话把状态机卡住十几秒，这段时间里人自己站起来了
   也不会进 `RESOLVED`，事件视频窗口跟着错位。现在拨号在后台线程。
2. **「命令返回 0」≠「有人接听」。** 走 AMI 才拿得到 `Reason`（0/4 接通、
   3 无人接听、5 忙、8 拥塞）。把「没人接」记成「外呼失败」，事后就分不清
   「那晚家里没人」和「那晚 SIP 挂了」。
3. **`ack` 必须挂断正在响的那一通。** 只做「不再拨下一个」的话，家属手机要响满
   `sip_ring_timeout`（45s），而界面写着「已确认，不再重复呼叫」。挂断要按频道名
   前缀找（`PJSIP/1001-0000000a` 的流水号是 Asterisk 分的）。
4. **`test` 告警永不外呼。** 半夜把女儿的手机打醒一次，这功能就再没人敢点了。

部署与验收（Asterisk 配置、密码、语音提示生成、四种结果的现场实测）见
**[`deploy/sip/README.md`](../deploy/sip/README.md)**。

仍然成立的一条边界：**手机 App 推送才是最现实的通知通道，电话是兜底**。
外呼失败不影响 `/alert/state`、事件视频和手机端弹窗。

---

## 四、交付物

| 文件 | 作用 |
|---|---|
| `lightdog/lightdog_fall_detector.py` | 感知层：RGB-D → `/fall/state` |
| `lightdog/lightdog_alert_manager.py` | 决策层：两级确认状态机 |
| `lightdog/config/p5_params.yaml` | 全部阈值与时间参数（含每条的理由） |
| `lightdog/launch/lightdog_p5.launch.py` | 起上面两个节点 |
| `lightdog/web/{index.html,style.css,app.js}` | 全屏告警层 + 告警记录 + 蜂鸣 |
| `lightdog/lightdog_server.py` | `/api/v1/alarms`、ack/test/reset、`alarms.jsonl` |
| `sim/p5.sh` | 宿主机助手：摆人 / 触发 / 确认 / 诊断 / 自检 |
| `sim/entrypoint.sh` | `ENABLE_P5=1` 时随容器自动拉起 |

### 接口

| 方法 | 路径 | 说明 |
|---|---|---|
| GET | `/api/v1/alarms` | `{alarms: [...], current: {...}}`，按 `alarm_id` 折叠 |
| POST | `/api/v1/alarms/ack` | 确认已知晓，停止重复呼叫 → `ACKED` |
| POST | `/api/v1/alarms/test` | 模拟一次告警（**不拨号、不自动解除**） |
| POST | `/api/v1/alarms/reset` | 人工复位 → `NORMAL` |
| GET | `/api/v1/status` | `status.alert` 里带完整告警状态 + `event_id` |

`/api/v1/alarms` 返回的 `current` 与 `/status` 的 `alert` 走**同一个**
`current_alert()` 方法 —— 两个接口各算一份的话，手机端轮询 `/status`
拿到了 `event_id`、切到事件页却发现 `current` 里没有，这种
"同一个东西两副面孔"的 bug 最难查。

### 话题

| 话题 | 类型 | 方向 |
|---|---|---|
| `/fall/state` | `String` | `standing` / `lying` / `absent` / `unknown` |
| `/fall/z_p95` | `Float32` | 人体点云离地高度 95 分位（米） |
| `/fall/lying` | `Bool` | 兼容用布尔量 |
| `/fall/debug_image` | `Image` | 掩膜 + 姿态 + 数值标注 |
| `/alert/state` | `String` | JSON，手机端直接显示 |
| `/alert/voice` | `String` | 待说的话（真机接 TTS） |
| `/alert/command` | `String` | `ack` / `reset` / `test` |

---

## 五、实测验收

环境：`linorobot2:gazebo`（ROS 2 Jazzy + Gazebo Harmonic），软件渲染，
实时因子约 **0.26x**（下文标注"≈仿真秒"的就是按这个折算的）。

摆人用 `gz service -s /world/<world>/set_pose`：`person` 是 `static` 模型，
不参与动力学，可以直接搬。绕自身 +X 轴转 90°（四元数 `x=w=0.7071`）：
local `(x,y,z) → world (x,-z,y)`，整条身体沿世界 `-y` 横躺，
中轴高度 = `position.z`。取 `z=0.18`（≈躯干半径），
再把原点沿 `+y` 平移 1.095m，让橙色躯干正中目标点。

### 5.1 姿态判据

```
=== 站立 ===
  z_p95 = 1.435    span = 0.69 m   → standing

=== 躺倒 ===
  z_p95 = 0.340    span = 0.30 m   → lying
```

`/fall/debug_image`（`bash sim/p5.sh debug`）核对：站立时红色掩膜完整覆盖
竖直躯干，躺倒时覆盖横躺的整根躯干，画面下沿没有把人切掉。
2.0m 距离是有讲究的 —— 相机光心离地 0.3745m、垂直半视场角约 23.4°，
2.0m 处能看到的最低高度约 -0.49m；人躺下后整体压在 0~0.36m，
**再近就会掉出画面下沿**（1.2m 时直接看不见）。

### 5.2 状态机时间线（真实日志）

```
1793545491.9  状态 NORMAL -> SUSPECTED（检测到躺倒姿态，开始计时）
1793545511.2  状态 SUSPECTED -> CONFIRMING（连续躺倒 5s）        +19.3s 墙钟 ≈ 5 仿真秒
1793545511.2  [语音] 您还好吗？需要帮忙请回答我。
1793545570.4  状态 CONFIRMING -> ALARM（躺倒 5s 且问询后 15s 无起身）  +59.3s 墙钟 ≈ 15 仿真秒
1793545570.4  已请求事件切片：{"type":"fall","label":"检测到跌倒","pre":25.0,"post":10.0,
                              "note":"alert_manager:alm_20261101_150610"}
1793545570.5  [语音] 检测到您可能摔倒了，正在联系家人。
1793545571.3  [语音] 正在联系女儿。
1793545590.2  状态 ALARM -> ACKED（有人确认已知晓，停止继续呼叫）
1793545613.8  状态 ACKED -> RESOLVED（人已起身，告警解除）
1793545634.3  状态 RESOLVED -> NORMAL（告警解除后回到正常）
```

从手机端点「我已知晓」到状态变 `ACKED` 用了约 4 秒（HTTP → ROS 话题 → 状态机
→ 状态回传 → 前端轮询），人眼完全无感。

`ACKED` 之后 `RESOLVED` 没来之前**不会**再拨号 —— `_maybe_call()` 第一行就
`if self.state == self.ACKED: return`。这是"确认"这个动作的全部意义。

### 5.3 事件切片（含事发前）

```
id       evt_20261101_150610_fall
note     alert_manager:alm_20261101_150610
t_from   320.6   t_ev 345.6   t_to 355.6      span 35.0 s（= pre 25 + post 10）
frames   441     fps 12.622   bytes 43207     codec avc1
clips    seg_000083 … seg_000087（5 段环形缓冲拼成）
ffprobe  h264 640x480 441 帧 34.939s
```

抽帧核对：**t=1s 人还站着，t=12s 已经躺平** —— 事发全过程都在。
（第一版 `pre=15` 时 t=2s 就已经躺着了，这是 §6.1 的直接证据。）

### 5.4 HTTP 接口自检（`bash sim/p5.sh probe`）

```
/api/v1/alarms          200 application/json; charset=utf-8  2160B
/api/v1/status          200 application/json; charset=utf-8   963B
POST /alarms/bogus      400   (期望 400)
```

最后一条验的是路由前缀没有把 `/alarms/xxx` 当通配吞掉 ——
否则手机端一个拼错的按钮会静默成功，比报错难查得多。

### 5.5 手机页面（无头 Edge 真实渲染，500×1000）

三种状态都截图核对：

- **告警中**：整屏红色（`animation: alarm-pulse` 呼吸），标题`检测到跌倒`，
  副标题`正在按顺序联系 2 位紧急联系人`，
  kv 表 `发生时间 22:55:43 / 已持续 1分42秒 / 联系人 2 位 / 呼叫进度 1/3 轮 / 人体高度 0.34 m`，
  按钮`我已知晓`（红底、`order:-1` 排在最上面，拇指好按）+`看现场视频`，
  脚注`本机未配置外呼命令，仅记录并通知 App`
- **已确认**：整屏转绿、停止脉冲，标题`告警已确认`，
  副标题`已收到确认，正在等人员起身`，按钮变`已确认`且禁用
- **模拟告警**：标题`模拟告警`，副标题`这是一次测试，不会打扰任何联系人`，
  `联系人 未设置 / 呼叫进度 0/3 轮`，脚注
  `测试告警：不拨号、不自动解除，点「我已知晓」或在设置里复位结束`

事件页的「告警记录」区块：`跌倒告警 / 22:55:43 · 已呼叫 2 次 / 已解除`（灰标签），
点一下直接打开那次告警的视频。

### 5.6 告警落盘与折叠

`/data/alarms.jsonl` 只在 `(alarm_id, state)` 变化时落一行 ——
状态机是 1Hz 周期上报的，照抄会把文件写爆。

```
{"state":"ALARM",   "alarm_id":"alm_20261101_150610", ...}
{"state":"ACKED",   "alarm_id":"alm_20261101_150610", ...}
{"state":"RESOLVED","alarm_id":"alm_20261101_150610", ...}
{"state":"NORMAL",  "alarm_id":"alm_20261101_150610", ...}
```

`/api/v1/alarms` 按 `alarm_id` 折叠成"每条告警的最终状态"，最新的在前。

`NORMAL` 也要落一行，是因为**人工 reset 是从 ALARM 直接跳 NORMAL 的**
（不经过 RESOLVED）。不记的话，这条告警在历史里会永远停在"告警中"（踩过，见 §6.6）。

### 5.7 端到端复验（2026-11-02，容器 sim）

两个脚本，都在容器内跑：

```
# 主链路：站立 -> 躺倒 -> SUSPECTED -> CONFIRMING -> ALARM -> 起身 -> NORMAL
docker exec sim bash -lc 'source /opt/ros/jazzy/setup.bash; \
  source /home/ros/linorobot2_ws/install/setup.bash; \
  python3 /home/ros/linorobot2_ws/src/lightdog/tests/verify_p5.py --ros-args -p use_sim_time:=true'

# 补充：误报抑制 + 呼叫/ack
python3 .../tests/verify_p5_extra.py --ros-args -p use_sim_time:=true
```

「躺倒」用 `gz service set_pose` 给 person 加 `roll=90°` 并把 z 抬到 0.17m
（躯干半径，不抬会有一半埋进地面）。实测时间线：

```
fall=standing z_p95=1.14  alert=NORMAL
    +0.6s sim   fall=lying    z_p95=0.33  alert=SUSPECTED
    +5.5s sim   fall=lying    z_p95=0.33  alert=CONFIRMING   ← 一级确认 4.9s
   +20.4s sim   fall=lying    z_p95=0.33  alert=ALARM        ← 二级确认 14.9s
```

- 姿态判据：站 1.14m / 躺 0.33m，阈值 0.50 与 0.68 两侧余量都很大。
- 告警事件：`pre=25 / post=10 / duration=35.0`，**935 帧**，78KB。
  抽首帧 = 人站着，尾帧 = 人躺在地上 —— 这正是 `event_pre: 25` 想买的东西。
- 手机端：`/status.alert` 直接给出 `ALARM + alarm_id + contact_count=2 +
  call_attempts[{张阿姨, dialed:false}]`；`/alarms` 历史同步。
- 起身后 3.6s sim 进 RESOLVED、8.6s sim 回 NORMAL（`resolved_hold`）。

补充脚本验的两条：

| 场景 | 结果 |
|---|---|
| 躺 3s 就起身（弯腰/蹲下）| 只到 `SUSPECTED`，起身后回 `NORMAL`，**没进 CONFIRMING/ALARM** |
| 真实跌倒 -> 拨打 -> ack | 第 1 次呼叫写入 `call_attempts`；ack 后 70s（>1 个 `call_interval`）不再新增 |

注意所有时间都是**仿真秒**：本机 RTF≈0.42，`confirm_window: 15` 在墙钟上是 35 秒左右，
用秒表对比会以为状态机慢了 2.4 倍。

---

## 六、踩的坑

1. **`event_pre: 15` 的视频里人已经躺平了**

   告警在「躺倒 5s + 问询 15s」之后才触发，`pre=15` 的视频起点正好卡在
   问询开始那一刻。抽首帧一看，人早就躺下了 —— 最需要的"怎么倒的"没拍到。
   改成 `pre=25`（= 5+15+5），复测 t=1s 站着、t=12s 躺下，全过程完整。

2. **`sim/p5.sh` 里一句注释把整条命令跑飞了**

   抓调试图的注释写成 ``# 不能用 `ros2 topic echo` 抓这张图``，而这个 heredoc
   是 `<<EOS`（**不加引号**，因为要展开 `$OUT`）。反引号在未加引号的 heredoc 里
   就是命令替换 → 宿主 shell 当场执行了一次 `ros2 topic echo`，
   报 `sim/p5.sh: line 374: ros2: command not found`。
   注释里不能出现反引号、`$`、`\`；要么改成加引号的 heredoc，要么别用它们。

3. **`ros2 topic echo` 抓不了图像**

   `sensor_msgs/Image` 的 `uint8[]` 在 YAML 里是**一个字节一行**
   （640×480×3 = 921600 行）。`timeout 20` 只打印到第 933 个字节，
   解析器拿到的是半张图。改成容器内用 `rclpy` 订阅一次再 `cv2.imwrite`。

4. **`LaunchConfiguration` 喂给 bool 参数会让节点起不来**

   `{'auto_reset': LaunchConfiguration('auto_reset')}` 求值出来是字符串
   `'true'`，rclpy 判类型不符直接抛异常 —— **不是静默转 false，是节点根本不起**。
   必须 `ParameterValue(LaunchConfiguration(...), value_type=bool)`。
   （字符串参数 `contacts_path` / `call_command` 没这个问题，
   所以空串 `call_command:=` 依然安全。）

5. **`test` 告警被人站着时的 auto_reset 秒解除**

   站在机器人面前点「模拟一次告警」，弹窗 3 秒后自己消失，界面长什么样根本没看清。
   于是给测试告警加了两个特例：**不跟随姿态自动解除**，**绝不真拨号**
   （半夜把女儿手机打醒一次，这个功能就再没人敢点了）。
   `is_test` 一路带到 `/alert/state`，前端据此换文案。

6. **历史告警拿不到视频 / reset 之后永远停在"告警中"**

   两件事，同一个根因：状态折叠得太粗糙。

   - 原来只对 `ALARM/ACKED` 的条目按 `newest_event_since(wall)` 找视频，
     于是"已解除"的历史告警全都点不开；而且按时间找会串到后面那次告警上。
     → 改用 `note == alert_manager:<alarm_id>` **精确匹配**。
   - `reset` 从 `ALARM` 直接跳 `NORMAL`，不落盘，折叠后停在 `ALARM`。
     → 把 `NORMAL` 也纳入落盘状态，并且前端把 `NORMAL` 显示成"已解除"。

7. **`已持续` 显示成"告警已确认，已持续 14 秒"**

   直接用了状态机里的 `age`，而 `age` 是"进入**当前状态**后过了多久"，
   进 `ACKED` 会归零。前端改成按**墙钟**从 `alarm_wall` 算到 `Date.now()`，
   并兜一层异常值（手机和机器人没对时的话，差值可能是负的或好几天）。

8. **`os.path.realpath` / `pkill -f` 这类"通用做法"在这个容器里都不成立**

   - 判路径穿越不能用 `realpath`：colcon `install(DIRECTORY)` 是**逐文件符号链接**，
     正常静态文件全被判成越界。要用**词法** `normpath`。
   - `pkill -f` 在容器里不生效（P3 就踩过），必须按 PID 杀。
   - 这两条具体在 P3/P4 文档里，P5 的 `p5.sh` 沿用了同样的规避写法。

---

## 七、已知限制

1. **外呼没有真的实现**。`call_command` 默认空 = 只记录不拨号。
   真机要接 SIP 或运营商 API；模板机制已经把接口留好了。
2. **只认"橙色上衣"**。颜色判据沿用 P2 的仿真模型。真机必须换成
   人体检测（YOLO / MediaPipe Pose）—— 判据（离地高度）可以整套保留，
   只是把"橙色掩膜"换成"人体分割掩膜"。
3. **单相机、正前方 ±35°**。人在机器人背后摔倒看不见。
   真机需要多机位或机器人巡游视角。
4. **没有"缓慢下滑"的识别**。从沙发滑到地上、从轮椅滑落这类过程，
   姿态是渐变的，`z_p95` 会在阈值之间来回穿；当前只能靠 `lying_gap`
   的容忍窗口撑住，表现是告警延迟变大而不是漏报。
5. **`test` 告警也会切一段视频**（方便验收），真机上可以考虑关掉以省空间。
6. **仿真时间与墙钟的比例**在软件渲染下约 0.26x，所以"5 秒 → 告警"墙上要等
   约 20 秒。真机上两者一致，不需要改参数。

7. **传感器失效时的语义必须分清 `unknown` 和 `absent`**。这是本轮复查补的，
   也是上真机最容易踩的一条：

   | 情况 | `lightdog_fall_detector` 发什么 | 告警状态机怎么处理 |
   |---|---|---|
   | 相机断流 / 尺寸不一致 / 内参缺失 | `unknown`（且**不刷新** `last_seen`） | 不改任何姿态时间戳，SUSPECTED 保持、CONFIRMING 继续倒计时（fail-safe：宁报不误复位） |
   | 相机正常、画面里确实没人 | `absent` | 连续 `absent_grace`(10s) 才判「人已离开」并复位到 NORMAL |

   改之前没有 `unknown` 这一档：一次遮挡（有人从镜头前走过、被子盖住）会被当成
   「人站起来了」，把已经开始的确认流程直接抹掉。反过来，如果为了省事把
   `unknown` 也当 `absent`，相机一坏就会静默复位 —— 两种都是真机上会出人命的错。

   参数：`frame_timeout` 1.5s、`depth_timeout` 1.0s（都是**墙钟**，与仿真跑多快
   无关）、`absent_grace` 10s。

---

## 八、与其它阶段的关系

| 阶段 | 关系 |
|---|---|
| P1 | 告警时的"呼吸灯 / 警示灯"可以直接复用 P1 的 LED 通道（尚未接） |
| P2 | 颜色判据整套复用；`person` 模型直接复用；两个检测器订阅同一路图像互不影响 |
| P3 | 无关。跌倒告警不触发回充，回充也不影响检测 |
| P4 | **强依赖**：告警视频由 `lightdog_recorder` 切，`/api/v1/alarms` 由 `lightdog_server` 提供；但 P4 不依赖 P5，缺了 P5 一切照常 |
| 真机 | 需要换掉：颜色掩膜 → 人体分割；外呼走 SIP（`deploy/sip/`）；`/alert/voice` → TTS 引擎 |

---

## 附：常用命令

```bash
bash sim/p5.sh start                  # 拉起检测 + 状态机（entrypoint 已自动起）
bash sim/p5.sh spawn && bash sim/p5.sh stand
bash sim/p5.sh fall                   # 放倒 → 等着看状态爬台阶
bash sim/p5.sh status                 # 位姿 / z_p95 / 告警状态 / 告警记录
bash sim/p5.sh z                      # 只看判据物理量
bash sim/p5.sh debug                  # 存一张带标注的调试图到 data/fall_debug.png
bash sim/p5.sh test | ack | reset     # 绕过两级确认 / 确认 / 复位
bash sim/p5.sh probe                  # 告警 HTTP 接口自检
bash sim/p5.sh log                    # 状态机日志
```

## 与系统层的衔接（后补）

P5 原来只发 `/alert/state` 和 `/alert/voice` 两个话题，**没有任何人消费**：
Car 该做什么、该说什么，都不归它管。现在接上了：

| 话题 | 消费者 | 行为 |
|---|---|---|
| `/alert/state` | `lightdog_mission` | `CONFIRMING / ALARM / ACKED` 期间 → `mode=ALERT_HOLD`：收回跟随、**按住轮子**、取消导航目标（人可能正躺在这条路径上）。5s 收不到状态就自动放开，避免告警节点崩了把车锁死 |
| `/alert/voice` | `lightdog_voice` | 转发到 `/voice/say`；`tts_command` 非空时直接起 TTS 子进程。1.5s 内同一句不重复播（告警状态机 1Hz 刷新，否则会念 20 遍） |

注意「按住」和「急停」是两条通道：按住会过期（3s），急停不会。
告警属于「按一下等确认」，不属于「人按了刹车」，所以用前者 ——
具体取舍见 [P6_system_layer.md](P6_system_layer.md) §五。

> 上真机前请先过一遍 **[hardware_readiness.md](hardware_readiness.md)**（参数清单 / 失效矩阵 / 真机上不能跑的节点）。
