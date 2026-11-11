# 硬件实施前必须知道的事

这份文档不介绍功能，只回答一个问题：**这套代码搬到真车上，会在哪里咬人。**

> **车已到货。** 实际是 RASPBOT V2（Pi 5 16GB / 四麦轮 / USB 单目广角相机 + 可动云台 /
> Pi 直驱无 STM32 / Raspberry Pi OS + ROS2 Humble），与本文 §〇 的 D1（Pi 5 8GB +
> Ubuntu 24.04 + Jazzy）、D3（MCU = STM32）不一致。本文下面各条**通用坑依然有效**，
> 但选型与配置要以 [`hardware_arrival.md`](hardware_arrival.md) 的实车对照为准。
> 特别注意：`deploy/install.sh` 的发行版探测（第 90 行）本来就包含 `humble`，
> 它没有 Ubuntu 版本号硬检查；真正要拍板的是 §四 的系统路线。
> **软件侧已按实车改造完**（新驱动节点、真机配置集、AprilTag/云台适配），
> 详见 [`hardware_arrival.md`](hardware_arrival.md) §八；本文 §一 的参数清单仍要
> 对照那份文件里的 `real_*.yaml` 去看，别再照仿真配置改。

要采购什么、买几个、为什么必须是那个规格，见 [`bom.md`](bom.md) —— 两份文档
是一对：bom 说"买这个"，这里说"买回来之后会在哪咬你"。

仿真里所有传感器都是「永远有数据、时间永远在走、电量永远听话」的，所以下面的
问题在 Gazebo 里一条都测不出来。每条都对应代码里的一个判据或一个参数，改之前
请先看这一页。

---

## 〇、选型已定（D1–D4）

这四项原先在所有文档里都是「待定」，定了之后很多判断才有意义，所以先摆在这里。
具体落地约束见 §八。

| # | 项 | 决定 | 直接后果 |
|---|---|---|---|
| D1 | 算力平台 | **树莓派 5**（8GB） | ARM64 Ubuntu 24.04 + ROS 2 Jazzy。**没有 CUDA**：YOLO 只能走 CPU（onnxruntime/NCNN 320）或加 Hailo-8L；供电与散热从此是故障源而不是配置项 |
| D2 | 通话方案 | **自建 SIP**（Asterisk） | 走 AMI 外呼家属分机，能区分「接通/无人接听/忙」。部署见 `deploy/sip/README.md`，代码见 `lightdog/lightdog_call.py` |
| D3 | MCU | **STM32** | 串口帧格式定稿（`docs/firmware_protocol.md` §六）；Pi 不再直接驱动电机/LED，GPIO 兼容性问题一次性消失 |
| D4 | 摄像头权限 | **保持现状** | 静态页不鉴权、`/api/*` 全部要 `auth_token`；不引入账号体系。所以 `auth_token` 从「建议改」升级为「不改等于裸奔」 |

> D4「保持现状」的含义要说清楚：现在的模型是**单令牌**，同一个 WiFi 下的人只要
> 拿到 token 就能看画面、翻录像、读联系人、发急停。它适合「家里一个可信局域网」，
> 不适合出租屋或多户共用网络。真到那种场景要加的是「每设备令牌 + 只读令牌」，
> 不是把页面藏起来。

---

## 一、上真机前必须改的参数

| 文件 | 参数 | 仿真值 | 真机值 | 不改会怎样 |
|---|---|---|---|---|
| 全部 `*_params.yaml` | `use_sim_time` | `true` | **`false`** | 没有 `/clock` 时 ROS 时钟停走，所有基于 ROS 时钟的逻辑永久失效（本仓库已把超时类判据全部改成墙钟，但依赖 `header.stamp` 的地方仍会错） |
| `p3_params.yaml` | `sim_charging` | `true` | **`false`** | 节点会自己往电量里灌值，永远显示在充电，`DOCKED → UNDOCKED` 的判断全部失真。代码里已加兜底：`use_sim_time=false` 时强制关闭并报 error，但**仍然应该显式改成 false** |
| `p4_params.yaml` | `auth_token` | `''` | **一个随机串** | 空 = 不鉴权。同一个 WiFi 下任何人都能看摄像头、翻录像、读联系人 |
| `p2_params.yaml` | `expected_frame` | `base_link` | 按实际检测器输出的坐标系 | 不一致时跟随器会拒绝坐标并停车（这是刻意的），日志里有 error |
| `p3_params.yaml` | `marker_length` | `0.084` | **用卡尺量黑框边长** | 距离按比例整体缩放。0.084 填成 0.10 会让距离多算 19%，表现为提前停或一直往桩里顶 |
| `p3_params.yaml` | `max_undocking_time` | `30.0` | 桩前净空不同需实测 | 太短 → 返回 905 `FAILED_TO_CONTROL`；太长 → 卡住时干等 |
| `p4_params.yaml` | `event_retain_days` / `max_event_mb` | `30` / `512` | 按存储卡容量 | 不设保留策略时 `events/` 会一直涨到写满分区，而告警录像恰恰在半夜写 |
| `p5_params.yaml` | `sip_ami_secret` | `''` | **与 `manager.conf` 一致的随机串** | 空 = 不外呼（`call_mode=auto` 退化成仅记录）。告警照常弹窗、事件视频照常切，只是**一个电话都不会打**，日志里只有一行「仅记录」 |
| `p5_params.yaml` | `sip_channel_template` | `PJSIP/{number}` | 按中继/分机改 | 填错的表现是 `channel not found`：AMI 通了、电话没打出去 |
| `p2_params.yaml` | `backend` | `color` | **`onnx`**（装好 onnxruntime 后） | `color` 只认「高饱和橙色上衣」，真人穿什么都有，见 §四.1。**别用 `yolo`**：Pi 5 无 CUDA，PyTorch CPU 只有 1~2 FPS，见 §8.5 |
| `p4_params.yaml` | `ssl_certfile` / `ssl_keyfile` | `''` | 自签或真证书路径 | 留空 = 全明文。局域网可用，但只要经过公共 WiFi 就必须开。**证书读失败时服务会拒绝启动而不是退回明文**（这是刻意的） |
| `p4_params.yaml` | `faststart` | `false` | `true`（需 ffmpeg） | 视频的 `moov` 在文件尾，手机端边下边播要等整段下完。真机上 ffmpeg 一般都有，开了只是多一次 `-c copy` 重封装 |
| `system_params.yaml` | `dock_pose_x/y/yaw` | `-0.50/0.00/0.00` | **按现场测** | 低电回充的导航目标是「桩前 1m」，桩位填错会导航到墙上。填错不会崩，只会永远差那么一点 |
| `system_params.yaml` | `patrol_waypoints` | `''` | `"x,y,yaw; x,y,yaw"` | 空 = 不巡游。格式必须用分号分隔、逗号分字段，坏数据会被**计数并跳过**而不是猜一个默认值 |
| `mcu_params.yaml` | `port` / `baud` | `/dev/ttyAMA0` / `115200` | 按实际接线与固件改 | 打不开串口时车**完全不动**，日志里只有一行「串口打不开」。用 USB 转 TTL 时写 `/dev/serial/by-id/...`，别写 `/dev/ttyUSB0`（重启后编号会变） |
| `mcu_params.yaml` | `cmd_hold` | `0.30` | 与 `twist_mux` 的 `follow_hold` 一致 | 太长：跟随节点让出控制权后车还带着最后一条速度走；太短：正常指令被切成一顿一顿 |
| `mcu_params.yaml` | `cmd_vel_topic` | `/cmd_vel_muxed` | **二选一，见右栏** | 真机上它是唯一到固件的一跳，订阅谁就等于把谁接到车轮上。`/cmd_vel_muxed` 在 `collision_monitor` **之前**；仿真里速度出口是 `/cmd_vel`（后面还有 `ros_gz_bridge → gz`），真机没有那一跳，于是 Nav2 的碰撞保护退化成一个只往死话题发零速的节点 —— **前方有障碍也拦不住车**。改成 `/cmd_vel` 能拿回碰撞保护，但要求 Nav2 常驻，否则车不会动 |
| `p1` 灯组 | 见 `docs/P1_led_battery.md` | — | — | LED 真机由固件驱动，`lightdog_led` 要停掉 |

## 二、真机上不要启动的节点

| 节点 | 原因 |
|---|---|
| `lightdog_led` | 它把亮度翻译成 **Gazebo 材质自发光**。真机 LED 由固件订阅 `/led/brightness` 驱动，这个节点只会往一个没人转发的话题发消息，现象是「灯不亮也没有任何报错」。代码里检测到 `use_sim_time=false` 会打印一条 error 提醒 |
| `lightdog_person_sim` | 它用 `gz service set_pose` 让仿真里的「人」走起来 |
| `lightdog_server` 的静态页 | 页面本身不用鉴权（否则手机第一次打开就是空白），但 `/api/*` 全部要 token。见下表 |

反过来，真机上**必须多起**两个仿真里可选、真机里关键的东西：

| 节点/机制 | 为什么真机必须有 |
|---|---|
| `lightdog_mission` | 仿真里可以只跑 P2/P3 手动验证，真机上没人负责「跟随器别压死 Nav2」「低电自己回桩」。**没有它，跟随和回充会互相抢方向盘** |
| `lightdog_mcu_bridge` | 速度与心跳出树莓派的**唯一一跳**。仿真里没有 STM32 所以默认不启用；真车忘了 enable 的表现是「车完全不动，日志里只有串口打不开」，很容易被误判成底盘坏了 |
| MCU 侧 300ms 断 PWM 看门狗 | 只有它能保证「上位机进程死了车也会停」。ROS 侧再怎么努力都做不到这一点，见 `docs/firmware_protocol.md` §二 |

## 三、传感器/上游失效时的行为（fail-safe 方向）

这是本轮复查的重点。原则是：**任何一条输入消失，系统都必须往「不动、不灭灯、
不误复位」的方向倒。**

| 失效 | 判据 | 参数（默认） | 行为 | 日志关键字 |
|---|---|---|---|---|
| 相机冻帧/掉线（P2 检测） | 墙钟无新帧 | `frame_timeout` (1.5s) | 立即报 `present=false` 并**停止发位置** | `没有新帧` |
| 深度图掉了、彩色还在 | 深度到达时刻 | `depth_timeout` (1.0s) | 退回「躯干高度/像素高度」粗估距离 | — |
| 相机冻帧/掉线（P5 跌倒） | 墙钟无新帧 | `frame_timeout` (1.5s) | 发 `unknown`，**不刷新** `last_seen`，不把「看不见」当成「人站起来了」 | `彩色图像断流` |
| 检测器进程崩了（P2 跟随） | 墙钟无 `present` | `person_timeout` (1.5s) | 停车（持续发零速） | — |
| 检测到人但不更新坐标 | 坐标到达时刻 | `person_timeout` (1.5s) | 停车，并 warn 一次 | `坐标已 … 没更新` |
| 检测器进程崩了（P2 自动灯） | 墙钟无 `present` | `present_timeout` (3.0s) | 按无人处理，走正常淡出。**没有这条就会整夜亮灯** | `没收到 /person/present` |
| 检测器死掉（P5 告警） | `fall_detector` 停发 | — | SUSPECTED 保持；CONFIRMING **必须走到 ALARM**（fail-safe） | — |
| 一次遮挡/蹲下（P5） | `unknown` 状态 | `absent_grace` (10.0s) | 不复位，继续确认；连续 10s 明确 `absent` 才判人已离开 | `画面里没人` |
| dock_manager 崩了 | 墙钟无 `/dock/inhibit` | `inhibit_timeout` (5.0s) | 跟随自行解除让位，并报 error | `判定回充节点已退出` |
| 跟随节点崩了 | 指令时效 | `follow_hold` (0.30s) / `follow_timeout` (0.6s) | twist_mux 输出零速，状态标 `follow(stale)` | — |
| 停到位但没充上电 | 电量涨幅 | `verify_charging` / `charge_check_timeout` (30s) | 只报 error，**不自动离桩**（BMS 上报会滞后） | `没有上涨` |
| 相机断流（P3 对准） | 墙钟无新帧 | `frame_timeout` (1.0s) | 主动把 `/dock/marker_visible` 置 false，让对准逻辑去扫描而不是原地微调 | — |
| 调度层崩了 | `/follow/enable` 断流 | `follow_enable_timeout` (5.0s) | **跟随器自动关门**并停发（带超时的门，不是「一次 OOM 永久关闭」） | `放行信号已 …s 没更新` |
| 调度层崩了 | `/mission/hold_motion` 断流 | `hold_timeout` (3.0s) | `twist_mux` 的「按住」自行过期。**急停不会过期** —— 两者的区别是刻意的 | — |
| 告警节点崩了 | `/alert/state` 断流 | `alert_timeout` (5.0s) | 不再按住轮子（回到跟随/待命），但急停仍然独立生效 | — |
| 语音节点崩了 | `/voice/command` 无人订阅 | — | 物理按扭、手机端、CLI 三条通道照常工作；反过来语音也必须能独立工作（它不经过调度层发急停） | — |
| TTS 引擎缺失 | `/voice/say` 没人订阅 | — | 只损失「说话」，告警录像、手机推送、日志全部不受影响 | — |
| Nav2 不可用 | action server 未就绪 | — | 低电回充降级为「直接让 `dock_manager` 停靠」；巡游功能跳过而不报错 | `Nav2 不可用` |
| 手机端断连（SSE） | 客户端断开 | `sse_interval` (1.0s) | 服务端线程自行退出，不影响其它接口；重连即恢复（事件列表接口仍然可用） | — |
| 急停后调度层重启 | 启动时握手 | `SAFETY_QOS` | 通过 `TRANSIENT_LOCAL` **继承**「已经急停」，不会从 `IDLE` 开始猜。实测重启后 0.8s 内进入 ESTOP | `收到急停闭锁` |
| **`twist_mux` 自己重启**（systemd `Restart=always`） | `/var/lib/lightdog/estop` 落盘 | `estop_state_path` | 启动时**先读回闭锁再建订阅**，车仍是锁住的；状态文件读不懂时按「已急停」处理。没有这条，自动重启就等于自动解除急停 | `恢复了急停闭锁` |

## 四、真机上会变得不一样的三件事

1. **`lightdog_person_detector` 的 `color` 后端不能上真机。**
   它认的是「高饱和橙色上衣」（`h_min/h_max/s_min`）。仿真里的「人」是我们自己造的
   几何体，颜色可控；真人穿什么都有。

   现在它已经是**可插拔**的：把 `p2_params.yaml` 的 `backend` 改成 `yolo`，
   并装好 `ultralytics` 即可（`yolo_model` / `yolo_conf` / `yolo_person_class`）。
   装不上时节点会打印一条 error 并**回退到 color**，不会崩 —— 但那条 error
   意味着它正在用占位实现，别当成正常。

   两个后端共用同一套「深度 → base_link」几何换算，所以换后端不会改变
   `/person/position` 的语义（`frame_id=base_link`）。这也是下游不用改的原因。

2. **相机安装假设 rpy = 0。**
   `lightdog_person_detector` 和 `lightdog_aruco_dock` 都只补偿了平移
   （`camera_offset`），因为 linorobot2 4wd 的 `depth_sensor_pose` 旋转是零。
   真机若把相机装成低头/侧装，必须改成用 TF 查询，**不能硬套这套公式** ——
   否则解出的 x/y 会整体偏，而且不报任何错。

3. **四驱 skid steer 的 odom yaw 高估约 35%。**
   所以 P3 的停靠朝向不依赖导航，而是用 ArUco 视觉对准（`align_before_dock`）。
   上真机后如果发现停车朝向总差一截，先怀疑这条，不要先去调 `align_kp`。

## 五、深度图对齐

P2/P5 都订阅 `/camera/depth/image_rect_raw` 并假设它和彩色图**逐像素对齐**。

- 仿真：`rgbd_camera` 输出天然对齐。
- 真机（RealSense D435i 等）：必须在驱动层开 `align_depth`，或者改订阅
  `/camera/aligned_depth_to_color/image_raw`。

不对齐时不会崩，只会安静地算出错误的距离；`lightdog_fall_detector` 发现尺寸
不一致会打 error 并提示这两个选项，`lightdog_person_detector` 则直接退回
「躯干高度/像素高度」粗估（距离误差可达数十厘米）。

## 六、外部输入边界

| 入口 | 限制 | 实现 |
|---|---|---|
| `POST /api/v1/events` 的 `type` | 只保留 ASCII 字母数字和 `-`、`_`，截断到 24 字符 | `safe_token()`，防路径穿越 |
| 事件 `label` / `note` | 截断到 80 / 200 字符 | 防超长字符串写进索引 |
| 事件 `pre` / `post` 秒数 | 钳到 0~300，NaN/非数字回落默认值 | `clamp_sec()` |
| 请求体 | 上限 1 MB，超限回 413 并断连接 | `Handler.MAX_BODY` |
| 鉴权 | query / `X-Auth-Token` / `Authorization: Bearer` 三种都收，定长比较 | `hmac.compare_digest` |
| 静态文件、事件/快照/录像路径 | 词法规整 + 越界判定 + 拒绝 `/`、`\` | `_static()` / `event_dir()` |

## 七、验证手段

```bash
# 1) 离线逻辑回归（宿主机，不需要 ROS/容器）
#    判据是「0 失败」；项数以 run.py 末尾打印的为准，别把它抄进文档
cd lightdog/tests && python run.py

# 2) 系统层验收（容器里，mission + voice 全链路）
bash sim/system.sh start
bash sim/system.sh status                 # mode / 跟随放行 / 急停 / 仲裁源 / 亮度
bash sim/p2.sh spawn && bash sim/p2.sh start
bash sim/system.sh status                 # 人出现后应为 mode=FOLLOW, follow=true
bash sim/system.sh listen 把灯打开          # 语音 → 灯（/led/brightness 0 → 0.85）
bash sim/system.sh listen 不要再跟着我了     # 语义正确性：必须停、不能朝人开
bash sim/system.sh listen 急停              # 语音急停：mission=ESTOP + mux=estop
bash sim/system.sh listen 解除急停           # 解锁不能被理解成加锁
bash sim/system.sh estop / clear_estop     # CLI 闭环确认（两边都到位才报成功）

# 3) 现场探针：只订阅、只统计，不改任何状态
#    （容器里 ros2 topic echo 不可靠，验收用这个而不是 CLI）
docker cp sim/probe.py sim:/tmp/probe.py
docker exec sim bash -lc "source /opt/ros/jazzy/setup.bash; \
  source /home/ros/linorobot2_ws/install/setup.bash; python3 /tmp/probe.py 20"

# 4) 相机断流（验证 fail-safe 真的会触发）
docker exec sim bash -lc "source /opt/ros/jazzy/setup.bash; \
  source /home/ros/linorobot2_ws/install/setup.bash; \
  ros2 topic hz /camera/color/image_raw"     # 先记下正常频率，再把相机进程 SIGSTOP
```

### 5) 真机服务化（systemd）

上面 1~4 都是手动拉起节点做的验证。真车上节点由 systemd 托管，
所以**还要单独验一次「自己起来」和「崩了自己回来」**：

```bash
sudo ./deploy/install.sh --dry-run     # 先看会生成什么
sudo ./deploy/install.sh
sudo ./deploy/verify.sh                # unit 语法 / 路径 / 进程 / 话题

# 崩溃重启真的会回来，且不会丢急停
sudo lightdog-estop set
sudo systemctl kill -s KILL lightdog-safety-mux
sleep 5
sudo lightdog-estop status             # 必须仍是「已闭锁」
sudo lightdog-estop clear --yes

# 反复重启是最难查的故障：active 但每 2 秒换个 PID
systemctl show -p NRestarts --value lightdog-mission
```

细节见 `deploy/README.md`。**注意这套只给真机**：它检测 `/opt/ros` 和
workspace 的 `install/`，仿真容器里没有 systemd，继续用 `sim/*.sh`。

### 6) 上车验收清单（一条命令）

`docs/firmware_protocol.md` §七 那张 10 条的表可以直接跑：

```bash
python3 deploy/acceptance.py            # 完整
python3 deploy/acceptance.py --quick    # 自查（时长缩短，不能签字）
```

生成的 `deploy/acceptance-<时间>.md` 里，脚本测不到的项会明确标成「待人工」，
不会因为它们没失败就算通过。退出码 2 表示「清单还没走完」。

> 注意：`ros2 topic echo` 在本容器里不稳定（订阅不到，而直接写 rclpy 的探针能收到），
> 验收时用探针而不是 CLI。
>
> 更要紧的一条：**`ros2 topic pub` 的短突发会丢，而且丢包是按订阅者发生的**。
> 实测 `-r 10 -t 6`（0.6s）连发 5 轮，`/safety/estop` 只到了 1 轮；更糟的是出现过
> `twist_mux` 收到了、`lightdog_mission` 没收到 —— 一个锁上、一个还自以为能动。
> 根因是 CLI 发布进程发完就退，RELIABLE 的样本还没被所有订阅者 ACK 就析构了。
> 所以：**脚本里的每条命令都要「发 → 读回 → 不行再发」**，`sim/system.sh` 已经
> 按这个原则改过；生产路径上的急停必须由常驻节点发（语音 / server / 固件桥接）。

---

## 八、树莓派 5 与 STM32 的落地约束（D1 / D3）

选型定了之后，下面这些从「可能需要考虑」变成了「一定会遇到」。每条都给了
**判据**和**症状**：不写判据的话，现场只会看到「有时候卡」这种没法复现的描述。

### 8.1 供电：最先咬人的一项

| 项 | 要求 | 不满足的症状 |
|---|---|---|
| 电源 | 官方 5V/5A（27W）USB-C，或等规格 | 欠压下 CPU 从 2.4GHz 掉到 1.5GHz，10Hz 的 `twist_mux` 变成 6~8Hz；电机启动瞬间还可能直接重启 |
| 判据 | `vcgencmd get_throttled` 必须是 `0x0` | 非 0 说明**过去**发生过欠压/降频，这个寄存器是粘滞的，断电才清 |
| 电机构电 | 电机与 Pi **分开供电、共地**，H 桥侧并 1000~4700µF | Pi 和电机共用一路 5V 时，加速瞬间的画面卡顿和 WiFi 掉线是同一个原因 |

`get_throttled` 的位含义（只用记这两个）：bit0 = 当前欠压，bit16 = 曾经欠压。
`deploy/verify.sh` 不查它（Pi 专属），上车时手工跑一次并记进验收报告。

### 8.2 散热：80°C 软降频，85°C 硬降频

- 判据：`vcgencmd measure_temp` 稳态 < 70°C；`vcgencmd get_throttled` 的 bit2/bit18
  （当前/曾经因温度降频）必须为 0。
- 症状：录像丢帧、跟随变慢、**日志里没有任何异常**。封闭外壳 + 连续写 NVMe/SD 是
  最坏组合。
- 结论：必须上主动散热（官方风扇或铝制散热壳），不要指望「室内 25°C 不会热」。

### 8.3 存储：SD 卡是消耗品

`events/` 里是 mp4 + `alarms.jsonl`，告警恰恰在半夜写。SD 卡的写放大 + 掉电
（直接拔电源）是这块板子最常见的「用三个月就坏」的原因。

| 做法 | 为什么 |
|---|---|
| 优先 NVMe（M.2 HAT+），至少 A2 级 SD | 事件视频是持续小文件写，A1 卡会明显拖慢 |
| `event_retain_days` / `max_event_mb` **必须设** | 默认 30 天/512MB 是给仿真的；真机按卡容量改，见 §一 |
| 开 `fstrim.timer` | 不 trim 的 eMMC/SD 写性能会一路衰减 |
| 留 10% 空闲 | 分区满了以后 recorder 的失败日志会淹没 journal |
| 数据目录放 NVMe，系统放 SD | `data_dir` 指到 `/mnt/nvme/lightdog`，坏卡只丢系统 |

### 8.4 WiFi 省电：一个必须关掉的默认值

```bash
# 症状：MJPEG 流每隔几秒卡 300ms，SSE 偶发断连，ros2 topic hz 有周期性缺口
sudo iw dev wlan0 set power_save off
```

开机自动执行：`/etc/systemd/system/wifi-powersave-off.service`（`Type=oneshot`,
`ExecStart=/sbin/iw dev wlan0 set power_save off`, `After=network.target`,
`WantedBy=multi-user.target`）。这条不做，后面所有「网络相关」的排查都会被它污染。

### 8.5 没有 CUDA：检测后端必须换算法，不是换参数

`p2_params.yaml` 的 `backend: yolo` 在 Pi 5 上直接跑 `ultralytics`（PyTorch，CPU）
大约 **1~2 FPS**（640×640），撑不住 `frame_timeout: 1.5s` 要求的节奏（它需要
≥3 FPS 才算「帧是新的」，实际希望 ≥8）。三条可行路线：

| 路线 | 预期 | 代价 / 现状 |
|---|---|---|
| onnxruntime + `yolov8n` 输入 320 | 6~10 FPS | **已实现**：`backend: onnx`，预处理与后处理在 `lightdog_person_detector.py`（`letterbox` / `decode_yolo_onnx` / `check_onnx_input`），有 37 项回归。导出：`yolo export model=yolov8n.pt format=onnx imgsz=320 opset=12`。**帧率是 Pi 上的预期值，尚未实测** |
| NCNN（若 onnxruntime 仍不够快） | 与上接近 | 需要额外适配，当前实现里没有。先用 onnx 路线测一轮再决定 |
| Hailo-8L（M.2 HAT，26 TOPS） | 30+ FPS | 要 `.hef` 模型和 HailoRT，驱动与内核版本绑定 |
| 保持 `color` 后端 | 与仿真一致 | 只认「高饱和橙色上衣」，见 §四.1 |

跌倒检测（P5）用的判据是深度图上的 `z_p95`，**不依赖检测框精度**，所以
「320 输入的 yolov8n」对它够用；跟随（P2）需要的是「人在画面里的大致方位 +
距离」，也够用。真正需要高精度的是「多人场景里认准要跟的那个人」——这不在当前
需求里，不要为它提前上算力。

### 8.6 音频与 USB 带宽

- Pi 5 **没有 3.5mm 模拟输出**。TTS 只能走 HDMI、USB 声卡或 I2S DAC。
  `lightdog_voice` 的 `tts_command` 要指向实际能出声的那条链路，配错的表现是
  「日志里 TTS 成功、现场没声音」。
- RealSense（USB3）+ USB 麦克风**分插两个控制器**，不要串在同一个 Hub 上：
  带宽抢不到的症状是深度图掉帧，而彩色正常 —— 会被误判成「深度相机坏了」。
- 麦克风灵敏度按现场调：老人说话音量低、距离远（2~3m），
  唤醒词阈值宁低勿高，但**误唤醒的代价只是多问一句**，漏唤醒的代价是叫不应。

### 8.7 CPU 调频：别用省电模式

```bash
# 10Hz 控制环在 powersave 下会抖到 8Hz，跟随表现为一顿一顿
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor
```

不需要 `PREEMPT_RT` 内核：控制频率只有 10Hz，抖动预算是几十毫秒，普通内核够用。
上 RT 的收益远小于它带来的驱动兼容问题（尤其 RealSense 与 WiFi）。

### 8.8 STM32 侧

| 项 | 决定/要求 | 理由 |
|---|---|---|
| 型号 | STM32F405/F407 级（168MHz，1MB Flash，192KB RAM，FPU） | 4 路 PWM + 4 路编码器 + 2 路 UART + CAN 全部够；F4 的生态与例程最厚 |
| 与 Pi 的连接 | UART，**3.3V 直连**（不要接 5V TTL），共地，短线或串 22Ω | 两边都是 3.3V 逻辑；5V TTL 会打坏 Pi 的 GPIO |
| 帧格式 | 定稿见 `docs/firmware_protocol.md` §六 | CRC16-CCITT-FALSE + 长度 + 类型；CRC 错整帧丢弃 |
| 波特率 | 115200 起步，验收通过后可提到 460800 | 心跳 10Hz + 速度 10Hz + 电量 1Hz 在 115200 下占用 < 5% |
| 看门狗 | **独立看门狗 IWDG 只在「收到心跳」时喂**，不要在 `while(1)` 里喂 | 主循环卡死时 `while(1)` 里的喂狗照样跑，等于没有看门狗 |
| 电机 PWM 布线 | 远离编码器线与 UART | 编码器计数乱跳的表现是 odom 原地漂移，通常被误判成「轮距没标定」 |
| 上电默认 | 驱动使能脚**默认低**（断 PWM），必须由固件主动拉高 | 见 §二：宁可多等 100ms，也不要「一上电就使能、等上位机来告诉它停」 |

> 分工没变：Pi 决定「该走多快」，STM32 决定「一旦上位机失联立刻自己停」。
> ROS 侧的 `twist_mux` 只是软件兜底，`kill -9` 掉它之后唯一还能刹住车的是固件。

### 8.9 连续运行：2 小时只是下限

验收脚本（`deploy/acceptance.py`）证明的是「此刻各项都对」，证明不了
「连跑一夜之后还对」。这台机器最终的形态是 24 小时开机、一周才有人想起来看一眼。

```bash
sudo python3 deploy/soak.py run --duration 7200 --out /tmp/soak.csv \
     --watch-dir /data
```

在树莓派上有四件事必须同时看，缺一条就可能是「曲线很漂亮但机器快挂了」：

| 检查 | 判据 | 不满足说明什么 |
|---|---|---|
| `vcgencmd get_throttled` | 跑完之后仍是 `0x0` | 非 0 = 这 2 小时里发生过欠压或过温降频。曲线会因此**变得更好看**（CPU 被限速、吞吐下降），是最容易误判成「优化有效」的假象 |
| `vcgencmd measure_temp` | 稳态 < 70°C | 封闭外壳里持续录像写卡是最坏组合，见 §8.2 |
| 可用内存 | 末值 > 起始值的 70% | 掉得比进程 RSS 总和多 = 有进程没被 `--match` 采到（gz/浏览器），或者内核 page cache 被顶掉 |
| fd 地板 | 2 小时后没有抬高 | 抬高 = 连接线程没退。**峰值不重要**，地板上移才是泄漏 |

另外两条 Pi 专属的：

- **日志也要有界**。`journalctl --disk-usage` 应低于 `SystemMaxUse`；节点每 10 秒
  报一次的串口错误在 8 小时里就是 2880 行。`deploy/lightdog.service.in` 里设了
  `SyslogIdentifier`，按节点过滤很容易，但**总量**要单独限制。
- **`/tmp` 在树莓派上是 tmpfs**（内存）。`soak.csv` 2 小时约 1.5MB，不碍事；
  但不要指望把事件录像写到 `/tmp` 来「省 SD 卡」——那是拿内存换存储。

判据的取值和「为什么涨了不等于泄漏」写在 `deploy/README.md` 的「长时间浸泡」一节。
