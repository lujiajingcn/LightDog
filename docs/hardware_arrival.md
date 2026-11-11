# 实际到货硬件对照（RASPBOT V2 / 树莓派 5 16GB）

> 资料来源：`C:\Users\lujiajing\Downloads\智能小车产品简介\IMG_7080.PNG` ~ `IMG_7103.PNG`（24 张商品详情截图，1170×2532）。
> 本文只记录**图上写明**的规格与**代码里真实存在**的依赖；推断项一律标注「待确认」。
> 采购计划见 [`bom.md`](bom.md)，上真机通用坑见 [`hardware_readiness.md`](hardware_readiness.md)。
> 本文与那两份的关系：它们写的是「计划买什么 / 买回来会在哪咬人」，本文写的是**「实际买到的是这台，跟计划差多少，下一步改哪里」**。

---

## 〇、结论先行

**不能开箱即跑。** 这台车把 BOM 里最贵、最难买的几件（Pi 5、摄像头、底盘、电池、供电）一次性覆盖了，
但它不是按 LightDog 的硬件契约造的：有 **6 个结构性冲突**，其中 3 个不改代码就跑不起来。

| # | 冲突 | 严重度 | 是否要改代码 |
|---|---|---|---|
| C1 | 底盘是**四麦轮全向**，项目固件是**两轮差速** | 🔴 阻塞 | 要（协议帧 + 运动学 + 里程计） |
| C2 | **没有 STM32**，Pi 5 直接驱动；项目整套 CRC16 串口协议没有目标硬件 | 🔴 阻塞 | 要（写 Linux 侧适配层） |
| C3 | **TT 减速电机、参数表无编码器**；`/odom` 由固件上报帧驱动 | 🔴 阻塞 | 要（换里程计源或接受开环） |
| C4 | 相机装在**二自由度可动云台**上，项目三个感知节点用静态 `camera_offset` | 🟠 高 | 要（锁云台或动态 TF） |
| C5 | 预装 **Raspberry Pi OS + ROS 2 Humble**，项目是 **Ubuntu 24.04 + Jazzy** 生态 | 🟠 高 | 要选路线（见 §四） |
| C6 | 回充标签用 **AprilTag**，项目 `dictionary: DICT_4X4_50`（ArUco） | 🟡 中 | 大概率只改一行参数 |

**两个好消息**：相机确认是**普通 USB 单目**，之前定的「走普通相机路线」是对的；
驱动板明确**可为 Pi 5 提供 5.1V/5A**，BOM 里最担心的供电问题由车体解决了。

---

## 一、图上写明的规格（逐字摘录）

### 1.1 产品身份

- `IMG_7080`：RASPBOT V2，面向初学者的 AI 大模型视觉小车，「采用树莓派5主控，配备麦克纳姆轮、摄像头云台、AI大模型语音模块，支持全向运动、AI视觉交互与语音交互」，提供 16 节学习课程案例。
- `IMG_7086` 套餐对比表：

| 项 | 标准版 | 豪华版 |
|---|---|---|
| 推荐主控 | 树莓派5（2GB/4GB/8GB/16GB） | 树莓派5（4GB/8GB/16GB） |
| AI大模型语音模块 | / | ✓ |
| AI大模型玩法 | / | ✓ |
| AI语音交互 | / | ✓ |
| AI视觉交互 | ✓ | ✓ |
| 主控系统 | Raspberry Pi OS | Raspberry Pi OS |
| ROS系统 | ROS2 Humble | ROS2 Humble |
| 推荐人群 | 适用于学习AI视觉功能 | 适用于学习AI大模型、AI语音交互及AI视觉功能应用 |

- `IMG_7082`「发烧级硬件配置」：超声波 / 红外巡线 / OLED / 灯条、**麦克纳姆轮运动底盘**、**二自由度摄像头云台**、树莓派5主板。
- `IMG_7100`：「采用ROS2系统开发」，明确 **ROS2 Humble**「*本产品采用此版本」，支持 Rviz 仿真；`IMG_7086` 提到「赠送亚博原创课程资料」，品牌为亚博智能。
- 用户已确认：所购为带 **树莓派 5 16GB** 的版本；上一轮商品图另标 **64GB TF 卡（选配）**。

### 1.2 产品参数（`IMG_7102`、`IMG_7103`，单位 mm）

| 项 | 值 |
|---|---|
| 产品名称 | RASPBOT V2 AI大模型视觉小车 |
| 主控 | 树莓派 5 |
| 系统版本 | **Raspberry Pi OS + ROS2 Humble** |
| 输入 | 超声波传感器、四路巡线传感器、按键、红外接收头、摄像头、AI大模型语音模块 |
| 输出 | OLED显示屏、电机、RGB炫彩灯、蜂鸣器、舵机、腔体扬声器 |
| 可拾音距离（豪华版） | 5 米 |
| 摄像头方案 | **100W 像素 USB 摄像头** |
| 摄像头分辨率 | **480p/720p** |
| 摄像头视场角 | 对角 120°、**水平 110°** |
| 摄像头云台 | 2 自由度（水平 180°、上下 110°） |
| 二维云台舵机 | YB-SG90 数字舵机；脉宽 500–2500us ↔ 0–180°；工作电压 4.8–6.0V DC |
| 电机方案 | **TT 减速电机** |
| 电机扭矩 / 转速 | 0.8 N·m / 245±10% rpm（参数表原文；TT 马达这个扭矩量级存疑，以实测为准） |
| 机身材质 | 金属 |
| 遥控方式 | PC电脑、红外遥控器、建图导航 APP（iOS/Android） |
| 电池 | **7.4V 25 锂电池组（2000mAh）**（"25" 疑为 25C，按 2S 锂电理解，待确认） |
| 续航 | 约 2h（持续运动） |
| 组装后重量 | 标准版 895g / 豪华版 917.4g |
| 组装后尺寸 | 208.74 × 165.83 × 127.05 mm |
| 尺寸图 | 正视图轮距宽 165.83；侧视图轴距 117.20、总长 208.74、总高 127.05、云台高 80 |

### 1.3 结构与供电（`IMG_7101`、`IMG_7085`）

- 爆炸图部件：AI大模型语音模块 **[选配]**、扬声器 **[选配]**、二自由度摄像头云台、超声波模块、四路巡线模块、**7.4V 大容量电池组**、OLED屏、散热风扇、树莓派5 **[选配]**、转接板、**机器人驱动板**、**麦克纳姆轮×4**、**减速TT电机×4**。
- 局部特写标注：AI大模型语音模块、高性能二自由度云台、**高清 110° 广角摄像头**、四路巡线模块、超声波传感器、OLED显示屏、RGB炫彩灯带、7.4V锂电池组。
- `IMG_7085` 原文：「机器人支持树莓派5供电协议，**可为树莓派5提供 5.1V/5A 供电**，避免卡顿和重启等异常情况」。
- `IMG_7097`：四路红外传感器巡线（4 个红外探头）+ OpenCV 视觉巡线（ROI、二值化、PID）。
- `IMG_7099`：超声波避障（d=10cm）、红外巡线模式、蜂鸣器鸣笛、超声波跟随。

### 1.4 感知与交互能力

- `IMG_7088`「视觉大模型」原文：**「RASPBOT V2 搭载 USB 高清摄像头」**，理解/分析图像并输出文本和语音反馈。全 24 张图**没有出现**深度相机、双目、RGB-D、ToF、点云、对齐深度等字样。
- `IMG_7096`「二自由度摄像头云台」：「搭载两个高品质数字舵机、**高清广角摄像头**，支持在水平和垂直方向进行 AI 视觉追踪」，110° 垂直 / 180° 水平旋转。
- `IMG_7095`「360°全向移动」：前行、**横移**、斜行、旋转，配麦轮受力示意图。
- `IMG_7091` AI视觉识别：Python + OpenCV + MediaPipe，YOLOv5-Lite 物体识别、车牌识别、人体姿态估计（33 关键点）、垃圾识别、画笔/手指控制。
- `IMG_7093`：「**标签码识别/追踪/跟随：支持识别 AprilTag 标签码**，实现对标签码的云台跟踪以及小车跟随」；另有二维码控制、14 类手势控制。
- `IMG_7083`「升级AI大模型语音模块」：远场拾音、回声消除与环境降噪。
- `IMG_7087`「语音大模型」：搭载 AI大模型语音模块和扬声器，语音与文字实时互转，接入通义千问；`IMG_7081` 大模型列表含通义千问 / DeepSeek / 百川智能 / 零一万物。
- `IMG_7094`「AI语音交互（仅限豪华版）」：离线语音控制玩法无需大模型；「语音控制小车运动：支持通过预设语音指令（**固定词条**）控制前进、后退、左转、右转和停止」；灯带颜色变幻、颜色问答、语音控制自动驾驶/物体追踪。
- `IMG_7098`：AI 自动驾驶（场景地图需另购）：转向决策、鸣笛标志识别、红绿灯识别、四红外探头巡线。

---

## 二、上一轮 5 个待确认项的答案

| # | 问题 | 答案 | 依据 |
|---|---|---|---|
| 1 | 相机型号 / 是否出对齐深度 | **普通 USB 单目，无深度**。100W 像素、480p/720p、对角 120°/水平 110° 广角；装在 2 自由度云台上 | `IMG_7088`、`IMG_7102`、`IMG_7096`、`IMG_7101` |
| 2 | 轮子类型 / 有无编码器 | **麦克纳姆轮 ×4 + TT 减速电机 ×4**，全向运动。参数表**未列编码器**；按 TT 马达常规配置推断为开环，**须拆车或查商家文档确认** | `IMG_7095`、`IMG_7101`、`IMG_7102` |
| 3 | 下位机主控 / 能否刷固件 | **没有独立 STM32**。主控即 Pi 5，经「转接板 + 机器人驱动板」驱动电机。商家协议文档未随图提供，能否刷写未知 | `IMG_7101`、`IMG_7102` |
| 4 | 电池与降压 / 物理急停 | 7.4V 2000mAh 锂电池，驱动板给 Pi 5 **5.1V/5A**（满足 27W）；有散热风扇、OLED、蜂鸣器。输入只列「按键」，**没有自锁蘑菇头急停** | `IMG_7085`、`IMG_7101`~`IMG_7103` |
| 5 | 语音模块怎么对外输出 | 豪华版含 AI大模型语音模块（5m 拾音）+ 扬声器；离线固定词条 + 在线通义千问。输出在**商家 Python 程序内部**，不是 ROS 话题 | `IMG_7083`、`IMG_7087`、`IMG_7094`、`IMG_7102` |

---

## 三、结构性冲突（逐条对照代码）

### C1 四麦轮全向 vs 两轮差速 🔴

- 实车：`IMG_7095` 支持前行/横移/斜行/旋转，四麦轮独立驱动。
- 代码：`lightdog/lightdog_mcu_proto.py:43` 下行 `T_CMD_VEL 0x01` 只有 `f32 linear.x + f32 angular.z`；
  `firmware/app/lightdog_app.c:569-578` 用 `half = 0.5 * angular * wheel_base` 算 `l = v - half`、`r = v + half`，
  再 `motor_set(l, r)` —— **左右两路**差速。
- 后果：横移能力完全用不上；纯差速指令下斜滚子持续打滑，里程计（即使有）会漂；`wheel_base_m` 对麦轮没有对应物理量。
- 两条路：① 把麦轮当四轮差速凑合（改动最小，能跑但精度差，且必须实测打滑系数）；
  ② 从 `T_CMD_VEL` 扩展 `linear.y`、四轮逆解矩阵、`T_ODOM` 的 `vx/vy`、URDF 一路改造 —— 这是改运动学，不是改参数。

### C2 没有 STM32，串口协议没有目标硬件 🔴

- `docs/hardware_readiness.md` 的 D3 决策是「MCU = STM32，Pi 不再直接驱动电机」，
  `deploy/nodes.conf:45` 的 `mcu` 单元描述就是「下位机串口桥接（STM32）」。
- 实车是 Pi 5 + 商家驱动板，商家 ROS2 Humble 包直接发电机命令。
- 后果：整套 `firmware/`（CRC16-CCITT-FALSE 帧、IWDG、急停闭锁、编码器 HAL）在车上**没有可烧写的目标**；
  `mcu_params.yaml` 的 `/dev/ttyAMA0` / 115200 对不上就一帧都通不了。
- 现实做法：写「商家驱动板串口协议 ↔ LightDog 帧」适配层，或把 `lightdog_mcu_bridge.py` 的后端换成商家 SDK/ROS 话题，
  保留它在上层看到的 `/cmd_vel_muxed`、`/odom`、`/battery`、`/safety/estop`、`/led/brightness` 契约。
  **上层节点一行不用改**，代价是安全联锁（急停闭锁、心跳看门狗）要在适配层重新实现，不能丢。

### C3 无编码器，`/odom` 无源 🔴

- `lightdog/lightdog_mcu_bridge.py:377` 的 `_publish_odom()` 只是把 `decode_odom()` 的 `x, y, yaw, v, w` 转发；
  真值由固件按 `firmware/app/lightdog_app.c:252` 的 `m_per_tick = 2π·wheel_radius_m / ticks_per_rev` 从编码器积分得到。
  默认 `wheel_radius_m = 0.0325`、`ticks_per_rev = 1560`（`lightdog_app.c:84-86`）全是仿真 4wd 值。
- 实车参数表无编码器项 → `/odom` 要么没有发布者，要么是开环积分（会越走越偏）。
- 影响面：`lightdog_mission.py` 的 Nav2 `NavigateToPose`、`lightdog_follower.py` 的跟随闭环、
  `lightdog_aruco_dock.py` 的位姿链都依赖它。
- 三条路：① 给 TT 电机外挂霍尔/光电编码器（BOM §3.3 已有接线方案）；
  ② 用「单目视觉里程计 / AprilTag 位姿」补 `/odom`（Pi 5 16GB 算得动，但要新写节点）；
  ③ 明确降级为开环，只保留 P2 随身跟随这类不依赖全局定位的功能。

### C4 可动云台 vs 静态相机外参 🟠

- `IMG_7096`：云台水平 180°、上下 110°，商家自带的「颜色/人脸/标签码追踪」会主动转动它。
- 项目三个节点都把相机当**固定在 base_link 上**：
  `lightdog/config/p2_params.yaml:43`、`p3_params.yaml:25`、`p5_params.yaml:17` 全是
  `camera_offset: [-0.065, 0.0, 0.255]`；`p3_params.yaml:28` 固定 `optical_rpy_deg: [-90, 0, -90]`；
  `base_link_height: 0.1195`（`p3_params.yaml:26`、`p5_params.yaml:18`）也是仿真 4wd 值。
- `lightdog/lightdog_aruco_dock.py:45-46` 的注释**已经预见了这个情况**：
  「真机上若换成可转动的云台，再改成 tf2 查询」—— 目前还是静态外参。
- 后果：云台一转，P2 单目测距的 `x_cam/y_cam` 反投影、P3 的 `solvePnP` 回充位姿整体错，
  而且是**不报错的错**（数值照常出，只是方向不对）。
- 两条路：① 上电把两个舵机锁到固定角度并禁止商家追踪程序转动（改动最小，推荐先这样）；
  ② 读舵机角度，动态发布 `camera_link -> base_link` 的 TF，三个节点改查 tf2。

### C5 Raspberry Pi OS + ROS2 Humble vs Jazzy 生态 🟠

- 实车：`IMG_7102`「系统版本 Raspberry Pi OS + ROS2 Humble」，`IMG_7100` 明确「*本产品采用此版本」。
- 项目：仿真镜像是 `linorobot2:gazebo`（ROS 2 Jazzy + Gazebo Harmonic），`docs/linorobot2_upstream.md` 记录上游分支为 `jazzy`、
  Jazzy 上 `description.launch.py` 必崩并打了 patch；`sim/*.sh` 硬编码 `/opt/ros/jazzy/setup.bash`；
  `Readme.md` 与 `docs/hardware_readiness.md:21` 的 D1 写的是「ARM64 Ubuntu 24.04 + ROS 2 Jazzy」。
- **更正上一轮的一处说法**：`deploy/install.sh:90` 的发行版探测是
  `for d in "${ROS_DISTRO:-}" jazzy humble iron rolling`，**Humble 本身在候选里**，安装器不排斥它，
  也没有 Ubuntu 版本号的硬检查。真正的差异在 Python 版本（Pi OS Bookworm 3.11 vs Ubuntu 24.04 3.12）、
  `opencv-contrib` 的 aruco API、launch 里裸 `LaunchConfiguration` 传 bool/int 的类型推断、以及上游分支。
- 项目节点只用 `std_msgs`/`sensor_msgs`/`geometry_msgs`/`nav2_msgs` 等稳定消息，
  **纯 rclpy 部分理论上 Humble 也能跑，但未上机验证**，不能当成已支持。

### C6 AprilTag vs ArUco 字典 🟡

- `IMG_7093` 商家用 AprilTag；项目 `lightdog/config/p3_params.yaml:20` 是 `dictionary: DICT_4X4_50`（ArUco），
  `lightdog_aruco_dock.py:251` 用 `cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))`。
- OpenCV contrib 的 aruco 模块**同时支持** `DICT_APRILTAG_16H5 / 25H9 / 36H10 / 36H11`，
  且代码是按名字动态取字典的 —— 所以只要商家用的是这四种之一，**改 `p3_params.yaml:20` 一行即可，代码不用动**。
- `marker_length: 0.084`（`p3_params.yaml:18`）是**黑框边长**，必须按实印 AprilTag 用卡尺重测；
  `marker_id: 0` 也要跟实际贴的标签对上。若商家用的是非 OpenCV 家族的 tag，才需要换标签或换检测器。

### 其它到货差异（不阻塞，但要记）

| 项 | 实车 | 项目现状 | 处理 |
|---|---|---|---|
| 相机话题 | 商家驱动话题名未知，多半不是 `/camera/color/image_raw` | `p2_params.yaml:13-14` 写死 `/camera/color/image_raw` + `/camera/color/camera_info` | launch remap 或改参数 |
| `camera_info` | 普通 USB 摄像头一般**不发布**已标定内参 | P2/P3 的 `solvePnP` 与单目测距硬依赖 `msg.k` | 棋盘格标定 + 自己发 `camera_info`；110° 广角畸变必须去畸变 |
| 2D LiDAR | **没有**。只有超声波（前向）+ 四路红外（朝下）+ 单目 | 项目节点不直接订阅 `/scan`（已 grep 确认），但 `mission` 走 Nav2 `NavigateToPose` | Nav2 costmap 缺测距源：要么只用 P3 视觉伺服对接，要么补 LiDAR / 深度转 laserscan |
| 电池遥测 | 驱动板是否上报电压/电流未知 | `T_BATTERY 0x81` 要 `percentage/voltage/current`；`dock_manager` 靠它判低电 | 无电压上报时低电回充会失效，需在适配层合成或明确降级 |
| 物理急停 | 只有「按键」（点动语义） | `lightdog_mcu_bridge.py:395` 依赖 `T_ESTOP_STATE 0x83` 的**自锁闭锁**语义；BOM §交互要求 22mm 自锁蘑菇头 | 外接自锁按钮，或在适配层把按键映射成闭锁（点动松手即恢复，与代码语义不符） |
| 存储 | 64GB TF 卡（选配），**无 NVMe** | `p4_params.yaml:29/45` 配额 512MB / 512MB | 按 `df -h /data` 重配；持续小文件写寿命风险见 `hardware_readiness.md` §8.3 |
| 内存 | **16GB**（BOM 计划 8GB） | `max_event_frames` 等可放宽 | 只解决 OOM；**磁盘配额别跟着放大** |
| 机身尺寸 | 208.74 × 165.83 × 127.05 mm，917.4g | 外参/轮径/轮距全是仿真 4wd 值 | 全部重标，见 §五 |

---

## 四、系统路线（必须先拍板，否则后面配置白改）

| 路线 | 做法 | 优点 | 风险 |
|---|---|---|---|
| **A（推荐先试）Docker 跑 Jazzy** | 保留商家 Raspberry Pi OS + Humble 镜像，在容器里跑 Ubuntu 24.04 + ROS 2 Jazzy；容器映射 `/dev/video*`、`/dev/ttyUSB*`、`/dev/gpiochip*`、`/dev/i2c-*`，`--network=host` | **项目回归底座与仿真镜像完全一致**；16GB 内存足够；商家课程/驱动库不丢 | 设备直通、摄像头延迟、串口实时性、systemd 部署要重验；`deploy/install.sh` 的 unit 要改成管容器 |
| **B 重装 Ubuntu 24.04 + Jazzy** | 刷 Ubuntu 24.04 Server ARM64，装 ROS 2 Jazzy，`colcon build` | 与 `install.sh` 假设完全一致，systemd 直接可用 | 商家驱动板/语音模块/OLED 的 Python 库多半只适配 Raspberry Pi OS；放弃预装课程与建图 APP |
| **C 保留 Humble 直接跑** | 在商家镜像上 `colcon build` 本项目包 | 改动最小，所有板载外设开箱可用 | 全套 385 项回归与仿真脚本都是 Jazzy 语境；`linorobot2` 的 `jazzy` 分支/patch 用不上；launch 参数类型、OpenCV aruco API 差异要逐条上机验 |

> 无论选哪条，`deploy/install.sh` 都要带 `--ros-distro humble` 或 `jazzy` 明确指定，别依赖探测顺序。
> 真机不需要 Gazebo，所以 C5 里最重的「上游 linorobot2 分支」问题在真机上其实**可以绕开**。

---

## 五、到货必改清单（不改就「不动 / 不报错但一直不对」）

> 以下按 `docs/hardware_readiness.md` §一 扩展，新增实车相关项。
> **`depth_mode` 一律带引号**：裸写 `on/off` 会被 PyYAML 1.1 解析成 bool（`p2_params.yaml:84`、`p5_params.yaml:50` 有注释）。

| 文件 | 参数 | 现值 | 真机值 |
|---|---|---|---|
| 所有 `*_params.yaml` | `use_sim_time` | `true` | **`false`**（`p2_params.yaml:152` 的 `twist_mux` 本来就是 false，别动） |
| `deploy/nodes.conf:45` | `mcu` 第 5 列 | `no` | `yes`；若走 C2 适配层，改为新适配节点 |
| `lightdog/config/p2_params.yaml:25` | `backend` | `color` | `onnx`（先导出 `yolo export model=yolov8n.pt format=onnx imgsz=320 opset=12`） |
| `p2_params.yaml:86` | `depth_mode` | `'auto'` | **`'off'`**（无深度） |
| `p2_params.yaml:13-14` | 图像/内参话题 | `/camera/color/*` | 按商家话题 remap + 标定 `camera_info` |
| `p2_params.yaml:43` / `p3_params.yaml:25` / `p5_params.yaml:17` | `camera_offset` | `[-0.065, 0, 0.255]` | 实车实测；若云台可动必须走 §C4 方案 |
| `p2_params.yaml:92-93` | `mono_height_color/person` | `0.62 / 1.70` | 按实际目标身高标定（单目测距唯一精度来源） |
| `p3_params.yaml:20` | `dictionary` | `DICT_4X4_50` | 按商家 AprilTag 家族改 `DICT_APRILTAG_*` |
| `p3_params.yaml:18-19` | `marker_length` / `marker_id` | `0.084 / 0` | 卡尺实测黑框边长 / 实际标签 id |
| `p3_params.yaml:26` `p5_params.yaml:18` | `base_link_height` | `0.1195` | 实测 `base_link` 离地高度 |
| `p3_params.yaml` | `sim_charging` | `true` | `false` |
| `p5_params.yaml:53` | `depth_mode` | `'on'` | **`'off'`** + `nodes.conf:36 fall-det` 保持 `no` |
| `p5_params.yaml:83` | `sip_ami_secret` | `''` | 随机串（空着不会打任何告警电话）；Asterisk 真机四步验收仍待做 |
| `p4_params.yaml:70` | `auth_token` | `''` | 必填（空 = 同 WiFi 任何人能看摄像头） |
| `p4_params.yaml:54/29/45` | `faststart` / `max_disk_mb` / `max_event_mb` | `false / 512 / 512` | `true` / 按 `df -h /data` 重配 |
| `p4_params.yaml` | `ssl_certfile` / `keyfile` | 空 | 填自签证书路径 |
| `system_params.yaml:40-42` | `dock_pose_x/y/yaw` | `-0.5 / 0 / 0` | 从实建地图量，或 `/detected_dock_pose` 反算 |
| `mcu_params.yaml` | 串口设备 / 波特率 | `/dev/ttyAMA0` / `115200` | 按商家驱动板实际串口改（若走适配层则整体重写） |
| `firmware/app/lightdog_app.c:84-86` | `wheel_radius_m` / `ticks_per_rev` | `0.0325 / 1560` | 实测；无编码器时见 C3 |

**六项实测照 `docs/bom.md:308-322`**：轮半径、轮距（麦轮要量横向+纵向两组）、编码器每转计数、
marker 边长、桩位姿、存储容量；另加相机内参棋盘格标定、`mono_height_person` 身高标定、
云台固定角度下的 `camera_offset`、`vcgencmd get_throttled` 必须为 `0x0`。

---

## 六、上机验收顺序（先证明链路通，再谈功能）

1. **系统层**：选定 §四 路线 → `python lightdog/tests/run.py`（宿主机 385 项）→ 目标板上 `colcon build` →
   `sudo ./deploy/install.sh --dry-run --ros-distro <所选>` 看 unit 是否正确。
2. **供电**：满电 + 四电机启动瞬间跑 `vcgencmd get_throttled`，必须 `0x0`；稳态 CPU 温度 < 70°C
   （`hardware_readiness.md` §8）。5.1V/5A 是商家声明值，要用实测确认。
3. **相机**：`ros2 topic hz` 确认彩色帧频率；`camera_info` 的 `k` 非零且已去畸变；
   棋盘格重投影误差记录到本文附录。
4. **云台**：先锁死到固定角并确认商家追踪进程没在动它，再跑 P2 静态目标测距，
   对比 1m/2m/3m 实测距离 —— 这一步能同时验 C4 和 `mono_height_person`。
5. **串口/底盘**：`mcu` 单元起来后确认心跳、`T_ODOM`、`T_BATTERY` 有帧；
   低速直线 → 原地旋转 → 横移（若做 C1 方案②），分别比对地面实测位移。
6. **急停**：按下后 `/safety/estop=true`、PWM 断、松手不自动恢复（闭锁语义）。
   没有自锁按钮时这条**不能通过**，要如实记 FAIL，别用软件置位冒充。
7. 再依次 `deploy/verify.sh`、`acceptance.py`（注意 `--only` 子集不会给「全部通过」）、
   `soak.py run --duration 7200`。

---

## 七、仍需问商家 / 实测确认（本文没有替你猜）

1. 机器人驱动板的串口协议文档、波特率、电机命令格式；能否刷自定义固件。
2. TT 电机**到底有没有编码器**（参数表未列，这是 C3 的分岔点）。
3. AprilTag 具体家族（tag36h11 / tag25h9 / …）与标签实物边长、id。
4. USB 摄像头在 ROS2 下实际发布的话题名，以及有没有 `camera_info`。
5. 语音模块能否把识别文本导出到 ROS 话题或串口；`tts_command` 用的 ALSA 设备号。
6. 「按键」的电气与语义（点动还是自锁）、有没有预留急停接口。
7. 电池「7.4V 25」的完整标注（是否 25C）、驱动板 5V/5A 在满载时是否稳。
8. 64GB TF 卡是否随货，是否有 NVMe/M.2 扩展位。
9. 商家 Humble 镜像里 `python3-opencv` 是否带 `aruco` contrib（`lightdog_aruco_dock.py:249-250` 会直接报错）。
