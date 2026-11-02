# 硬件实施前必须知道的事

这份文档不介绍功能，只回答一个问题：**这套代码搬到真车上，会在哪里咬人。**

仿真里所有传感器都是「永远有数据、时间永远在走、电量永远听话」的，所以下面的
问题在 Gazebo 里一条都测不出来。每条都对应代码里的一个判据或一个参数，改之前
请先看这一页。

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
| `p1` 灯组 | 见 `docs/P1_led_battery.md` | — | — | LED 真机由固件驱动，`lightdog_led` 要停掉 |

## 二、真机上不要启动的节点

| 节点 | 原因 |
|---|---|
| `lightdog_led` | 它把亮度翻译成 **Gazebo 材质自发光**。真机 LED 由固件订阅 `/led/brightness` 驱动，这个节点只会往一个没人转发的话题发消息，现象是「灯不亮也没有任何报错」。代码里检测到 `use_sim_time=false` 会打印一条 error 提醒 |
| `lightdog_person_sim` | 它用 `gz service set_pose` 让仿真里的「人」走起来 |
| `lightdog_server` 的静态页 | 页面本身不用鉴权（否则手机第一次打开就是空白），但 `/api/*` 全部要 token。见下表 |

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

## 四、真机上会变得不一样的三件事

1. **`lightdog_person_detector` 的颜色判据不能上真机。**
   它认的是「高饱和橙色上衣」（`h_min/h_max/s_min`）。仿真里的「人」是我们自己造的
   几何体，颜色可控；真人穿什么都有。这一层是**闭环验证用的占位实现**，接口
   （`/person/present` + `/person/position`，frame=base_link）才是产品契约。
   换模型时只需要替换 `_detect()`，下游不用改 —— 但务必保持 `frame_id` 语义，
   否则 `expected_frame` 会拦住它（这是设计好的保护）。

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
# 1) 离线逻辑回归（宿主机，不需要 ROS/容器）：53 项
cd lightdog/tests && python run.py

# 2) 现场探针：只订阅、只统计，不改任何状态
#    （容器里 ros2 topic echo 不可靠，验收用这个而不是 CLI）
docker cp sim/probe.py sim:/tmp/probe.py
docker exec sim bash -lc "source /opt/ros/jazzy/setup.bash; \
  source /home/ros/linorobot2_ws/install/setup.bash; python3 /tmp/probe.py 20"

# 3) 相机断流（验证 fail-safe 真的会触发）
docker exec sim bash -lc "source /opt/ros/jazzy/setup.bash; \
  source /home/ros/linorobot2_ws/install/setup.bash; \
  ros2 topic hz /camera/color/image_raw"     # 先记下正常频率，再把相机进程 SIGSTOP
```

> 注意：`ros2 topic echo` 在本容器里不稳定（订阅不到，而直接写 rclpy 的探针能收到），
> 验收时用探针而不是 CLI。
