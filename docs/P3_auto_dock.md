# P3：低电自动回充

**状态：仿真端到端跑通**（2026-11-01）
**代码**：`lightdog/lightdog_aruco_dock.py`、`lightdog/lightdog_dock_manager.py`、
`lightdog/models/lightdog_dock/`、`sim/p3.sh`、`sim/patch_upstream.py`（补丁 2/3/4）

---

## 一、目标

电量低时自己开回充电底座、停靠、充电、充满后离桩。

在 P2 的拓扑上加两条链路：

```
                ┌─ 视觉对准（P3 新增）────────────────────────────┐
                │                                                 │
  相机 ──> lightdog_aruco_dock ──/detected_dock_pose──> lightdog_dock_manager
                │                                        (低电状态机)
                │                                                 │
                │                                        /dock_robot action
                │                                                 ↓
                │                                        Nav2 docking_server
                │                                                 │ /cmd_vel_dock
                └─────────────────────────────────────────┐       │
                                                          ↓       ↓
                                              lightdog_twist_mux（prio 30）
                                                          │ /cmd_vel_muxed
                                                  collision_monitor
                                                          │ /cmd_vel
                                                    ros_gz_bridge → 底盘
```

关键点：**回充的速度也走仲裁和碰撞保护**，不是直连底盘。

---

## 二、三个绕了几圈才想明白的设计

### 1. `dock.pose` 的语义是「机器人停靠位姿」，不是「桩的位姿」

`opennav_docking` 的 dock database 里，`pose` 表示**机器人停在桩上时应该在的位姿**。
我一开始写的是桩自己的位姿（原点在桩中心、朝向朝外），症状非常隐蔽：

- `docking_server` 把 staging 点算到桩的**背后**（穿墙位置）
- 导航当然到不了 → `error_code=903 FAILED_TO_STAGE`

正确做法是：桩位姿沿桩的 +x 前移 `DOCK_APPROACH`（0.30m，车头贴住桩体时
base_link 距桩原点的距离），朝向转 180°（机器人是正对着桩开进去的）。
这个换算由 `sim/patch_upstream.py` 在生成 `/tmp/dock_database.yaml` 时做，
ArUco 节点发布前做同样的一步，两边必须一致。

### 2. 桩体刻意压在激光扫描平面以下

4wd 的 2D 激光装在 `base_link` z=+0.125（离地约 **0.2445m**）。充电桩整个模型
最高点 0.225m，`/scan` 里**一个点都没有**。

这不是为了偷懒，而是必须的：Nav2 的 `collision_monitor` 用的是
`FootprintApproach`（预测 1.2s 内碰撞就停），docking 的最后 10cm 必然被它拦停。
真实产品靠触点/红外做末端定位，仿真里用「物理碰撞挡住车头」代替 ——
桩有 `collision`，机器人撞上去就停住，只是激光看不见它。

代价写在「已知限制」里：**桩在代价地图上不存在**，全局规划会从桩上穿过去。
P3 的边界是「已经在桩附近」，所以不暴露这个问题；P4 做任务调度时必须处理。

### 3. 停靠前必须自己做视觉对准，不能交给 staging 导航

这是 P3 花时间最多的一处。现象：

```
NAV_TO_STAGING → INITIAL_PERCEPTION → error_code=904 (FAILED_TO_DETECT_DOCK)
```

查下去是**里程计 yaw 严重失真**：

| 量 | 值 |
|---|---|
| Gazebo 真值 yaw（原地转约 92°） | 1.605 rad |
| `/imu/data` 的 yaw | 1.607 rad ← 绝对姿态本身很准 |
| `/odom`（EKF 输出） | 2.503 rad ← 高估 55% |

原因：四驱原地转向靠轮子**横向滑移**（skid steer），轮速积分出的转角必然偏大；
而上游 `ekf.yaml` 的 `imu0_config` 只取 `vyaw`（角速度），姿态 yaw 三项全是
`false` —— 也就是说 yaw 没有任何绝对观测能纠正它。

试过把 `imu0_config` 的 yaw 打开、并给 `/imu/data` 补上 `orientation_covariance`
（原来是全 0，robot_localization 会当作无效测量丢掉），**实测 EKF 输出的 yaw
依旧没有变化**，仍是纯积分结果。没有再往下钻 rl 的内部逻辑，改用下面的方案。

最终方案：`docking_server` 进 `INITIAL_PERCEPTION` 后只会等检测、**不会转向**，
所以对准必须在发 action 之前由我们自己做完：

- 状态机新增 `ALIGNING`
- 看不见 marker → 以 0.50 rad/s 慢扫（约 12.6s 一圈 sim time）
- 看见了 → 用 `atan2(y, x)` 算偏角，P 控制转到 ±5.7° 以内
- 对准完成后才发 `DockRobot(navigate_to_staging_pose: false)`

全程只用相机 bearing，**不碰里程计**。实测偏角 48.4° → -0.2°，收敛约 8 秒。

---

## 三、交付物

| 文件 | 作用 |
|---|---|
| `lightdog/models/lightdog_dock/model.sdf` | 充电桩。几何拼的 ArUco（不用贴图，见坑 1） |
| `lightdog/models/lightdog_dock/materials/textures/aruco_0.png` | 同一张 marker 的位图，留给真机打印 |
| `lightdog/lightdog_aruco_dock.py` | ArUco 检测 → `/detected_dock_pose`（停靠位姿，base_footprint 系） |
| `lightdog/lightdog_dock_manager.py` | 低电状态机 + 视觉对准 + dock/undock action 客户端 |
| `lightdog/config/p3_params.yaml` | 两个节点的参数 |
| `lightdog/launch/lightdog_p3.launch.py` | 起上述两个节点 |
| `sim/p3.sh` | 宿主机助手（spawn/start/dock/undock/battery/status/watch/pose） |
| `sim/patch_upstream.py` | 补丁 3（docking 出口 → `cmd_vel_dock`）、补丁 4（docking 参数 + 生成数据库） |
| `sim/.env` | `DOCK_X/Y/YAW`、`DOCK_APPROACH` |

---

## 四、实测验收

### 4.1 视觉检测精度（机器人距桩 0.40m，正对）

| 量 | 检测值 | 真值（由 gz 位姿反算） | 误差 |
|---|---|---|---|
| 桩在机器人坐标系 x | 0.3933 m | 0.3953 m | 2 mm |
| 桩在机器人坐标系 y | -0.0591 m | -0.0621 m | 3 mm |
| yaw | 172.1° | 173.0° | 0.9° |

### 4.2 完整闭环（注入电量 15% 触发，全自动）

```
15.0% 低于阈值，确认中…
发起回充（低电）：先做视觉对准
对准完成（偏角 -0.1°），开始停靠（距桩约 0.78m）
回充进度: NAV_TO_STAGING → INITIAL_PERCEPTION → CONTROLLING
停靠成功（重试 0 次），进入 DOCKED          ← 19 秒，实际移动 0.78m
电量已满（95.3%），开始离桩                ← 充电 80 秒
发起离桩
离桩成功，回到 IDLE
```

停靠后真值位姿：`(-1.897, 0.494, -179.7°)`，桩在 `(-2.3, 0.5)` →
机器人中心距桩中心 **0.403m**，目标停靠位 `(-2.0, 0.5)` 误差 **0.104m**
（`docking_threshold: 0.12` 以内）。

### 4.3 视觉扫描（从背面开始）

初始朝向与桩相差 180°，先慢扫 40 秒找到 marker，再收敛对准：

```
还没看到 marker，慢速扫描中…      ×7
对准中：偏角 +48.4°，w=+0.45
对准中：偏角 +35.6°，w=+0.45
…
对准完成（偏角 -0.2°），开始停靠
```

### 4.4 速度链路

`docking_server` 的出口被补丁改到 `/cmd_vel_dock` 后，实测 `/cmd_vel` 的
发布者仍然是 `collision_monitor`，仲裁状态在回充期间为 `dock`、
跟随期间为 `follow` —— 两条链路都过碰撞保护。

---

## 五、踩的坑

1. **PBR 贴图在 gz Harmonic 里解析不了 `model://`**

   `albedo_map: model://lightdog_dock/materials/textures/aruco_0.png` 报
   `Unable to find file with URI`，marker 渲染成一块黑。把模型目录从 `dock`
   改名成 `lightdog_dock`（与 `model.config` 的 name 对齐）、把
   `GZ_SIM_RESOURCE_PATH` 指到 models 父目录，**依然找不到**。

   放弃贴图，改成**用 box 拼**：白色底板 + 黑色 6×6 底板 + 8 个白色数据格。
   反而更好 —— 软件渲染下省掉纹理采样，颜色对比稳定，而且没有资源解析这个
   失败点。`aruco_0.png` 留在仓库里是给真机打印用的。

2. **`<emissive>` 不是 `<pbr><metal>` 的合法子元素**

   `XML Element[emissive], child of element[metal], not defined in SDF`。
   SDF 1.9 的 metal workflow 只有 `emissive_map`，没有标量 `emissive`。

3. **`use_sim_time` 不能自己 declare**

   params 文件里带 `use_sim_time: true` 时 rclpy 会自动声明它，节点里再
   `declare_parameter('use_sim_time')` 直接抛 `ParameterAlreadyDeclaredException`，
   节点起不来。P2 的几个节点都没声明，原因就在这里。

4. **改 `/opt/ros/.../navigation_launch.py` 需要 root**

   该文件属 `root:root 555`，镜像里没有 sudo，补丁脚本写它直接
   `PermissionError`（容器因为 `set -e` 直接退出 —— 这是设计意图：宁可不启动）。
   解法：`docker-compose.yaml` 加 `user: root`。整条链路（Xvfb / x11vnc / gz /
   ros 节点）都不依赖非 root，没有副作用。

5. **`dock_database` 必须是绝对路径，且每次启动重新生成**

   写死在 yaml 里，一换出生点就全错。现在 `patch_upstream.py` 按
   `SPAWN_X/Y` + `DOCK_X/Y/YAW` 算出 odom 系坐标写到 `/tmp/dock_database.yaml`，
   `navigation.yaml` 的 `dock_database` 指向它。

6. **`filter_coef: 0.1` 在 3.9Hz 相机下收敛太慢**

   SimpleChargingDock 对检测到的 pose 做一阶低通，0.1 意味着每帧只吃 10%。
   软件渲染下相机只有 3.9Hz，光收敛就要十几帧。改成 0.5。

7. **`docking_threshold` 是相对 dock pose 的容差，不是「离桩多远」**

   上游默认 0.05，配合我们的桩几何（车头贴桩时误差应为 0）会让控制器一直
   往桩里推、轮子打滑、最后 `905 FAILED_TO_CONTROL`。给 0.12 让它提前判定。

8. **`navigate_to_staging_pose` 的 fallback 很坑**

   `dock_prestaging_tolerance: 0.5` 意味着导航失败但只要在 staging 0.5m 内
   就继续。日志里会打 `Successful navigation to staging pose`，看着像成功，
   实际位置和朝向都差得远。排查「为什么看不到 marker」时别被这句话骗了。

9. **`gz model -m X -p` 读顶层位姿，但 `pose/info` 里全是 lump 后的 link 名**

   排查时想抓机器人/桩的位姿，用 `gz model` 干净；用 `gz topic -e pose/info`
   会被 `base_footprint_fixed_joint_lump__led_front_visual_visual_3` 这种名字淹没。

10. **`ros2 topic echo` 的嵌套字段是 4 空格缩进**

    `grep '^  x:'` 抓不到 `pose.position.x`，白花一次排查。

---

## 六、已知限制

1. **停靠精度 ±10cm**，由 `docking_threshold: 0.12` 决定。真实产品应该在
   末端加触点/红外/电流检测（`use_stall_detection`）把最后一段收掉。
2. **桩在代价地图上不存在**（低于激光平面）。局部避障不会绕开它，
   全局规划会直接从桩上穿过去。P4 做任务调度时要显式处理（要么给桩一个
   costmap 层，要么把桩周围划成禁行区）。
3. **不支持从远处自动回充**。`navigate_to_staging` 默认 `false`，
   「从别处开回桩附近」属于任务调度层（P4）。强行打开会受 odom yaw 高估
   影响，需要留足 `dock_prestaging_tolerance`。
4. **仿真里电池不会真的充电**。Gazebo 的 `LinearBatteryPlugin` 只放电，
   所以 `sim_charging: true` 时由 `dock_manager` 按 `sim_charge_rate` 累加电量。
   **上真机必须把它关掉**，让 `BatteryState` 说话。
5. **ArUco 只认一个 marker、一个 id**，且要求基本正对（±45° 内）。侧向
   进入看不到。多桩场景需要扩展 id 分发。
6. **`low_ratio` / `low_voltage` 的取值来自 P1 实测**（12.47V 满电、
   `percentage` 字段不可信），真实电池的放电曲线不同，上真机要重标。
7. **视觉对准需要时间**：从背面开始扫一圈约 60s 墙钟（实时因子 0.25x）。
   真机上按 1.0x 算约 12.6s。

---

## 七、与其它阶段的关系

- **P1**（已完成）：`/battery_state` —— 低电判定的输入。P1 的结论
  「`percentage` 不可信、必须用 `charge/capacity`」在这里被直接采用。
- **P2**（已完成）：新增 `/dock/inhibit`，回充期间跟随节点让位。
  `twist_mux` 的 `dock` 源（prio 30）在这一阶段第一次真正被用到。
- **P4 视频存储 + 手机互联**：回充过程要能从手机看（桩前的画面最有说服力），
  录像要能按事件切片。
- **P5 跌倒检测**：与回充共享 `dock_manager` 的状态机骨架
  （低电触发 → 动作 → 完成 → 复位），告警链路可以照这个结构写。
