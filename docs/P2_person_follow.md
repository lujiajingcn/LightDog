# P2：人体检测 + 跟随 + 光照联动

场景 1（夜间去卫生间）的软件闭环。P1 解决了「灯能被控制」，P2 解决「灯知道该什么时候亮、
该跟着谁走」。

## 一、数据流

```
  /camera/color/image_raw ─┐
  /camera/depth/image_rect_raw ─┼─> lightdog_person_detector ─> /person/present  (Bool)
  /camera/color/camera_info ─┘                             └─> /person/position (PointStamped, base_link)
                                                                     │
                                       ┌─────────────────────────────┴──────────────┐
                                       ▼                                            ▼
                              lightdog_light_auto                        lightdog_follower
                                       │                                            │
                              /led/brightness                            /cmd_vel_follow
                                       │                                            │
                              （P1 材质下发）                                       │
                                                                            ┌───────┴────────┐
   Nav2: controller → velocity_smoother → /cmd_vel_smoothed ────────────────┤                │
                                                            lightdog_twist_mux（常驻仲裁）
                                                                            └───────┬────────┘
                                                                                    │ /cmd_vel_muxed
                                                                          collision_monitor
                                                                                    │ /cmd_vel
                                                                          ros_gz_bridge → gz 底盘
```

四个 P2 节点只通过上面这几个话题耦合。换检测器、换控制律、换照明策略都不牵连其他人。

**执行层不直接发 `/cmd_vel`。** 那个话题上本来就挤着 Nav2 的 `collision_monitor`、
`docking_server`、`command_timeout` 和 `ros_gz_bridge`，跟随指令直发进去会被它们的
空闲零速淹没；而把它们 kill 掉又会丢掉碰撞保护。所以中间插一层常驻仲裁
（`lightdog_twist_mux`，由 `sim/entrypoint.sh` 拉起，优先级 跟随 20 > Nav2 10 > 空闲），
仲裁结果再交给 `collision_monitor`，仍由它独占 `/cmd_vel`。

| 节点 | 职责 | 输入 | 输出 |
|---|---|---|---|
| `lightdog_person_detector` | 在画面里找到人，算出前向距离与左右偏移 | 相机 RGB-D + camera_info | `/person/present`、`/person/position`、`/person/debug_image` |
| `lightdog_light_auto` | 有人亮、人走后延迟渐灭 | `/person/present` | `/led/brightness` |
| `lightdog_follower` | 朝人走、保持 1.2m | `/person/position`、`/person/present` | `/cmd_vel_follow` |
| `lightdog_twist_mux` | `/cmd_vel` 多源仲裁（**常驻，不属于 P2**） | `/cmd_vel_smoothed`、`/cmd_vel_follow`、`/cmd_vel_dock` | `/cmd_vel_muxed`、`/cmd_vel_mux_state` |
| `lightdog_person_sim` | 让仿真里的「人」按路径走（**仅仿真**） | — | gz `set_pose` |

## 二、两个关键取舍

### 1. 这一版检测器不是 YOLO，是有意的

P2 要验证的是**控制闭环**（检测→跟随→照明）通不通，不是检测精度。仿真里的「人」
是我们自己造的几何体，颜色可控。所以这一版用 **HSV 颜色掩码 + 深度中位数**：

- 检测器本身零额外依赖：纯 numpy（不引 torch / ultralytics）。当前网络下装
  torch 要拉约 2GB，会让 P2 卡在装环境上。
  注意**整个功能栈并非零依赖**：`recorder` / `server` / `fall_detector` /
  `aruco_dock` 都用 opencv（镜像里装了 `python3-opencv`，含 contrib/aruco）。
  说「镜像没有 opencv」是早期版本留下的错误说法，已纠正。
- 结果确定：仿真人上衣是高饱和橙 `RGB(0.95, 0.33, 0.05)`，HSV 上 `H≈0.052`。
  环境里其余东西都能被挡掉：
  - 灰白墙：`S` 很低
  - 绿地：`H≈0.33`
  - 黄车身：`H≈0.14`
  - 暖白 LED 灯条 `RGB(1,0.92,0.80)`：`S≈0.20`，被 `s_min=0.45` 排除
  - 人的头发：特意做成深色，`S` 极低（原本用肤色会被误检，见第六节第 4 条）

人的几何尺寸（`models/person/model.sdf`）：腿 0~0.72 / 躯干 0.72~1.47 / 头 1.47~1.69，
总高 1.69m。躯干下沿压到 0.72 而不是按人体比例的 0.85，是**被相机高度逼的** ——
4wd 车型的相机光心离地 **0.3745m**（`depth_sensor_pose` z=0.255 + base_link 0.1195），
距离越近能看到的躯干越少，下沿太高的话人走到 0.8m 就只剩腿。
（早期文档写的 0.275m 只算了 camera 相对 base_link 的偏移，漏了 base_link 离地高度。）

**更换路径**：外部只依赖 `/person/present` 和 `/person/position` 两个话题。
换 YOLOv8-pose 时只替换 `lightdog_person_detector.py` 里的 `_detect()`，
follower 和 light_auto 不用动。真机 + P5 跌倒检测时才必须上真模型（那时需要姿态关键点）。

### 2. 跟随不用 Nav2

「跟随 ≠ 导航」。Nav2 需要地图 + `map` 系下的目标点，人一进屋、一消失，代价地图
就不知道往哪走；而且人钻的地方机器人未必过得去，把人的位置当导航目标会让它硬撞墙。

P2 的本质是**视觉伺服**：`/person/position` 直接给 `base_link` 下的前向距离和左右偏移，
P 控制算速度。Nav2 留给 P3 的自动回充（那是确定目标点 + 需要精确停靠的场景）。

## 三、检测原理与几何换算

1. RGB 图转 HSV（numpy 手写，不用 cv2），取橙色掩码
2. 列投影找纵向条带，允许 6px 空隙（容忍身上阴影断开），取像素最多的那条
3. **形状约束**：条带 bbox 要满足 `高 ≥ min_height_px` 且 `高 ≥ aspect_min × 宽`。
   这一条是实测加的 —— 只靠颜色挡不住仿真人的头部（见第六节第 4 条），
   但阈值不能卡太紧（第 5 条），当前 `aspect_min=0.55`。
4. 条带内取掩码像素的**深度中位数**作为距离 —— 用中位数而不是最小值，避免背景
   （墙、门框）把目标拉远
5. 像素中心 → 相机光学系 → `base_link`：

```
  X_cam = (u - cx) / fx * Z          # 右
  Y_cam = (v - cy) / fx * Z          # 下     ← 内参里 fy 就等于 fx，直接用 fx
  x_base = Z            + offset_x   # 前
  y_base = -X_cam       + offset_y   # 左
  z_base = -Y_cam       + offset_z   # 上
```

`offset` = 相机在 `base_link` 下的安装位。4wd 车型是 `(-0.065, 0, 0.255)`，
来自 `linorobot2_description/urdf/4wd_properties.urdf.xacro` 的 `depth_sensor_pose`。
rpy 为零，所以相机光轴就是 `base_link` 的 +X，不需要 TF 变换 —— 换车型改参数即可。

深度没有有效值时退化成 `0.62 × fx / bbox高度` 粗估（假设人身高 1.7m），
只为不中断闭环用。

## 四、运行

```bash
cd /e/study_lujiajing/WorkSpace_LightDog/LightDog/LightDog/sim
docker compose --env-file .env up -d          # 仿真栈（若没跑）

cd ..
bash sim/p2.sh start                          # 拉 P2 四个节点（仲裁器已由 entrypoint 常驻）
bash sim/p2.sh spawn                          # 在场景里生成「人」
```

浏览器打开 **http://127.0.0.1:6080/vnc.html**：应该能看到橙色上衣的人沿直线往返走，
机器人调整朝向跟上去。

观察数据：

```bash
bash sim/p2.sh watch        # present / position / cmd_vel 实时流
bash sim/p2.sh status       # 节点、仲裁状态、话题、相机帧率、亮度、人的位姿
bash sim/p2.sh det          # 读一次检测结果
bash sim/p2.sh log          # P2 节点日志
```

「车不动」时第一件事是看 **谁赢了仲裁**：

```bash
docker exec sim bash -lc 'source /opt/ros/jazzy/setup.bash && ros2 topic echo /cmd_vel_mux_state --once'
# follow → 跟随在控制；nav → Nav2 在控制；idle → 没有任何源在发
```

跑完一轮想把机器人放回出生点：

```bash
bash sim/p2.sh reset        # 实际是重启容器（原因见第五节第 7 条）
```

手动接管亮度（会被 light_auto 覆盖，先 stop）：

```bash
bash sim/p2.sh stop
bash sim/p1.sh led 0.5
```

## 五、验收结果（2026-11-01 实测）

### 1. 检测精度（静态标定）

把人用 `set_pose` 摆到 `base_link` 系若干已知点，隔 1.2s 读一次 `/person/position`，
同时用 `gz model -p` 读真值：

| 设定前向 | 实测前向 | 偏差 | 说明 |
|---|---|---|---|
| 1.00 m | 0.857 m | −0.143 | 差值是躯干半径 0.17 减去相机后移 0.065 |
| 1.50 m | 1.380 m | −0.120 | |
| 2.00 m | 1.650 m | −0.350 | 当时视线上有柱子干扰 |
| 2.00 m / 左 0.50 | 1.843 / +0.500 | −0.157 / +0.000 | 横向完全准确 |
| 2.00 m / 右 0.50 | 1.846 / −0.571 | −0.154 / −0.071 | |
| 1.50 m / 左 0.30 | 1.892 / +0.382 | +0.392 / +0.082 | 柱子遮挡的一次坏样本 |

**结论**：前向存在稳定的 −0.12~−0.16m 偏差，来源是**深度测的是人体表面、不是人体中心**
（躯干半径 0.17m 减相机后移 0.065m）。这是正确行为，不是误差 —— 跟随要的正是
「离身体表面多远」。横向误差 < 0.08m。1.4m 处单点复核：实测 0.922m / 1.317m
对理论 0.895m / 1.295m，偏差 22~27mm。

`/person/position` 的 `z` 是检测框中心的高度，近距离时躯干顶部会顶出画面上边缘，
所以 `z` 在近距离偏小 —— 只用它判断站/躺，不参与控制。

### 2. 跟随闭环

45s 连续采样（`/cmd_vel` 与 `/person/position` 同帧记录）：

| 检测前向 | 下发线速度 | 行为 |
|---|---|---|
| 2.67 → 2.43 m | +0.25 m/s | 满速追（限幅） |
| 1.95 → 1.54 m | +0.25 m/s | 追 |
| 1.40 m | +0.18 m/s | 进入比例段减速 |
| **1.20 m** | **0.00** | **收敛到目标距离，落在 ±0.15 死区** |
| 1.07 m | −0.15 m/s | 太近，后退（限幅 0.6×max） |
| 0.83 m | −0.15 m/s | 人继续贴上来，继续退 |
| 检测丢失 1.5s 后 | 0.00 | `person_timeout` 生效，停住 |

角速度在人横向偏移时按 `k_angular × atan2(y, x)` 输出，实测人在侧前方时
`w` 到 0.28~0.43 rad/s，`|theta| > 0.9` 时 `v` 归零只转不走。

### 3. 照明联动

| 事件 | 实测 |
|---|---|
| 检测到人 | 亮度在下一次 tick 就开始升，按 `dt/fade_time` 线性爬到 0.85 |
| 人离开 | 保持 0.85 → 5s（`fade_delay`）后开始降 → 2s（`fade_time`）降到 0.00 |
| 人回来 | 立刻开始升，无延迟 |

时间全部按 **sim time** 计。注意软件渲染下墙钟约是 sim time 的 4.5 倍，
所以在 noVNC 里看着像「渐变要八九秒」，实际仿真时间里正好 2s。

### 4. 其他实测数值

| 项 | 值 |
|---|---|
| 相机帧率 | 3.9 Hz（标称 30Hz，软件渲染 GL 限制） |
| 深度编码 | `32FC1`（float32 米），彩色 `rgb8` |
| 相机内参 | fx=343.16, cx=320.0, cy=240.0, hfov=1.5 rad |
| 检测掩码 | 1.4m 处 9962 px，1.0m 处 8526 px |
| 跟随到位精度 | 1.20 m（目标值），死区 ±0.15 |
| 仿真实时因子 | 约 0.22x～1.0x（随渲染负载波动） |
| 仲裁结果 | P2 全程 2763/2763 采样为 `follow`，Nav2 未抢到一次 |
| 碰撞拦停 | 0.2m/s 撞向 0.75m 外障碍，停在距障碍 4cm 处，零接触 |

### 5. 仲裁层验收（`/cmd_vel` 多源）

三条路径分别独立验证过，缺一条都不算数：

| 场景 | 操作 | 期望 | 实测 |
|---|---|---|---|
| 跟随优先 | P2 运行、人在视野内 | 仲裁为 `follow` | ✓ 2763/2763 采样 |
| Nav2 能接管 | 停掉 P2，向 `/cmd_vel_smoothed` 发 0.15 | 仲裁为 `nav`、车前进 | ✓ 6s 位移 0.32m |
| 碰撞能拦 | 恒定 0.2m/s 直行撞向障碍 | 停在障碍前、不接触 | ✓ 停在 4cm 外 |

第三条尤其重要：它证明仲裁没有把碰撞保护短路掉 —— 这正是 P2 早期
「杀掉 Nav2 让 follower 独占」那个权宜之计留下的最大隐患。

## 六、踩过的坑（都是实测踩出来的）

1. **`CameraInfo.k` 是 numpy 数组，`if not msg.k` 直接抛异常**
   `ValueError: truth value of an array with more than one element is ambiguous`，
   检测节点启动即崩。判空只能写 `len(msg.k) < 9`。

2. **出生点 (0.5, 0) 正对一根柱子，相机被 near clip 糊死**
   `turtlebot3_world` 地上铺了一组 3×3 的装饰圆柱（`ros_symbol`，半径 0.15，
   坐标 x,y ∈ {−1.1, 0, 1.1}）。从 (0.5,0) 出发，正前方 0.95m 处就是 (1.1,0) 那根，
   相机（车头前 0.065m）到柱面只有 **0.289m**，卡在 `near=0.3` 以内 →
   深度图整块 0、彩色画面被柱子糊死，检测率 0%。
   现在出生点改到 **(−1.8, 0.5)**：y ∈ [0.3, 0.7] 这一带是场景里唯一一条
   从 x=−2.4 到 2.4 全程无柱的长走廊。

3. **Nav2 抢 `/cmd_vel`，follower 的指令被零速淹没**（这一条后来演化成了仲裁层）
   `/cmd_vel` 上有 4 个发布者：`collision_monitor`、`docking_server`、`command_timeout`、
   `ros_gz_bridge`。Nav2 导航空闲时它们持续发零速，实测 follower 一直发 −0.15，
   机器人 6 秒只挪了 0.05m。
   跟随和导航本来就是互斥的运动模式，最初的权宜之计是 `p2.sh start` 里把 Nav2 的
   运动节点整个 `kill -9` —— 能用，但**把碰撞保护一起杀了**。
   现在换成标准架构：新增 `lightdog_twist_mux` 常驻仲裁（跟随 20 > 回充 30 >
   Nav2 10，带超时释放），`collision_monitor` 的输入由 `patch_upstream.py`
   改到 `/cmd_vel_muxed`，出口仍是 `/cmd_vel`。于是两个目标同时成立：
   P2 跟随期间实测 **2763/2763 个采样都是 follow 生效**，Nav2 一次没抢到；
   而碰撞保护一直在链路上（见第七节真机化的第 1 条实测）。

4. **仿真人的头被检测器当成「人」，导致人走了灯不灭**
   头部原本用肤色 `RGB(0.82, 0.68, 0.58)`，软件渲染的暗光下饱和度被抬到 0.45 以上、
   H 也落进橙色区间，在 2.5m 处有 574px（阈值 150），于是「人在远角站着不动」
   被持续报成 present=true，亮度永远不降。
   现在头部改成**深色头发**（S 极低），并在检测器里加了形状约束兜底。

5. **形状约束一开始卡太紧，把中近距离的人一起滤掉了**
   加了 `aspect_min=1.20` 后，头部误检没了，但人走到 0.83m 时躯干在画面里
   只剩一条横带（140×56px，比值 0.4），也被滤掉 —— 检测在 0.83m 就断。
   因为相机光心只有 0.3745m 高，越近能看到的躯干越少。现在放宽到 `aspect_min=0.55`，
   主要靠「深色头发」从颜色上解决误检。注意 `lightdog_person_detector.py` 里
   这两个参数的**代码默认值**曾经还是 1.20 / 36，不带 params 启动就会复发这个 bug；
   现已与 `p2_params.yaml` 对齐成 0.55 / 30。

6. **`p2.sh spawn` 把成功当失败，连开 5 次**
   gz Harmonic 实际输出 `Entity creation successful.`，脚本 grep 的是
   `created entity` / `Entity created`，永远不匹配。现在三种都认。

7. **`set_pose` 搬不动 dynamic 模型，会把车压进地面卡死**
   `gz service /world/<w>/set_pose` 给 `z=0` 时轮子直接压进地面：pose 显示
   (−1.8, 0.5, 0)，但之后 gz 侧**确实收到了** `linear.x: 0.2`（`gz topic -e` 验证），
   机器人却一动不动，base_footprint 在 −0.0138 处缓慢上浮。给 `z=0.15` 让它
   从空中落下也一样卡住。所以 `p2.sh reset` 干脆改成重启容器。

8. **颜色渐变按 tick 计数，时长会随负载漂移**
   原实现是每 tick 加固定步长 `1/(step_hz × fade_time)`，前提是 timer 真能跑满
   `step_hz`。实测 timer 会丢 tick，设计 2s 的渐变会跑成 4~9s。改成按 `dt` 时间积分
   （`max_delta = dt / fade_time`）后，渐变时长与 tick 频率解耦。

9. **`pkill -f` 在这个容器里不生效**
   试过 `pkill -f controller_server` 等，进程纹丝不动，必须
   `ps | grep | awk '{print $1}'` 拿到 PID 再 `kill -9`。`p2.sh` 的 `kill_p2` 按这个来。

10. **双向桥接的方向符容易看错**
    `gazebo.launch.py` 里是 `/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist`，
    两个 `@` 是**双向**（`]` 才是 ROS→GZ 单向）。双向桥接会让 gz 侧的零速回灌到 ROS，
    排查运动问题时先确认谁在发。

11. **仲裁器自己踩的两个坑（写的时候就该想到）**
    - **状态话题只在「切换」时发布** → `ros2 topic echo --once` 永远等不到
      （除非刚好撞上切换那一刻），排查「为什么车不动」时全是超时，反而更迷惑。
      改成每次 tick 都发一条（`String` 很短，20Hz 的开销可以忽略）。
    - **超时用 sim time** → 实时因子 0.22x 时，设计的 0.6s 窗口实际是 2.7s 墙钟，
      上游进程已经死了还在转发它的最后一条指令。改成 `time.monotonic()`。

12. **修改 `navigation.yaml` 必须走补丁脚本，不能手改**
    该文件在 `install/` 空间，容器重建后会被上游覆盖；手改一次看似生效，
    下次 `docker compose restart` 就悄悄回退了，而症状是「车突然不响应导航」。
    `sim/patch_upstream.py` 每次启动都跑，幂等且锚点找不到会直接报错退出。

## 七、已知限制

1. **相机太矮导致近距离看不见人**：相机光心只有 0.3745m 高，人走到 0.5m 以内时视野上沿
   只到 0.63m，躯干（0.72~1.47m）完全出画面，只剩深蓝的腿 —— 颜色方案在这个距离上
   必然失效。`stop_distance=0.55` 因此实际不会触发，真正兜底的是 `person_timeout`
   （丢人 1.5s 后停车）。真机换成 YOLO 后这个盲区依旧存在，需要靠低机位构图或
   广角镜头解决。
2. **颜色依赖**：检测器认的是橙色上衣。换衣服、环境光偏色都会失效 —— 这是仿真
   占位实现，不是产品方案。
3. **只处理画面里最大的一块橙色**：同时有两个人会跟最近/最大的那个，没有多目标跟踪
   和身份保持（ID 会跳）。多人与身份保持属于 P4/P5 的范围。
4. **避障已接入，但只覆盖「前方碰撞」**
   `/cmd_vel` 现在由 `collision_monitor` 独占，跟随指令也要先过它。
   实测：恒定 0.2 m/s 直行撞向 0.75m 外的静态障碍，车在距障碍 **4cm** 处完全停住
   （连续 3 秒零位移），没有发生接触。用的还是上游的 `FootprintApproach`
   polygon（`time_before_collision: 1.2s`），配置没有改动。
   局限在于它只管正前方的预测碰撞：侧向擦碰、低矮障碍（激光扫不到的高度）
   都不在覆盖范围内，也不做绕行 —— 只会停，不会躲。
5. **人迎面走来时机器人只会后退**：人的路径是往返，人往回走时会一路把机器人逼退，
   退到墙角就顶着不动。真实产品应该做侧向让位，P2 的 P 控制没有这一层。
   （注意：后退方向如果撞到障碍，`collision_monitor` 同样会介入拦停。）
6. **仿真「人」是 static 模型**：靠 `set_pose` 搬运（5Hz，每次起一个 `gz service`
   子进程），没有行走动画，也不会被机器人推动。视觉上是个会滑动的圆柱人，
   够验证检测/跟随，但不代表真人的运动特征。
7. **软件渲染下仿真跑不满实时**：实时因子约 0.22x，相机 3.9Hz。所有节点都用
   `use_sim_time: true`，所以逻辑时长是对的，但操作手感比真机慢。
   仲裁器的超时窗口是**唯一**按墙钟算的（它判断的是「上游还在不在发包」，
   与仿真快慢无关），其余时间逻辑一律用 sim time。
8. **跟随期间 Nav2 拿不到控制权**：跟随节点即使人丢了也在持续发零速，
   所以 `follow` 源一直「活跃」，优先级压着 Nav2。要在跟随与导航之间切换，
   必须停掉跟随节点（`enable_follow:=false`）或改优先级，不能指望它自己让位。

## 八、与路线图上其它阶段的关系

- **P1**（已完成）：`/led/brightness`、`/battery_state` —— P2 直接复用
- **P3 自动回充**：用 Nav2 `docking_server` + 底座 ArUco。仲裁层的 `/cmd_vel_dock`
  入口（prio 30）已经预留好，回充控制器接到那个话题上就会自动压过跟随；
  跟随期间要回充则需先 `enable_follow:=false`
- **P5 跌倒检测**：需要姿态关键点，届时把检测器换成 YOLOv8-pose / MediaPipe，
  本文件的接口不变
