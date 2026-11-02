# P1：LED 灯组与电池模型

对应 LightDog 计划书的「可以调节 led 亮度」与「电量低时自动去充电」两项能力的数据基础。
本阶段只打通**感知/控制接口**，不涉及跟随、回充逻辑（那是 P2/P3）。

## 一、改了哪些文件

上游 `linorobot2` 源码**不入库**（见 `docs/linorobot2_upstream.md`），我们对它的影响只有三层：
覆盖层（URI/xacro）、启动层（launch）、以及启动时打在镜像 install 副本上的 1 个补丁。

| 文件 | 作用 |
|---|---|
| `lightdog/urdf/led.urdf.xacro` | 5 个 LED link：四周主灯条 ×4 + 底部指示灯 ×1，亮度用 visual emissive 表达 |
| `lightdog/urdf/battery.urdf.xacro` | `LinearBatteryPlugin`，3S 锂电 / 5Ah / 15W 负载 |
| `lightdog/urdf/lightdog_4wd.urdf.xacro` | **覆盖层**：include 上游 `4wd.urdf.xacro` + 调用上面两个宏（2wd 版本同目录另存） |
| `lightdog/launch/lightdog_sim.launch.py` | include 上游 `gazebo.launch.py`，只覆盖 `urdf` 参数；加 `material_color`、电池两条 bridge；拉起 LED 节点 |
| `lightdog/lightdog_led.py` | 亮度控制器节点 |
| `lightdog/config/led_visuals.yaml` | visual 名与配色参数（节点参数文件） |
| `sim/patch_upstream.py` | 启动时幂等修补镜像内 `description.launch.py` 的 `robot_description` bug |

### 为什么不是改上游文件

`gazebo.launch.py` 本身已声明 `urdf` 启动参数，所以换 URDF 不用动它一行；
xacro 的 `<xacro:include>` 允许被包含文件自带 `<robot>` 根节点，内容并入外层，
所以覆盖层能"上游基座 + 我们的扩展"组合出一个 URDF。这样上游改动压到只剩 1 处编译期 bug。

### 容器侧

镜像里 `linorobot2_description` 是构建时 COPY 进 `/home/ros/linorobot2_ws/install/` 的**实体副本**（不是 symlink 安装），
改宿主机源码不生效、也不值得为一个 xacro 重建 8GB 镜像。现在的做法是：
`lightdog/` 挂到工作区 `src/` 下，容器启动时 `colcon build`（约 2 秒，非 symlink 也可，因为每次重建容器都重编），
补丁打在 install 副本上。改 `lightdog/` 下的文件后 `docker compose up -d --force-recreate` 即生效。

## 二、接口

| 话题 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/led/brightness` | `std_msgs/Float32` | 订阅 | 主灯亮度 0.0~1.0，超范围自动截断 |
| `/led/indicator` | `std_msgs/Float32` | 订阅 | 底部指示灯亮度，默认 0.06 常亮 |
| `/world/turtlebot3_world/material_color` | `ros_gz_interfaces/MaterialColor` | 发布 | 经 bridge 转入 gz，改 visual 的 emissive |
| `/battery_state` | `sensor_msgs/BatteryState` | 发布（来自 gz） | 电压/电流/电量 |

用法：

```bash
# 主灯全亮
docker exec sim bash -lc 'source /opt/ros/jazzy/setup.bash && \
  ros2 topic pub --once /led/brightness std_msgs/msg/Float32 "{data: 1.0}"'

# 熄灭
docker exec sim bash -lc 'source /opt/ros/jazzy/setup.bash && \
  ros2 topic pub --once /led/brightness std_msgs/msg/Float32 "{data: 0.0}"'
```

节点日志走容器 stdout（`docker logs sim | grep lightdog_led`），`ros2 launch` 不单独落文件。

## 三、怎么看效果

**一句话：浏览器开 `http://127.0.0.1:6080/vnc.html`，画面里就是 4wd 车身，改亮度能直接看到灯条明暗变化。**

前提是容器已经起来：

```bash
cd sim && docker compose --env-file .env up -d
```

### 画面里有什么

容器内 `gz` 跑的是 `-s`（纯服务端）模式，默认不带 3D 窗口 —— 只开 noVNC 看到的是 RViz 的
SLAM 图，看不见车身。所以 entrypoint 会额外拉起一个 GUI 客户端（`SHOW_GUI=1`，默认开），
并做两件事让它开箱可用：

1. 用 `sim/gui.config` 替换默认布局 —— 默认配置把「组件检查器 + 实体树」dock 在右侧，
   吃掉约 40% 宽度；我们的配置只留 3D 视图 + 播放控制 + 统计，窗口铺满 1920x1080。
   注意这个文件**不能**有 `<gui>` 根节点，gz-gui 8 期望 `dialog`/`window`/`plugin` 直接在顶层，
   套一层会被静默忽略并回退默认布局。
2. 启动后调 `/gui/follow` 把相机锁到机器人，跟着它走，不会一进来找不到车。

GUI 必须在 RViz **之后**启动 —— Xvfb 没有窗口管理器，窗口靠创建顺序叠放，后建的在上层。
这条顺序在 `entrypoint.sh` 里有注释，改动时别调换。

### 调亮度 / 读电池

`sim/p1.sh` 是在宿主机 Git Bash 里跑的封装（内部 `docker exec` 到容器）：

```bash
bash sim/p1.sh status          # 容器/节点/话题/GUI/车型 概览
bash sim/p1.sh led 0.8         # 主灯亮度 0~1
bash sim/p1.sh led off         # 主灯全灭
bash sim/p1.sh indicator 0.5   # 底部指示灯
bash sim/p1.sh sweep           # 0→1 分段扫一遍，肉眼看灯条渐变
bash sim/p1.sh battery         # 读 /battery_state
```

不想用脚本也可以直接打：

```bash
docker exec sim bash -lc 'source /opt/ros/jazzy/setup.bash && \
  ros2 topic pub --once /led/brightness std_msgs/msg/Float32 "{data: 1.0}"'
```

**看画面时的注意点：**

- 灯条是贴在车身上的自发光面片，**只从侧面看得到**。相机默认在左前上方，能同时看到前灯条和左灯条。
- 底部指示灯在底盘下方（z≈0.05），从上方俯视看不见，属正常。要确认它就把相机拖到地面以下，或直接看 gz 话题数值。
- `SHOW_GUI=0` 可关掉 GUI 省 CPU（软件渲染下 GUI 占一档渲染开销）；只做数据链路验证时建议关。
- 改 `SHOW_GUI` 或 `gui.config` 后要 `docker compose --env-file .env up -d --force-recreate` 才生效。

## 四、实测验证（ROBOT_BASE=4wd）

| 项 | 证据 |
|---|---|
| visual 索引 | SDF 与 `material_color` 抓包一致：`led_front_visual_visual_3`、`_left_5`、`_rear_6`、`_right_7`、`indicator_4` |
| 启动态 | 主灯 emissive 全 0，指示灯 `r:0.06 g:0.015 b:0.006` |
| 亮度 0.8 | 四条主灯 `emissive r:0.8 g:0.736 b:0.64` = 0.8 × (1.0, 0.92, 0.80)，指示灯维持 0.06 |
| 电池 | `voltage 12.477V`、`current 1.2022A`、`charge 4.994/5.0Ah`、`power_supply_status 2`(DISCHARGING) |
| 电池 gz topic | `/model/linorobot2_4wd/battery/lightdog_battery/state` |

## 五、已知限制（重要）

**1. 渲染层未做逐像素验证。** 数据链路（ROS → bridge → gz topic → 正确的 emissive 数值）已完整验证，
但没有确认 gz 渲染器把 emissive 真实画出来。原因是仿真以无头 + 软件渲染运行，
对准直径 1.2cm 的灯条做像素比对不现实。
要人眼确认：浏览器开 `http://127.0.0.1:6080/vnc.html`，拖近看车身侧面。

**2. Gazebo 不支持 URDF 光源。** 已实测 `gz-sim 8.15`：`<light>` 标签无论写在 link 级还是 model 级，
都会被静默丢弃（日志 `XML Element[light] ... not defined in SDF`），不生成可渲染光源。
所以 LED 只用 visual 自发光表达，**不会照亮地面**。P2 做夜间场景时若要真实照明，
得走「world 里定义 light + 改 pose 跟随机器人」的路子。

**3. `/battery_state` 要等几分钟才有数据。** bridge 先于 gz 的电池发布者启动，
gz-transport 需要重新发现才能连上，实测约 3~5 分钟后开始出数。不是配置错误。

**4. `percentage` 字段不可信。** 实测 `charge` 已近满（99%+）时，`percentage` 报 `0.381`。
P3 做低电回充判定时**必须用 `charge / capacity` 或 `voltage` 阈值**，不要用 `percentage`。

**5. visual 名与车型绑定。** Gazebo 把 fixed link 全部 lump 到 `base_footprint`，命名规则为
`base_footprint_fixed_joint_lump__<link>_visual_<N>`。因为 link 名本身已带 `_visual`，出现 `_visual_visual_` 双写。
`N` 按 link 名字母序编号，**换车型序号会变**。当前 `.env` 是 `ROBOT_BASE=4wd`，用 `_3/_5/_6/_7`；
2wd 需换成 `_4/_6/_7/_8`（`config/led_visuals.yaml` 里两套值都写好了，切换时对调注释）。
若改了 LED link 的数量或命名，必须重新从 SDF 取出真实名字。
另注意 4wd 的机体尺寸与 2wd 差别大（`base_height` 0.129 vs 0.003），LED 安装高度由宏参数跟随，但**贴片位置观感需人眼确认**。

## 六、这个上游 bug 的来龙去脉

`description.launch.py` 原本把 `robot_description` 直接交给 `Command(['xacro ', ...])`，
launch 会尝试把整个 URDF 文本当 YAML 解析。URDF 里任何一行出现 `:`（比如 XML 注释里）就会报
`Unable to parse the value of parameter robot_description as yaml`，容器起不来。
已在 Jazzy 上实测确认这是上游自身问题，不是我们的配置错误。

修法是 `ParameterValue(..., value_type=str)`，由 `sim/patch_upstream.py` 在容器启动时幂等修补镜像内副本。
**锚点找不到时脚本会报错退出让容器起不来**，而不是静默跑一个坏配置 —— 这样上游改版能立刻被发现。

## 手动灯光覆盖（后补）

`lightdog_light_auto` 原来只能按人体检测自动调光，语音说「把灯打开」没法生效 ——
因为下一拍自动调光就会按「没人」把它灭掉。现在加了一路覆盖：

```
/light/mode  String   auto | on | off
```

- `on` / `off`：**无条件**覆盖目标亮度（有人在也照做），直到显式切回 `auto`。
- `auto`：回到「跟随人体检测 + `fade_delay` 淡出」的默认行为。
- 覆盖状态是节点内存里的一个变量，**不写盘**：重启回到 `auto`。这是刻意的 ——
  深夜重启后如果还记着「有人手动关过灯」，那就再也不亮了。

三个入口都能写这个话题：`system.sh light on|off|auto`、语音「开灯/关灯/灯光自动」、
手机端。亮度本身仍然走原来那条按时间积分的渐变（`fade_time`），覆盖只改目标值，
所以手动切换也是平滑的，不会闪。

> 上真机前请先过一遍 **[hardware_readiness.md](hardware_readiness.md)**（参数清单 / 失效矩阵 / 真机上不能跑的节点）。
