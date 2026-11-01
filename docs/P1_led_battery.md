# P1：LED 灯组与电池模型

对应 LightDog 计划书的「可以调节 led 亮度」与「电量低时自动去充电」两项能力的数据基础。
本阶段只打通**感知/控制接口**，不涉及跟随、回充逻辑（那是 P2/P3）。

## 一、改了什么

| 文件 | 改动 |
|---|---|
| `linorobot2_description/urdf/lightdog/led.urdf.xacro` | 新增。5 个 LED link：四周主灯条 ×4 + 底部指示灯 ×1 |
| `linorobot2_description/urdf/lightdog/battery.urdf.xacro` | 新增。`LinearBatteryPlugin`，3S 锂电 / 5Ah 参数 |
| `linorobot2_description/urdf/robots/2wd.urdf.xacro` | include 并调用上述两个宏 |
| `linorobot2_description/urdf/robots/4wd.urdf.xacro` | 同上 |
| `linorobot2_description/launch/description.launch.py` | **修 bug**：`robot_description` 包 `ParameterValue(value_type=str)` |
| `linorobot2_gazebo/launch/gazebo.launch.py` | 新增两条 bridge：`material_color`（ROS→GZ）、电池 state（GZ→ROS 并 remap 到 `/battery_state`） |
| `linorobot2/docker/docker-compose.yaml` | 新增 4 个挂载：description 的 `urdf`/`launch`、以及本目录 `lightdog/` |
| `linorobot2/docker/slam_start.sh` | 启动 SLAM 后拉起 LED 节点 |
| `lightdog/lightdog_led.py` | 新增。亮度控制器 |
| `lightdog/config/led_visuals.yaml` | 新增。visual 名与配色参数 |

### 为什么用挂载覆盖 install 目录

镜像里的 `linorobot2_description` 是构建时 COPY 进 `/home/ros/linorobot2_ws/install/` 的**实体副本**（不是 symlink 安装），
改宿主机源码不会生效，也不值得为一个 xacro 改动重建 8GB 镜像。
因此把宿主机目录直接挂到 install 的 share 路径上，改完 `docker compose up -d --force-recreate gazebo` 即生效。
这是 P1 阶段的权宜做法，正式开发应改为挂 `src/` + `colcon build --symlink-install`。

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

节点日志在容器内 `/tmp/lightdog_led.log`。

## 三、实测验证

| 项 | 证据 |
|---|---|
| URDF 展开 | `xacro 2wd.urdf.xacro` → 577 行，5 个 `led_*_link` 齐全 |
| SDF 转换 | `gz sdf -p` 零错误，5 个 visual 均存在 |
| 亮度 0.5 | gz 侧 `emissive r:0.5 g:0.46 b:0.40` = 0.5 × (1.0, 0.92, 0.80) |
| 亮度 1.0 | gz 侧 `emissive r:1 g:0.92 b:0.8` |
| 指示灯光 0.5 | 节点日志 `indicator brightness -> 0.50` |
| 电池 | `voltage 12.464V`、`current 1.203A`、`charge 4.969/5.0Ah`、`DISCHARGING` |

## 四、已知限制（重要）

**1. 渲染层未做逐像素验证。** 数据链路（ROS → bridge → gz topic → 正确的 emissive 数值）已完整验证，
但没有确认 gz 渲染器把 emissive 真实画出来。原因是仿真以 `gui:=false` 无头运行，
而软件渲染下启动 GUI 后再对准直径 1.2cm 的灯条做像素比对不现实。
要人眼确认：在容器内 `DISPLAY=:200 gz sim -g` 打开 GUI，拖近看车身侧面。

**2. Gazebo 不支持 URDF 光源。** 已实测 `gz-sim 8.15`：`<light>` 标签无论写在 link 级还是 model 级，
都会被静默丢弃（日志 `XML Element[light] ... not defined in SDF`），不生成可渲染光源。
所以 LED 只用 visual 自发光表达，**不会照亮地面**。P2 做夜间场景时若要真实照明，
得走「world 里定义 light + 改 pose 跟随机器人」的路子。

**3. `/battery_state` 要等几分钟才有数据。** bridge 先于 gz 的电池发布者启动，
gz-transport 需要重新发现才能连上，实测约 3~5 分钟后开始出数。不是配置错误。

**4. `percentage` 字段不可信。** 实测 `charge 4.969/5.0Ah`（即 99.4%）时，`percentage` 报 `0.381`。
P3 做低电回充判定时**必须用 `charge / capacity` 或 `voltage` 阈值**，不要用 `percentage`。

**5. visual 名与车型绑定。** Gazebo 把 fixed link 全部 lump 到 `base_footprint`，命名规则为
`base_footprint_fixed_joint_lump__<link>_visual_<N>`。因为 link 名本身已带 `_visual`，出现 `_visual_visual_` 双写。
`N` 按 link 名字母序编号，**换车型序号会变**。当前 `.env` 是 `ROBOT_BASE=2wd`，用 `_4/_6/_7/_8`；
4wd 需换成 `_3/_5/_6/_7`（`config/led_visuals.yaml` 里两套值都写好了，取消注释即可）。
若改了 LED link 的数量或命名，必须重新 `gz sdf -p` 取出真实名字。

## 五、下次改 URDF 前必读

`description.launch.py` 原本把 `robot_description` 直接交给 `Command(['xacro ', ...])`，
launch 会尝试把整个 URDF 文本当 YAML 解析。URDF 里任何一行出现 `:`（比如 XML 注释里）就会报
`Unable to parse the value of parameter robot_description as yaml`，容器起不来。
已修成 `ParameterValue(..., value_type=str)`。**如果不走挂载、用了镜像内原始 launch 文件，这个 bug 依旧存在。**
