#!/usr/bin/env python3
"""给上游 linorobot2 打「必需的最小」补丁。

补丁 1：description.launch.py 的 robot_description 类型错误
------------------------------------------------------------------
现象（ROS 2 Jazzy + launch_ros 新版）：

    [ERROR] [launch]: Caught exception in launch: Unable to parse the value of
    parameter robot_description as yaml. If the parameter is meant to be a string,
    try wrapping it in launch_ros.parameter_descriptions.ParameterValue(value, value_type=str)

原因：`Command(['xacro ', ...])` 的替换结果被 launch_ros 按 YAML 解析，而 xacro 输出以
`<?xml` 开头，必然解析失败。上游自身的写法在 Jazzy 上就是坏的（已实测）。

修法即报错信息里给的标准写法：套一层 ParameterValue(..., value_type=str)。

补丁 2：navigation.yaml 的 collision_monitor 输入话题
------------------------------------------------------------------
上游的 Nav2 链路默认是

    controller_server -> /cmd_vel_nav -> velocity_smoother -> /cmd_vel_smoothed
        -> collision_monitor -> /cmd_vel -> 底盘

而我们的跟随/回充指令要走多源仲裁（`lightdog_twist_mux`）。仲裁必须插在
collision_monitor **之前**，否则绕过了碰撞保护。所以把它的输入话题从
`cmd_vel_smoothed` 改成 `cmd_vel_muxed`：

    velocity_smoother -> /cmd_vel_smoothed ─┐
    跟随              -> /cmd_vel_follow   ─┼-> twist_mux -> /cmd_vel_muxed
    回充              -> /cmd_vel_dock     ─┘        |
                                          collision_monitor -> /cmd_vel -> 底盘

`cmd_vel_out_topic` 保持 `cmd_vel` 不动 —— 让 collision_monitor 继续做 `/cmd_vel`
的唯一发布者，它和 command_timeout 那套「超时发零」的保险丝逻辑才继续成立。

补丁 3：docking_server 的 cmd_vel 出口改到仲裁入口
------------------------------------------------------------------
`navigation_launch.py` 里 docking_server 的 remappings 只有 tf，没有 cmd_vel，
所以它默认直接发 `/cmd_vel` —— 绕过仲裁也绕过 collision_monitor。

上游这么写是有道理的（docking 要贴到桩上，碰撞监测会把最后的接近拦停），
但我们的桩体是刻意压在激光扫描平面以下做的（见 lightdog/models/dock/model.sdf），
collision_monitor 本来就看不见它，不存在误拦。所以把 docking 也纳入仲裁：
出口改成 `cmd_vel_dock`，走 prio 30 的最高优先级。

顺带统一了「任何速度都必须过碰撞保护」这条不变量。

补丁 4：navigation.yaml 的 docking_server 参数
------------------------------------------------------------------
1) `docking_threshold: 0.05` → `0.12`。我们的桩体前后纵深约 0.10m，机器人车头
   贴住 marker 板时 base_link 到 dock 原点正好 0.30m，0.05 会让控制器一直往
   桩里推（轮子打滑、超时失败）。0.12 让它在物理接触（0.298m 处）之前就判定到位。
2) `external_detection_translation_*` / `rotation_*` 全部归零：安装偏差已经由
   `lightdog_aruco_dock` 在发布前用矩阵补偿掉了，这里再加一层只会重复计算。
3) `dock_database` 指向 `/tmp/dock_database.yaml`（由本脚本按环境变量生成，
   见下），不再用 navigation.yaml 里写死的实例坐标。

关于 `docking_threshold` 的取值：它是「机器人实际位姿与 dock pose 的距离容差」，
而 dock pose 的语义是**机器人停靠时应该在的位姿**（不是桩的位姿 —— 见下面
generate_dock_database 的说明）。我们的桩前后纵深 0.10m，车头贴住桩体时
base_link 距桩原点正好 0.30m，所以把 dock pose 定在「桩原点 +0.30m、朝向反过来」，
阈值给 0.12：控制器开到目标点 12cm 内就算成功，而物理碰撞会在 0.298m 处把车挡住，
两者不冲突。上游默认 0.05 会一直往桩里推（轮子打滑 + 超时 905）。

`staging_x_offset` 保持上游默认 -0.7：它相对 dock pose 再沿 -x 退 0.70m，
也就是桩前方 1.0m 处。这个距离下 marker（0.084m）在 640px 画面里约 28px，
是能稳定检出 4x4 marker 的下限附近；再远就得换更大的 marker。

生成 dock_database.yaml
------------------------------------------------------------------
桩在世界里的位置由 `sim/.env` 的 DOCK_X/DOCK_Y/DOCK_YAW 给出，而 docking_server
的 fixed_frame 是 `odom`，odom 原点就是机器人的出生点（SPAWN_X/SPAWN_Y）。
所以写进数据库的坐标必须是 **世界坐标减去出生点** —— 换个出生点就必须重算，
写死在 yaml 里迟早对不上。本脚本每次都按当前环境变量重新生成。

该脚本幂等：
  - 已打过补丁 → 打印 SKIP，退出 0
  - 找不到预期字符串（说明上游升级改了这里的代码）→ 打印错误，退出 1
    （故意让容器起不来，而不是静默跑一个坏配置）
"""

import math
import os
import pathlib
import sys

WS_INSTALL = pathlib.Path('/home/ros/linorobot2_ws/install')

DESCRIPTION_LAUNCH = (
    WS_INSTALL / 'linorobot2_description' / 'share' / 'linorobot2_description'
    / 'launch' / 'description.launch.py'
)

NAVIGATION_YAML = (
    WS_INSTALL / 'linorobot2_navigation' / 'share' / 'linorobot2_navigation'
    / 'config' / 'navigation.yaml'
)

# 上游 nav2_bringup 的 navigation_launch.py（系统空间，不在工作区里）
NAVIGATION_LAUNCH = pathlib.Path('/opt/ros') / os.environ.get('ROS_DISTRO', 'jazzy') \
    / 'share' / 'nav2_bringup' / 'launch' / 'navigation_launch.py'

DOCK_DB = pathlib.Path('/tmp/dock_database.yaml')


def patch_description_launch() -> int:
    """robot_description 必须包 ParameterValue(str)。"""
    target = DESCRIPTION_LAUNCH
    if not target.is_file():
        print(f'[patch] FAIL: 目标文件不存在: {target}')
        return 1

    text = target.read_text(encoding='utf-8')

    old = "                    'robot_description': Command(['xacro ', LaunchConfiguration('urdf')])"
    new = (
        "                    'robot_description': ParameterValue(\n"
        "                        Command(['xacro ', LaunchConfiguration('urdf')]),\n"
        "                        value_type=str)"
    )
    import_anchor = 'from launch_ros.substitutions import FindPackageShare'
    import_line = 'from launch_ros.parameter_descriptions import ParameterValue'

    if 'ParameterValue(' in text and old not in text:
        print('[patch] SKIP: description.launch.py 已打过补丁')
        return 0

    if old not in text:
        print('[patch] FAIL: description.launch.py 没找到预期的那一行，上游可能改了实现。')
        print(f'[patch]       位置: {target}')
        return 1

    if import_line not in text:
        if import_anchor not in text:
            print('[patch] FAIL: 找不到 import 锚点，无法安全插入 import。')
            return 1
        text = text.replace(import_anchor, f'{import_anchor}\n{import_line}', 1)

    target.write_text(text.replace(old, new, 1), encoding='utf-8')
    print('[patch] OK: description.launch.py  robot_description -> ParameterValue(str)')
    return 0


def _apply_pairs(text: str, pairs) -> tuple:
    """按 (old, new, 标签) 逐条替换。返回 (新文本, 错误信息或 None, 改动条数)。"""
    changed = 0
    for old, new, tag in pairs:
        if new in text:
            continue                      # 这一条已经打过了
        if old not in text:
            return text, (f'{tag}: 找不到锚点 {old!r}（上游可能改了实现）', changed), changed
        text = text.replace(old, new, 1)
        changed += 1
        print(f'[patch]   - {tag}')
    return text, None, changed


def patch_navigation_yaml() -> int:
    """collision_monitor 的输入 + docking_server 的参数。"""
    target = NAVIGATION_YAML
    if not target.is_file():
        print(f'[patch] FAIL: 目标文件不存在: {target}')
        return 1

    text = target.read_text(encoding='utf-8')

    # 先断言出口话题没被动过：collision_monitor 必须继续独占 /cmd_vel
    if 'cmd_vel_out_topic: "cmd_vel"' not in text:
        print('[patch] FAIL: navigation.yaml 的 cmd_vel_out_topic 不是 "cmd_vel"，'
              '需要重新评估整条命令链路。')
        return 1

    pairs = [
        (
            'cmd_vel_in_topic: "cmd_vel_smoothed"',
            'cmd_vel_in_topic: "cmd_vel_muxed"',
            'collision_monitor 输入 -> cmd_vel_muxed',
        ),
        (
            '    dock_plugins: [\'simple_charging_dock\']',
            '    dock_plugins: [\'simple_charging_dock\']\n'
            f'    dock_database: "{DOCK_DB}"',
            'docking_server 加载 dock_database（由本脚本生成）',
        ),
        (
            '      docking_threshold: 0.05',
            '      docking_threshold: 0.12',
            'docking_threshold 0.05 -> 0.12（dock pose 已定义在车头贴桩处，见文件头）',
        ),
        (
            '      external_detection_translation_x: -0.18',
            '      external_detection_translation_x: 0.0',
            'external_detection_translation_x -> 0（偏移已由 aruco 节点补偿）',
        ),
        (
            '      external_detection_rotation_roll: -1.57',
            '      external_detection_rotation_roll: 0.0',
            'external_detection_rotation_roll -> 0',
        ),
        (
            '      external_detection_rotation_pitch: -1.57',
            '      external_detection_rotation_pitch: 0.0',
            'external_detection_rotation_pitch -> 0',
        ),
        (
            '      filter_coef: 0.1',
            '      filter_coef: 0.5',
            'filter_coef 0.1 -> 0.5（软件渲染下相机只有 3.9Hz，0.1 收敛太慢）',
        ),
    ]

    text, err, changed = _apply_pairs(text, pairs)
    if err:
        print(f'[patch] FAIL: {err}\n[patch]       位置: {target}')
        return 1

    if changed == 0:
        print('[patch] SKIP: navigation.yaml 已全部打过补丁')
        return 0

    target.write_text(text, encoding='utf-8')
    print(f'[patch] OK: navigation.yaml（{changed} 处）')
    return 0


def patch_navigation_launch() -> int:
    """docking_server 的速度出口从 /cmd_vel 改到 /cmd_vel_dock。"""
    target = NAVIGATION_LAUNCH
    if not target.is_file():
        print(f'[patch] FAIL: 目标文件不存在: {target}')
        return 1

    text = target.read_text(encoding='utf-8')

    # 锚点取整段 Node 定义，保证只命中 docking_server 那一处（文件里还有
    # controller_server / behavior_server 也是 remappings + cmd_vel 的写法）
    old = (
        "            Node(\n"
        "                package='opennav_docking',\n"
        "                executable='opennav_docking',\n"
        "                name='docking_server',\n"
        "                output='screen',\n"
        "                respawn=use_respawn,\n"
        "                respawn_delay=2.0,\n"
        "                parameters=[configured_params],\n"
        "                arguments=['--ros-args', '--log-level', log_level],\n"
        "                remappings=remappings,\n"
        "            ),"
    )
    new = old.replace(
        '                remappings=remappings,\n',
        "                remappings=remappings + [('cmd_vel', 'cmd_vel_dock')],\n",
    )

    if new in text:
        print('[patch] SKIP: navigation_launch.py 的 docking_server 已打过补丁')
        return 0

    if text.count(old) != 1:
        print(f'[patch] FAIL: navigation_launch.py 里 docking_server 锚点出现 '
              f'{text.count(old)} 次（预期 1），上游可能改了实现。')
        print(f'[patch]       位置: {target}')
        return 1

    target.write_text(text.replace(old, new, 1), encoding='utf-8')
    print('[patch] OK: navigation_launch.py  docking_server cmd_vel -> cmd_vel_dock')
    return 0



def generate_dock_database() -> int:
    """按 SPAWN_*/DOCK_* 环境变量生成 odom 坐标系下的 dock_database.yaml。

    **dock pose 的语义**：opennav_docking 里 `dock.pose` 是「机器人停靠在桩上时
    应该在的位姿」，不是桩自己的位姿。搞错这一条的症状很隐蔽 —— docking_server
    会把 staging 点算到桩的背后（穿墙），然后返回 error_code=903 FAILED_TO_STAGE。
    所以这里要把 .env 里的「桩位姿」换算成「停靠位姿」：沿桩的 +x 前移
    DOCK_APPROACH 米，朝向转 180°（机器人是正对着桩开进去的）。
    """
    def env_float(name, default):
        try:
            return float(os.environ.get(name, default))
        except ValueError:
            return float(default)

    spawn_x = env_float('SPAWN_X', -1.8)
    spawn_y = env_float('SPAWN_Y', 0.5)
    spawn_yaw = env_float('SPAWN_YAW', 0.0)
    dock_x = env_float('DOCK_X', -2.3)
    dock_y = env_float('DOCK_Y', 0.5)
    dock_yaw = env_float('DOCK_YAW', 0.0)
    approach = env_float('DOCK_APPROACH', 0.30)

    # 世界 -> odom：odom 原点在出生点，且 odom 朝向 = 世界朝向（出生 yaw 旋转掉）
    dx = dock_x - spawn_x
    dy = dock_y - spawn_y
    c, s = math.cos(-spawn_yaw), math.sin(-spawn_yaw)
    ox = c * dx - s * dy
    oy = s * dx + c * dy
    oyaw = dock_yaw - spawn_yaw

    # 桩位姿 -> 机器人停靠位姿
    dock_in_odom_yaw = oyaw
    px = ox + approach * math.cos(dock_in_odom_yaw)
    py = oy + approach * math.sin(dock_in_odom_yaw)
    pyaw = math.atan2(math.sin(dock_in_odom_yaw + math.pi),
                      math.cos(dock_in_odom_yaw + math.pi))

    body = (
        '# 由 sim/patch_upstream.py 生成，每次容器启动按环境变量重算，不要手改。\n'
        '# frame 是 odom（docking_server 的 fixed_frame），odom 原点 = 机器人出生点。\n'
        f'# 桩的世界位姿   ({dock_x}, {dock_y}, yaw {dock_yaw})\n'
        f'# 出生点         ({spawn_x}, {spawn_y}, yaw {spawn_yaw})\n'
        f'# 机器人停靠位姿 = 桩 +{approach}m 沿桩 x 轴，朝向转 180°\n'
        'docks:\n'
        '  home_dock:\n'
        "    type: \"simple_charging_dock\"\n"
        '    frame: "odom"\n'
        f'    pose: [{px:.4f}, {py:.4f}, {pyaw:.4f}]\n'
    )
    DOCK_DB.write_text(body, encoding='utf-8')
    print(f'[patch] OK: {DOCK_DB}  home_dock 停靠位姿 @ odom '
          f'({px:.3f}, {py:.3f}, {math.degrees(pyaw):.1f}°)')
    return 0


def main() -> int:
    rc = patch_description_launch()
    rc |= patch_navigation_yaml()
    rc |= patch_navigation_launch()
    rc |= generate_dock_database()
    return rc


if __name__ == '__main__':
    sys.exit(main())
