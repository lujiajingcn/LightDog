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

该脚本幂等：
  - 已打过补丁 → 打印 SKIP，退出 0
  - 找不到预期字符串（说明上游升级改了这里的代码）→ 打印错误，退出 1
    （故意让容器起不来，而不是静默跑一个坏配置）
"""

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


def patch_navigation_yaml() -> int:
    """collision_monitor 的输入改到仲裁器的输出上。"""
    target = NAVIGATION_YAML
    if not target.is_file():
        print(f'[patch] FAIL: 目标文件不存在: {target}')
        return 1

    text = target.read_text(encoding='utf-8')

    old_line = 'cmd_vel_in_topic: "cmd_vel_smoothed"'
    new_line = 'cmd_vel_in_topic: "cmd_vel_muxed"'

    if 'cmd_vel_in_topic: "cmd_vel_muxed"' in text:
        print('[patch] SKIP: navigation.yaml 的 collision_monitor 已打过补丁')
        return 0

    if text.count(old_line) != 1:
        print(f'[patch] FAIL: navigation.yaml 里 {old_line!r} 出现 '
              f'{text.count(old_line)} 次（预期 1 次），上游可能改了实现。')
        print(f'[patch]       位置: {target}')
        return 1

    # 顺带断言出口话题没被一起改掉：collision_monitor 仍要独占 /cmd_vel
    if 'cmd_vel_out_topic: "cmd_vel"' not in text:
        print('[patch] FAIL: navigation.yaml 的 cmd_vel_out_topic 不是 "cmd_vel"，'
              '需要重新评估整条命令链路。')
        return 1

    target.write_text(text.replace(old_line, new_line, 1), encoding='utf-8')
    print('[patch] OK: navigation.yaml  collision_monitor 输入 -> cmd_vel_muxed')
    return 0


def main() -> int:
    rc = patch_description_launch()
    rc |= patch_navigation_yaml()
    return rc


if __name__ == '__main__':
    sys.exit(main())
