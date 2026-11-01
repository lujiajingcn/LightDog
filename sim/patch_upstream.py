#!/usr/bin/env python3
"""给上游 linorobot2 打「唯一必需的」补丁：修 description.launch.py 的 robot_description 类型错误。

现象（ROS 2 Jazzy + launch_ros 新版）：

    [ERROR] [launch]: Caught exception in launch: Unable to parse the value of
    parameter robot_description as yaml. If the parameter is meant to be a string,
    try wrapping it in launch_ros.parameter_descriptions.ParameterValue(value, value_type=str)

原因：`Command(['xacro ', ...])` 的替换结果被 launch_ros 按 YAML 解析，而 xacro 输出以
`<?xml` 开头，必然解析失败。上游自身的写法在 Jazzy 上就是坏的（已实测）。

修法即报错信息里给的标准写法：套一层 ParameterValue(..., value_type=str)。

该脚本幂等：
  - 已打过补丁 → 打印 SKIP，退出 0
  - 找不到预期字符串（说明上游升级改了这里的代码）→ 打印错误，退出 1
    （故意让容器起不来，而不是静默跑一个坏配置）
"""

import pathlib
import sys

TARGET = pathlib.Path(
    '/home/ros/linorobot2_ws/install/linorobot2_description'
    '/share/linorobot2_description/launch/description.launch.py'
)

IMPORT_ANCHOR = 'from launch_ros.substitutions import FindPackageShare'
IMPORT_LINE = 'from launch_ros.parameter_descriptions import ParameterValue'

OLD = "                    'robot_description': Command(['xacro ', LaunchConfiguration('urdf')])"
NEW = (
    "                    'robot_description': ParameterValue(\n"
    "                        Command(['xacro ', LaunchConfiguration('urdf')]),\n"
    "                        value_type=str)"
)


def main() -> int:
    if not TARGET.is_file():
        print(f'[patch] FAIL: 目标文件不存在: {TARGET}')
        return 1

    text = TARGET.read_text(encoding='utf-8')

    if 'ParameterValue(' in text and OLD not in text:
        print('[patch] SKIP: 已打过补丁')
        return 0

    if OLD not in text:
        print('[patch] FAIL: 没找到预期的那一行，上游可能改了实现。')
        print('[patch]       请重新评估这个补丁是否还需要，位置:')
        print(f'[patch]       {TARGET}')
        return 1

    if IMPORT_LINE not in text:
        if IMPORT_ANCHOR not in text:
            print('[patch] FAIL: 找不到 import 锚点，无法安全插入 import。')
            return 1
        text = text.replace(IMPORT_ANCHOR, f'{IMPORT_ANCHOR}\n{IMPORT_LINE}', 1)

    text = text.replace(OLD, NEW, 1)
    TARGET.write_text(text, encoding='utf-8')
    print(f'[patch] OK: 已修补 {TARGET.name}（robot_description -> ParameterValue(str)）')
    return 0


if __name__ == '__main__':
    sys.exit(main())
