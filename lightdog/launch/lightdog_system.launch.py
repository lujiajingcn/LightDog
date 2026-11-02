"""LightDog 系统层：任务调度 + 语音链路。

## 分层

- 调度层  lightdog_mission   模式仲裁 -> /follow/enable、/mission/hold_motion、
                             /dock/command、Nav2 NavigateToPose
- 语音层  lightdog_voice     ASR 文本 -> 语义指令；/alert/voice -> TTS

这两个节点都不产生速度。真正踩油门的是 follower / docking_server / Nav2，
它们的公共收口是 `lightdog_twist_mux`（安全闸也放在那里）。

## 启动顺序

`lightdog_mission` 起来时会立刻发一轮 `/follow/enable` 与
`/mission/hold_motion`。跟随器在**没收到过**这两个值时按自己的参数工作，
所以本 launch 晚于 P2 起来也不会让车停住 —— 但**先起它更好**：否则前几秒
跟随与 Nav2 之间的优先级没人协调。

## 用法

    ros2 launch lightdog lightdog_system.launch.py
    ros2 launch lightdog lightdog_system.launch.py patrol:=true
    # 只看调度不上语音：enable_voice:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('lightdog')
    params = PathJoinSubstitution([pkg, 'config', 'system_params.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('enable_mission', default_value='true'),
        DeclareLaunchArgument('enable_voice', default_value='true'),
        DeclareLaunchArgument('auto_mode', default_value='true',
                              description='false = 起来就是人工接管'),
        DeclareLaunchArgument('patrol', default_value='false',
                              description='开启巡游（需要 system_params.yaml 里有路点）'),
        DeclareLaunchArgument('dock_approach', default_value='true',
                              description='低电时是否先导航到桩前'),
        DeclareLaunchArgument('tts_command', default_value='',
                              description='空=只发 /voice/say；真机填 TTS 命令模板'),

        Node(
            package='lightdog',
            executable='lightdog_mission.py',
            name='lightdog_mission',
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_mission')),
            parameters=[
                params,
                # LaunchConfiguration 求值是字符串，bool 参数必须显式转换，
                # 否则 rclpy 判类型不符直接抛异常（节点起不来）。
                {'auto_mode': ParameterValue(
                    LaunchConfiguration('auto_mode'), value_type=bool),
                 'patrol_enabled': ParameterValue(
                    LaunchConfiguration('patrol'), value_type=bool),
                 'dock_approach': ParameterValue(
                    LaunchConfiguration('dock_approach'), value_type=bool)},
            ],
        ),

        Node(
            package='lightdog',
            executable='lightdog_voice.py',
            name='lightdog_voice',
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_voice')),
            parameters=[
                params,
                {'tts_command': LaunchConfiguration('tts_command')},
            ],
        ),
    ])
