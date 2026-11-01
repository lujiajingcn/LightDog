"""LightDog P5：跌倒检测 + 告警链路。

## 分层

- 感知层  lightdog_fall_detector   RGB-D -> /fall/state（standing|lying|absent|unknown）
- 决策层  lightdog_alert_manager   两级确认状态机 -> /alert/state + /recorder/event
- 呈现层  lightdog_server          订阅 /alert/state，手机端弹窗（本 launch 不起）

## 为什么告警视频不在这里合成

`alert_manager` 只往 `/recorder/event` 发一条消息，视频切片完全由 P4 的
`lightdog_recorder` 负责。这样 P5 不需要知道任何视频细节，P4 也不依赖 P5 存在
—— 换掉任何一个都不会连带崩。

## 依赖

需要 P4 的 recorder 在跑（否则告警时没有视频，但不影响告警本身），
需要 P2 的「人」在场景里（否则检测器一直报 absent）。

## 用法

    ros2 launch lightdog lightdog_p5.launch.py
    # 或者（宿主机）bash sim/p5.sh start
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
    params = PathJoinSubstitution([pkg, 'config', 'p5_params.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('enable_detector', default_value='true'),
        DeclareLaunchArgument('enable_alert', default_value='true'),
        DeclareLaunchArgument('publish_debug', default_value='true'),
        DeclareLaunchArgument('auto_reset', default_value='true',
                              description='人起身后是否自动解除告警'),
        DeclareLaunchArgument('contacts_path', default_value='/data/contacts.json'),
        DeclareLaunchArgument('call_command', default_value='',
                              description='空=只记录不真拨；真机填外呼命令模板'),

        Node(
            package='lightdog',
            executable='lightdog_fall_detector.py',
            name='lightdog_fall_detector',
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_detector')),
            parameters=[
                params,
                # 必须显式 value_type=bool：LaunchConfiguration 求值出来是字符串
                # 'true'，直接喂给声明成 bool 的参数会被 rclpy 判类型不符而抛异常
                # （不是静默转 false，是节点起不来）。
                {'publish_debug': ParameterValue(
                    LaunchConfiguration('publish_debug'), value_type=bool)},
            ],
        ),

        Node(
            package='lightdog',
            executable='lightdog_alert_manager.py',
            name='lightdog_alert_manager',
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_alert')),
            parameters=[
                params,
                {'auto_reset': ParameterValue(
                    LaunchConfiguration('auto_reset'), value_type=bool),
                 'contacts_path': LaunchConfiguration('contacts_path'),
                 # 注意：这里传的是**参数值**（空串合法），不是 ros2 launch 的
                 # `name:=value` 形式，所以空值不会触发 malformed argument。
                 'call_command': LaunchConfiguration('call_command')},
            ],
        ),
    ])
