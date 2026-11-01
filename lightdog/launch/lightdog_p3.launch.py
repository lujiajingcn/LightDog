"""LightDog P3：自动回充（ArUco 视觉对准 + 低电状态机）。

## 分层

- 感知层  lightdog_aruco_dock    相机 -> /detected_dock_pose（dock frame 位姿）
- 决策层  lightdog_dock_manager  电量 -> /dock_robot action / /dock/state
- 执行层  Nav2 docking_server    直接在 navigation_launch.py 里跑（上游自带），
                                 速度出口被 patch 到 /cmd_vel_dock

## 为什么执行层不在这里启动

`docking_server` 由上游 `nav2_bringup/navigation_launch.py` 拉起，并且挂在
`lifecycle_manager_navigation` 的管理列表里。在 launch 里再起一份会出现两个
同名节点抢 `/dock_robot` action。我们只做两件事：
  1. `sim/patch_upstream.py` 把它 Node 定义的 remappings 补上
     `('cmd_vel', 'cmd_vel_dock')`，让它进仲裁而不是直发 `/cmd_vel`
  2. 补丁把 `dock_database` 指向启动时生成的 `/tmp/dock_database.yaml`

## 桩体的生成不放在这里

和 P2 的「人」一样：`ros_gz_sim create` 必须等 gz server 真的就绪，
而就绪时间在软件渲染下是 15~25s 且不稳定。生成逻辑放 `sim/p3.sh`（带重试）。

## 用法

    ros2 launch lightdog lightdog_p3.launch.py
    # 或者（宿主机）bash sim/p3.sh start && bash sim/p3.sh spawn
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('lightdog')
    params = PathJoinSubstitution([pkg, 'config', 'p3_params.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('auto_dock', default_value='true',
                              description='低电是否自动发起回充'),
        DeclareLaunchArgument('enable_aruco', default_value='true',
                              description='是否启用 ArUco 对准（关掉只测状态机）'),
        DeclareLaunchArgument('sim_charging', default_value='true',
                              description='仿真补丁：停靠后模拟充电。真机必须关'),
        DeclareLaunchArgument('publish_debug', default_value='true'),

        Node(
            package='lightdog',
            executable='lightdog_aruco_dock.py',
            name='lightdog_aruco_dock',
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_aruco')),
            parameters=[
                params,
                {'publish_debug': LaunchConfiguration('publish_debug')},
            ],
        ),

        Node(
            package='lightdog',
            executable='lightdog_dock_manager.py',
            name='lightdog_dock_manager',
            output='screen',
            parameters=[
                params,
                {'auto_dock': LaunchConfiguration('auto_dock'),
                 'sim_charging': LaunchConfiguration('sim_charging')},
            ],
        ),
    ])
