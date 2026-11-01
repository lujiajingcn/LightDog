"""LightDog P2：人体检测 + 跟随 + 光照联动。

## 分层

- 感知层  lightdog_person_detector  相机 -> /person/present + /person/position
- 决策层  lightdog_light_auto       有没有人 -> /led/brightness（渐变）
- 执行层  lightdog_follower         人去哪 -> /cmd_vel_follow
- 仿真专用 lightdog_person_sim      让仿真里的「人」自己走起来

四个节点互相只通过上面这几个话题耦合，换检测器（YOLO）或换控制律不会牵连别人。

## 仲裁层不在这里

`lightdog_twist_mux` 由 `sim/entrypoint.sh` 常驻拉起，不跟 P2 一起起 ——
`collision_monitor` 的输入已经指向它的输出，它一停 Nav2 也发不出速度。
P2 只是往它里面多接一个 `/cmd_vel_follow` 源（优先级高于 Nav2）。

## 为什么执行层不直接发 /cmd_vel

`/cmd_vel` 上本来就挤着 Nav2 的 collision_monitor / docking_server /
command_timeout 和 ros_gz_bridge。跟随指令直发进去会被它们的空闲零速淹没，
而如果把它们 kill 掉又会丢掉碰撞保护。所以中间插一层仲裁，输出交给
collision_monitor，最后由它独占 `/cmd_vel`。

## 人的生成不放在这里

`ros_gz_sim create` 必须等 gz server 真的起来，否则静默失败；而 gz 就绪时间在
不同机器上差别很大（本机软件渲染要 15~25s）。所以人的生成交给 `sim/p2.sh`，
它带重试和结果检查。person_sim 节点即使先启动也只是 set_pose 失败告警，
等人一生成就会接上。

## 用法

    ros2 launch lightdog lightdog_p2.launch.py
    然后 sim/p2.sh 会自动 spawn 人（或手动 bash sim/p2.sh spawn）
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('lightdog')
    params = PathJoinSubstitution([pkg, 'config', 'p2_params.yaml'])
    world_name = LaunchConfiguration('world_name')

    return LaunchDescription([
        DeclareLaunchArgument('world_name', default_value='turtlebot3_world'),
        DeclareLaunchArgument('enable_follow', default_value='true',
                              description='是否启用跟随（关掉只做检测+照明）'),
        DeclareLaunchArgument('enable_auto_light', default_value='true',
                              description='是否让 P2 接管 /led/brightness'),
        DeclareLaunchArgument('enable_person_sim', default_value='true',
                              description='是否驱动仿真里的人移动'),
        DeclareLaunchArgument('publish_debug', default_value='true'),

        Node(
            package='lightdog',
            executable='lightdog_person_detector.py',
            name='lightdog_person_detector',
            output='screen',
            parameters=[
                params,
                {'publish_debug': LaunchConfiguration('publish_debug')},
            ],
        ),

        Node(
            package='lightdog',
            executable='lightdog_light_auto.py',
            name='lightdog_light_auto',
            output='screen',
            parameters=[
                params,
                {'enabled': LaunchConfiguration('enable_auto_light')},
            ],
        ),

        Node(
            package='lightdog',
            executable='lightdog_follower.py',
            name='lightdog_follower',
            output='screen',
            parameters=[
                params,
                {'enabled': LaunchConfiguration('enable_follow')},
            ],
        ),

        Node(
            package='lightdog',
            executable='lightdog_person_sim.py',
            name='lightdog_person_sim',
            output='screen',
            parameters=[
                params,
                {'world_name': world_name,
                 'enabled': LaunchConfiguration('enable_person_sim')},
            ],
        ),
    ])
