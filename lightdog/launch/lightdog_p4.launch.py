"""LightDog P4：视频存储 + 手机互联。

## 分层

- 存储层  lightdog_recorder  相机 -> 环形分段 mp4 + 事件切片（含事发前 N 秒）
- 接入层  lightdog_server    HTTP：MJPEG 实时流 / 事件回放 / 联系人 / 状态

两者是解耦的：`lightdog_server` 只读 `/data` 下的文件，`lightdog_recorder` 只写。
关掉后端录像照跑，关掉录像后端仍然能出实时流 —— 它们之间唯一的耦合是
「同一份 data_dir」和「后端把 POST /events 转成 /recorder/event 消息」。

## 为什么相机只有一个订阅者不够

P2 的人体检测、P3 的 ArUco 已经各订阅了一份 `/camera/color/image_raw`。
再挂两个（录像 + 推流）在仿真里没问题（1.5 Hz × 0.9MB 完全够），但真机上
更好的做法是让 recorder 缓存最近 N 帧再给 server —— 这里没那么做，是因为
server 需要「最新一帧」的独立性：录像卡住时实时画面不能跟着卡。

## 用法

    ros2 launch lightdog lightdog_p4.launch.py
    # 或者（宿主机）bash sim/p4.sh start
    # 手机打开 http://<机器人IP>:8000/
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('lightdog')
    params = PathJoinSubstitution([pkg, 'config', 'p4_params.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('data_dir', default_value='/data',
                              description='录像根目录（clips/ events/ snapshots/）'),
        DeclareLaunchArgument('enable_recorder', default_value='true'),
        DeclareLaunchArgument('enable_server', default_value='true'),
        DeclareLaunchArgument('recording', default_value='true',
                              description='只推流不落盘时设 false（省卡）'),
        DeclareLaunchArgument('http_port', default_value='8000'),
        DeclareLaunchArgument('auth_token', default_value='',
                              description='空=不鉴权；真机务必设置'),
        DeclareLaunchArgument('segment_sec', default_value='10.0'),

        Node(
            package='lightdog',
            executable='lightdog_recorder.py',
            name='lightdog_recorder',
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_recorder')),
            parameters=[
                params,
                {'data_dir': LaunchConfiguration('data_dir'),
                 'recording': LaunchConfiguration('recording'),
                 'segment_sec': LaunchConfiguration('segment_sec')},
            ],
        ),

        Node(
            package='lightdog',
            executable='lightdog_server.py',
            name='lightdog_server',
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_server')),
            parameters=[
                params,
                {'data_dir': LaunchConfiguration('data_dir'),
                 'http_port': LaunchConfiguration('http_port'),
                 'auth_token': LaunchConfiguration('auth_token')},
            ],
        ),
    ])
