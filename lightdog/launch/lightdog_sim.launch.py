"""LightDog 仿真入口。

设计原则：**不修改上游 linorobot2 的任何文件**。
做法是 include 上游的 gazebo.launch.py，只把它已经声明好的 ``urdf`` 参数换成我们自己的
覆盖层 xacro（lightdog_2wd/4wd.urdf.xacro = 上游机座 + LED + 电池），
再额外挂两条只属于 LightDog 的桥接，并把 LED 节点拉起来。

上游 gazebo.launch.py 自己已经桥接了相机/激光/里程计等 topic，所以这里不能再挂一份，
否则 parameter_bridge 会抢同一个 topic。
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 与上游保持一致：车型由环境变量决定（2wd / 4wd / mecanum）
    robot_base = os.getenv('LINOROBOT2_BASE', '2wd')

    pkg_lightdog = FindPackageShare('lightdog')

    urdf_path = PathJoinSubstitution(
        [pkg_lightdog, 'urdf', f'lightdog_{robot_base}.urdf.xacro']
    )
    led_params = PathJoinSubstitution([pkg_lightdog, 'config', 'led_visuals.yaml'])
    gazebo_launch = PathJoinSubstitution(
        [FindPackageShare('linorobot2_gazebo'), 'launch', 'gazebo.launch.py']
    )

    world_name = LaunchConfiguration('world_name')

    # 电池 gz topic 里的 model 名取自 URDF <robot name>，见 lightdog_*.urdf.xacro 注释
    battery_gz_topic = f'/model/linorobot2_{robot_base}/battery/lightdog_battery/state'

    return LaunchDescription([
        DeclareLaunchArgument(
            'gui', default_value='true', description='Enable Gazebo Client'),
        DeclareLaunchArgument(
            'world_name', default_value='turtlebot3_world',
            description='Gazebo world name'),
        DeclareLaunchArgument(
            'spawn_x', default_value='0.0', description='Robot spawn X'),
        DeclareLaunchArgument(
            'spawn_y', default_value='0.0', description='Robot spawn Y'),
        DeclareLaunchArgument(
            'spawn_yaw', default_value='0.0', description='Robot spawn heading'),

        # 上游仿真栈：用我们的 urdf 覆盖它的默认值，其余参数直传
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={
                'urdf': urdf_path,
                'gui': LaunchConfiguration('gui'),
                'world_name': world_name,
                'spawn_x': LaunchConfiguration('spawn_x'),
                'spawn_y': LaunchConfiguration('spawn_y'),
                'spawn_yaw': LaunchConfiguration('spawn_yaw'),
            }.items(),
        ),

        # LightDog 专用桥接（注意：每个桥接规格必须是一个 list 元素，
        # 否则三段字符串会被拆成三个独立 argv，parameter_bridge 直接退出 255）
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='lightdog_bridge',
            output='screen',
            arguments=[
                # LED 亮度：ROS 侧发 MaterialColor，改 visual 的 emissive
                ['/world/', world_name,
                 '/material_color@ros_gz_interfaces/msg/MaterialColor]gz.msgs.MaterialColor'],
                # 电池状态：gz 电池插件 -> ROS
                [battery_gz_topic, '@sensor_msgs/msg/BatteryState[gz.msgs.BatteryState'],
            ],
            remappings=[(battery_gz_topic, '/battery_state')],
        ),

        Node(
            package='lightdog',
            executable='lightdog_led.py',
            name='lightdog_led',
            output='screen',
            parameters=[
                led_params,
                {
                    'world_name': world_name,
                    'use_sim_time': True,
                },
            ],
        ),
    ])
