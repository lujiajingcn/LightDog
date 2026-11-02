#!/usr/bin/env python3
"""
LightDog LED 亮度控制器

逻辑接口（产品层，真机固件照这个实现）:
    /led/brightness   std_msgs/Float32   0.0 ~ 1.0   主 LED 四灯亮度
    /led/indicator    std_msgs/Float32   0.0 ~ 1.0   底部定位指示灯亮度

渲染层（仅仿真）:
    /world/<world_name>/material_color   ros_gz_interfaces/MaterialColor
    经 ros_gz_bridge 转入 gz-sim，修改对应 visual 的 emissive。

为什么不用 Gazebo 光源: 已实测 gz-sim 8.15 丢弃 URDF 的 <light>（link 级与
model 级都不生成可渲染光源），因此亮度只体现为材质自发光。
"""

import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import Float32
from ros_gz_interfaces.msg import MaterialColor, Entity


class LedController(Node):

    def __init__(self):
        super().__init__('lightdog_led')

        self.declare_parameter('world_name', 'turtlebot3_world')
        # 实测值 (gz sdf -p 输出)。fixed link 全部 lump 到 base_footprint，
        # 且链接名自带 _visual 后缀导致双写；序号按 link 名字母序固定。
        # 下列为 ROBOT_BASE=4wd（本产品车型）的实测名；
        # 2wd 对应 ..._visual_visual_4/6/7/8，指示为 _5，可用
        # config/led_visuals.yaml 覆盖。改动 LED link 数量或命名后需重新实测。
        self.declare_parameter('led_visuals', [
            'base_footprint_fixed_joint_lump__led_front_visual_visual_3',
            'base_footprint_fixed_joint_lump__led_left_visual_visual_5',
            'base_footprint_fixed_joint_lump__led_rear_visual_visual_6',
            'base_footprint_fixed_joint_lump__led_right_visual_visual_7',
        ])
        self.declare_parameter('indicator_visual',
            'base_footprint_fixed_joint_lump__led_indicator_visual_visual_4')

        # 暖白主光 / 琥珀红指示光（低亮度红光对褪黑素影响小）
        self.declare_parameter('main_color', [1.00, 0.92, 0.80])
        self.declare_parameter('indicator_color', [1.00, 0.25, 0.10])
        # 待机时主灯全灭，指示灯维持常亮微光
        self.declare_parameter('default_brightness', 0.0)
        self.declare_parameter('default_indicator', 0.06)

        world = self.get_parameter('world_name').value
        self.led_visuals = list(self.get_parameter('led_visuals').value)
        self.indicator_visual = self.get_parameter('indicator_visual').value
        self.main_color = list(self.get_parameter('main_color').value)
        self.ind_color = list(self.get_parameter('indicator_color').value)

        self.brightness = float(self.get_parameter('default_brightness').value)
        self.indicator = float(self.get_parameter('default_indicator').value)

        # 本节点的作用是**仿真渲染层**：把亮度翻译成 gz 的材质自发光。
        # 真机上 LED 由固件直接订阅 /led/brightness 驱动，这个节点不该启动。
        # 真机上若误拉起，它只会往一个没人转发的 ros_gz_interfaces 话题发消息，
        # 表现为「灯不亮、也没报错」，排查时极易误判成固件问题。
        try:
            sim_time = bool(self.get_parameter('use_sim_time').value)
        except Exception:
            sim_time = True
        if not sim_time:
            self.get_logger().error(
                'use_sim_time=false：本节点只服务 Gazebo 材质渲染，'
                '真机上请让固件消费 /led/brightness，并在 launch 里去掉本节点')

        latched = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.pub_mat = self.create_publisher(
            MaterialColor, f'/world/{world}/material_color', latched)

        self.create_subscription(
            Float32, '/led/brightness', self._on_brightness, 10)
        self.create_subscription(
            Float32, '/led/indicator', self._on_indicator, 10)

        # 低频重发，覆盖 gz 侧可能在实体重载后丢失的材质状态
        self.create_timer(2.0, self._republish)

        self.get_logger().info(
            f'LightDog LED ready: world={world}, '
            f'main_leds={len(self.led_visuals)}, '
            f'brightness={self.brightness:.2f}, indicator={self.indicator:.2f}')

        self._apply_all()

    # ---------- ROS 回调 ----------

    def _on_brightness(self, msg: Float32):
        value = max(0.0, min(1.0, float(msg.data)))
        if abs(value - self.brightness) < 1e-3:
            return
        self.brightness = value
        self.get_logger().info(f'main brightness -> {value:.2f}')
        self._apply_main()

    def _on_indicator(self, msg: Float32):
        value = max(0.0, min(1.0, float(msg.data)))
        if abs(value - self.indicator) < 1e-3:
            return
        self.indicator = value
        self.get_logger().info(f'indicator brightness -> {value:.2f}')
        self._apply_indicator()

    # ---------- 材质下发 ----------

    def _color(self, rgb, level):
        """亮度不仅加在 emissive 上：diffuse 也随亮度抬升，暗环境下更接近真实灯泡。"""
        emissive = level
        ambient = 0.01 + 0.09 * level
        diffuse = 0.03 + 0.35 * level
        return (tuple(c * emissive for c in rgb),
                tuple(c * ambient for c in rgb),
                tuple(c * diffuse for c in rgb))

    def _make_msg(self, visual_name, rgb, level):
        msg = MaterialColor()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.entity.name = visual_name
        msg.entity.type = Entity.VISUAL

        em, am, di = self._color(rgb, level)
        msg.emissive.r, msg.emissive.g, msg.emissive.b, msg.emissive.a = (*em, 1.0)
        msg.ambient.r, msg.ambient.g, msg.ambient.b, msg.ambient.a = (*am, 1.0)
        msg.diffuse.r, msg.diffuse.g, msg.diffuse.b, msg.diffuse.a = (*di, 1.0)
        msg.specular.r = msg.specular.g = msg.specular.b = 0.1 * level
        msg.specular.a = 1.0
        return msg

    def _apply_main(self):
        for name in self.led_visuals:
            self.pub_mat.publish(self._make_msg(name, self.main_color, self.brightness))

    def _apply_indicator(self):
        self.pub_mat.publish(
            self._make_msg(self.indicator_visual, self.ind_color, self.indicator))

    def _apply_all(self):
        self._apply_main()
        self._apply_indicator()

    def _republish(self):
        self._apply_all()


def main(args=None):
    rclpy.init(args=args)
    node = LedController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
