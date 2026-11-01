#!/usr/bin/env python3
"""
LightDog 光照联动（P2 / 场景 1）

场景：夜里人下床 -> 机器人看到人 -> 主灯亮起照亮去卫生间的路 -> 人离开后自动熄灭。

## 为什么不能直接「有人亮、无人灭」

人走两步、一转身，检测会短暂丢帧。如果立刻灭灯，房间就是一片闪烁，比不开灯更糟。
所以这里有三个时间参数：

    fade_delay   确认人离开多久后才开始变暗（默认 5s，容忍短暂丢检）
    fade_time    变暗/变亮各用多久（默认 2s，线性过渡）
    hold         无人时是否保留一点余光（默认全灭，符合「不影响睡眠」）

亮度直接下发到 /led/brightness（P1 已实现的接口），本节点不碰材质层。

## 接口

    订阅 /person/present   std_msgs/Bool
    发布 /led/brightness   std_msgs/Float32  0.0 ~ 1.0

`enabled=false` 时本节点完全不发布，把 /led/brightness 让给手动控制（p1.sh）。
"""

import math
import sys

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, Float32


class LightAuto(Node):

    def __init__(self):
        super().__init__('lightdog_light_auto')

        self.declare_parameter('on_brightness', 0.85)   # 有人时的亮度
        self.declare_parameter('idle_brightness', 0.0)  # 无人时的余光
        self.declare_parameter('fade_delay', 5.0)       # 人走后多久开始变暗
        self.declare_parameter('fade_time', 2.0)        # 过渡时长
        self.declare_parameter('step_hz', 20.0)         # 渐变刷新率
        self.declare_parameter('enabled', True)

        p = self.get_parameter
        self.on_level = float(p('on_brightness').value)
        self.idle_level = float(p('idle_brightness').value)
        self.fade_delay = float(p('fade_delay').value)
        self.fade_time = max(0.1, float(p('fade_time').value))
        self.enabled = bool(p('enabled').value)
        hz = max(1.0, float(p('step_hz').value))

        self.present = False
        self.last_seen = None
        self.last_t = None
        self.level = self.idle_level          # 当前实际下发值
        self.target = self.idle_level

        self.create_subscription(Bool, '/person/present', self._on_present, 10)
        self.pub = self.create_publisher(Float32, '/led/brightness', 10)

        self.create_timer(1.0 / hz, self._tick)

        self.get_logger().info(
            f'LightDog light-auto ready: on={self.on_level:.2f} '
            f'idle={self.idle_level:.2f} delay={self.fade_delay:.1f}s '
            f'fade={self.fade_time:.1f}s enabled={self.enabled}')

    def _on_present(self, msg: Bool):
        self.present = bool(msg.data)
        if self.present:
            self.last_seen = self.get_clock().now()

    def _tick(self):
        if not self.enabled:
            return

        now = self.get_clock().now()
        dt = 0.0 if self.last_t is None else (now - self.last_t).nanoseconds / 1e9
        self.last_t = now
        # 时间跳变保护（仿真暂停/时钟重置时别让亮度瞬间跳到底）
        dt = min(max(dt, 0.0), 0.5)

        # --- 目标亮度决策 ---
        if self.present:
            self.target = self.on_level
        elif self.last_seen is None:
            self.target = self.idle_level
        else:
            age = (now - self.last_seen).nanoseconds / 1e9
            self.target = self.idle_level if age >= self.fade_delay else self.on_level

        # --- 线性过渡 ---
        # 按时间积分，而不是「每 tick 固定一步」。原来那种写法假定 timer 真能跑满
        # step_hz，但实测软件渲染下 20Hz 的 timer 只跑到 ~4.5Hz，于是 fade_time=2s
        # 的渐变实际要 8~9s。改成按 dt 积分后，渐变时长与 tick 频率解耦。
        diff = self.target - self.level
        max_delta = dt / self.fade_time
        if abs(diff) <= max_delta:
            self.level = self.target
        else:
            self.level += math.copysign(max_delta, diff)

        self.pub.publish(Float32(data=float(self.level)))


def main(args=None):
    rclpy.init(args=args)
    node = LightAuto()
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
