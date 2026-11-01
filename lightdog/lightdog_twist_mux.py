#!/usr/bin/env python3
"""
LightDog 速度仲裁器（/cmd_vel 多源仲裁）

## 为什么需要这个节点

P2 跑跟随的时候发现一个硬问题：`/cmd_vel` 上有 4 个发布者在同时灌指令。

    collision_monitor   Nav2 的碰撞保护，把 cmd_vel_smoothed 中转成 cmd_vel
    docking_server      回充控制器，自己直接发 cmd_vel
    command_timeout     Nav2 的保险丝，/cmd_vel 一超时就发零速
    ros_gz_bridge       双向桥接，gz 侧的速度也会回灌到 ROS

导航空闲时 Nav2 那几个会持续发零速，把 follower 的 -0.15 m/s 淹掉 ——
实测机器人 6 秒只挪了 5cm。P2 当时的权宜之计是在 `p2.sh start` 里把
Nav2 的运动节点整个 kill 掉，代价是**丢了碰撞保护**。车装上真的电机之前
必须把这个补回来，否则一次近距离障碍就是一次撞车。

## 架构

                     ┌── /cmd_vel_smoothed  (Nav2,  prio 10) ──┐
                     │                                          │
   上游速度源 ────────┼── /cmd_vel_follow    (跟随,  prio 20) ──┼──> lightdog_twist_mux
                     │                                          │        │
                     └── /cmd_vel_dock      (回充,  prio 30) ──┘        │ /cmd_vel_muxed
                                                                         │
                                                          collision_monitor（碰撞保护）
                                                                         │ /cmd_vel
                                                               ros_gz_bridge → gz → 底盘

仲裁放在 `collision_monitor` **之前**，这样不管哪个源赢了，最终都要过一遍
碰撞保护 —— 这是与「直接抢 /cmd_vel」最大的区别。

## 仲裁规则

1. 每个源独立计时：最后一次收到该源消息的时间 + `*_timeout` 之内算「活跃」。
2. 活跃源里 `*_priority` 最大的那个独占输出；同优先级按 dock > follow > nav。
3. 一个活跃源都没有 → 发零速（默认），避免底层保持上一条指令。

超时用**墙钟**（`time.monotonic()`）而不是 ROS 时钟：这个窗口回答的是
「上游还在不在发包」，与仿真跑多快无关。本机软件渲染实时因子只有 0.22x，
用 sim time 会把 0.6s 的窗口拖成 2.7s 墙钟，上游已经死了还在转发旧指令。
反过来 Nav2 空闲时 20Hz 的发包在墙上也只有约 4Hz，0.6s 窗口照样容得下。

优先级高 ≠ 永久占用：跟随节点在人丢了以后仍在发零速，但 `*_timeout` 到点后
（节点被停掉的情况下）它会掉出活跃集，Nav2 自然接管。要在回充时让出场，
把跟随节点停掉（`enable_follow:=false`）或改优先级即可。

## 为什么不用 twist_mux

ROS 生态里有现成的 `twist_mux`，但这个镜像里没装，而容器内 `apt-get update`
实测跑不完（会被超时打断），重启容器后装的包也会丢。自己写一个 200 行的
节点反而更可控，而且能顺手把当前仲裁状态发布出来（`/cmd_vel_mux_state`），
排查「为什么车不动」时一眼就能看出是谁压住了谁。

## 为什么常驻而不是跟着 P2 起

`collision_monitor` 的输入现在指向本节点的输出。本节点不在，`/cmd_vel_muxed`
就断流，Nav2 的自主导航**连一条速度都发不出去**（会被 source_timeout 兜成零）。
所以它和 Nav2 一样是基础设施，由 `sim/entrypoint.sh` 常驻拉起，P2 只是往
里面多接一个跟随源。

## 话题

    订阅  /cmd_vel_smoothed  geometry_msgs/Twist   Nav2（velocity_smoother 的输出）
    订阅  /cmd_vel_follow    geometry_msgs/Twist   跟随控制器
    订阅  /cmd_vel_dock      geometry_msgs/Twist   回充控制器
    发布  /cmd_vel_muxed     geometry_msgs/Twist   仲裁结果（接 collision_monitor）
    发布  /cmd_vel_mux_state std_msgs/String       当前生效的源名，用于观测
"""

import sys
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist


class CmdVelSource:
    """一个速度输入源：话题、优先级、超时窗口。"""

    __slots__ = ('name', 'topic', 'prio', 'timeout', 'msg', 'stamp', 'count')

    def __init__(self, name, topic, prio, timeout):
        self.name = name
        self.topic = topic
        self.prio = prio
        self.timeout = timeout
        self.msg = None       # 最后一条消息
        self.stamp = None     # 收到时刻（time.monotonic）
        self.count = 0

    def is_active(self, now):
        """在超时窗口内收到过消息就算活跃。"""
        if self.stamp is None:
            return False
        return (now - self.stamp) <= self.timeout


class TwistMux(Node):

    def __init__(self):
        super().__init__('lightdog_twist_mux')

        self.declare_parameter('nav_topic', '/cmd_vel_smoothed')
        self.declare_parameter('nav_priority', 10)
        self.declare_parameter('nav_timeout', 0.6)

        self.declare_parameter('follow_topic', '/cmd_vel_follow')
        self.declare_parameter('follow_priority', 20)
        self.declare_parameter('follow_timeout', 0.6)

        self.declare_parameter('dock_topic', '/cmd_vel_dock')
        self.declare_parameter('dock_priority', 30)
        self.declare_parameter('dock_timeout', 1.0)

        self.declare_parameter('out_topic', '/cmd_vel_muxed')
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('zero_on_idle', True)

        p = self.get_parameter
        # 顺序即同优先级下的偏好：dock > follow > nav
        self.sources = [
            CmdVelSource('dock', str(p('dock_topic').value),
                         int(p('dock_priority').value), float(p('dock_timeout').value)),
            CmdVelSource('follow', str(p('follow_topic').value),
                         int(p('follow_priority').value), float(p('follow_timeout').value)),
            CmdVelSource('nav', str(p('nav_topic').value),
                         int(p('nav_priority').value), float(p('nav_timeout').value)),
        ]
        self.zero_on_idle = bool(p('zero_on_idle').value)
        rate = max(1.0, float(p('publish_rate').value))

        self.pub = self.create_publisher(Twist, str(p('out_topic').value), 10)
        self.pub_state = self.create_publisher(String, '/cmd_vel_mux_state', 10)

        self.active_name = None       # 上一个周期的赢家，用来只打切换日志
        self.subs = []
        for src in self.sources:
            # 用默认参数绑定的 lambda 捕获当前 src，避免闭包陷阱
            self.subs.append(self.create_subscription(
                Twist, src.topic, lambda m, s=src: self._on_twist(m, s), 10))

        self.create_timer(1.0 / rate, self._tick)

        desc = ', '.join(f'{s.name}={s.topic}(prio {s.prio}, {s.timeout:.1f}s)'
                         for s in sorted(self.sources, key=lambda x: -x.prio))
        self.get_logger().info(f'TwistMux ready: {desc} -> {p("out_topic").value}')

    # ---------- 回调 ----------

    def _on_twist(self, msg: Twist, src: CmdVelSource):
        src.msg = msg
        src.stamp = time.monotonic()
        src.count += 1

    # ---------- 仲裁 ----------

    def _pick(self):
        """选出当前该生效的源；没有活跃源返回 None。"""
        now = time.monotonic()
        best = None
        for src in self.sources:          # 已按优先级从高到低排列
            if src.is_active(now):
                if best is None or src.prio > best.prio:
                    best = src
        return best

    def _tick(self):
        best = self._pick()
        name = best.name if best is not None else 'idle'

        if name != self.active_name:
            if best is None:
                self.get_logger().info('仲裁: 无活跃源 -> 输出零速')
            else:
                self.get_logger().info(
                    f'仲裁: 切到 {best.name}（{best.topic}, prio {best.prio}）')
            self.active_name = name

        # 状态周期发布（不是只在切换时发）：否则 `ros2 topic echo --once`
        # 得等到下次切换才有消息，排查"为什么车不动"时全是超时。
        self.pub_state.publish(String(data=name))

        if best is None:
            if self.zero_on_idle:
                self.pub.publish(Twist())
            return

        self.pub.publish(best.msg)


def main(args=None):
    rclpy.init(args=args)
    node = TwistMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.pub.publish(Twist())    # 退出前确保下游收到停止指令
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
