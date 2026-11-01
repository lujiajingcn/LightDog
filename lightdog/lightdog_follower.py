#!/usr/bin/env python3
"""
LightDog 人体跟随控制器（P2 / 场景 1）

## 为什么不用 Nav2

路线里专门记过一笔：**跟随 ≠ 导航**。Nav2 要的是一张地图 + 一个 map 坐标系里的
目标点，人一进屋、一关门、一消失在视野里，代价地图就不知道该往哪走；而且人来
的地方机器人未必过得去，把人的位置当导航目标会让机器人硬往墙上撞。

P2 的需求是「人往哪走，灯就照到哪」，本质是**视觉伺服**：保持固定距离跟着轮廓走，
人停下机器人就停下。所以这里用最直接的 P 控制，输出 /cmd_vel。
Nav2 留给 P3 的自动回充（那里有确定的静态目标点）。

## 输入 / 输出

    订阅 /person/position  geometry_msgs/PointStamped  frame=base_link
    订阅 /person/present   std_msgs/Bool
    发布 /cmd_vel_follow   geometry_msgs/Twist

输出不直接发 `/cmd_vel`：那个话题上有 Nav2 的 collision_monitor / docking_server /
command_timeout 一群发布者，跟随指令会被它们的空闲零速淹没（P2 实测：follower
一直发 -0.15，机器人 6 秒只挪 5cm）。现在统一交给 `lightdog_twist_mux` 仲裁，
再由 collision_monitor 兜碰撞保护。话题名可用 `cmd_vel_topic` 参数改。

## 控制律

    err_dist  = x - target_distance          正=太远
    err_theta = atan2(y, x)                  正=人在左

    v = clamp(k_linear * err_dist, -0.6*max_v, max_v)，|err_dist| <= tol 时归零
    w = clamp(k_angular * err_theta, -max_w, max_w)，   |err_theta| <= ang_tol 时归零

三条安全约束（真机上这几条比控制精度重要）：
1. 距离小于 stop_distance 直接后退，不因为「还差一点没到目标距离」而继续顶上去。
2. 人离开视野超过 person_timeout 秒立刻停，宁可停在原地也不凭记忆往前冲。
3. 人在侧后方（|theta| > side_turn_only）时只转不走，避免绕着人画大圈。
"""

import math
import sys

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped, Twist


class PersonFollower(Node):

    def __init__(self):
        super().__init__('lightdog_follower')

        self.declare_parameter('target_distance', 1.20)   # 保持距离
        self.declare_parameter('distance_tol', 0.15)      # 距离死区
        self.declare_parameter('stop_distance', 0.55)     # 低于此值后退
        self.declare_parameter('k_linear', 0.9)
        self.declare_parameter('max_linear', 0.25)
        self.declare_parameter('k_angular', 1.4)
        self.declare_parameter('max_angular', 0.70)
        self.declare_parameter('angle_tol', 0.06)         # 角度死区
        self.declare_parameter('side_turn_only', 0.9)     # 超过此角度只转不走
        self.declare_parameter('person_timeout', 1.5)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_follow')
        self.declare_parameter('enabled', True)

        p = self.get_parameter
        self.target = float(p('target_distance').value)
        self.tol = float(p('distance_tol').value)
        self.stop_d = float(p('stop_distance').value)
        self.k_v = float(p('k_linear').value)
        self.max_v = float(p('max_linear').value)
        self.k_w = float(p('k_angular').value)
        self.max_w = float(p('max_angular').value)
        self.ang_tol = float(p('angle_tol').value)
        self.side_only = float(p('side_turn_only').value)
        self.timeout = float(p('person_timeout').value)
        self.enabled = bool(p('enabled').value)

        self.person = None            # 最新一次人位置
        self.present = False
        self.last_present = None      # rclpy Time

        self.create_subscription(PointStamped, '/person/position', self._on_pos, 10)
        self.create_subscription(Bool, '/person/present', self._on_present, 10)
        self.cmd_topic = str(p('cmd_vel_topic').value)
        self.pub = self.create_publisher(Twist, self.cmd_topic, 10)

        self.create_timer(0.1, self._tick)   # 10 Hz

        self.get_logger().info(
            f'LightDog follower ready: target={self.target:.2f}m '
            f'tol=±{self.tol:.2f}m stop<{self.stop_d:.2f}m '
            f'vmax={self.max_v:.2f}m/s wmax={self.max_w:.2f}rad/s '
            f'enabled={self.enabled}')

    # ---------- 回调 ----------

    def _on_pos(self, msg: PointStamped):
        self.person = msg

    def _on_present(self, msg: Bool):
        self.present = bool(msg.data)
        if self.present:
            self.last_present = self.get_clock().now()

    # ---------- 控制 ----------

    def _person_lost(self) -> bool:
        if self.last_present is None:
            return True
        age = (self.get_clock().now() - self.last_present).nanoseconds / 1e9
        return age > self.timeout

    def _tick(self):
        cmd = Twist()

        if not self.enabled or self._person_lost() or self.person is None:
            self.pub.publish(cmd)     # 持续发零速，防止底层保持上一条指令
            return

        x = float(self.person.point.x)      # 前向距离
        y = float(self.person.point.y)      # 左正
        if x <= 0.05:
            self.pub.publish(cmd)
            return

        err_theta = math.atan2(y, x)

        # --- 线速度 ---
        if x < self.stop_d:
            # 太近：按差距后退，限幅 0.6*max_v
            cmd.linear.x = -min(0.6 * self.max_v, self.k_v * (self.stop_d - x))
        elif abs(x - self.target) <= self.tol:
            cmd.linear.x = 0.0
        else:
            v = self.k_v * (x - self.target)
            cmd.linear.x = max(-0.6 * self.max_v, min(self.max_v, v))

        # --- 角速度 ---
        if abs(err_theta) <= self.ang_tol:
            cmd.angular.z = 0.0
        else:
            cmd.angular.z = max(-self.max_w, min(self.max_w, self.k_w * err_theta))

        # 人在侧后方：只转不走
        if abs(err_theta) > self.side_only:
            cmd.linear.x = 0.0

        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PersonFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.pub.publish(Twist())    # 退出前确保机器人停下
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
