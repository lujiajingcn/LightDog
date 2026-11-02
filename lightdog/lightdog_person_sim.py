#!/usr/bin/env python3
"""
仿真「人」的移动控制（仅仿真，真机不需要）

## 为什么用 set_pose 而不是给模型加插件

真机上「人」自己会走，仿真里得有人驱动。两条路：

1. 给人加 DiffDrive 插件，用 cmd_vel 控制 —— 需要模型参与动力学（static=false），
   圆柱人形没有自平衡能力，仿真里会直接倒下，属于和产品逻辑无关的噪音。
2. **保持 static=true，用 /world/<world>/set_pose 服务直接搬运** —— 稳定、可控，
   也不影响激光/相机对它的感知。

这里选 2。代价是每次设置位姿要起一次 `gz service` 子进程（~100-200ms），
所以默认 5Hz，人走 0.3m/s 时每帧位移 6cm，视觉上足够连续。

若后续要更平滑，可换成 gz-transport 的 Python 绑定（镜像里不一定有），
或者改用带动画轨迹的 Actor（需要 walk.dae 之类资源，当前没有）。

## 路径

默认一条直线往返，模拟「人从床边走向卫生间再走回来」：
    (1.0, 0.0) -> (4.0, 0.0) -> (1.0, 0.0) ...
可用 waypoints 参数改成任意折线。
"""

import math
import subprocess
import sys

import rclpy
from rclpy.node import Node


class PersonSim(Node):

    def __init__(self):
        super().__init__('lightdog_person_sim')

        self.declare_parameter('world_name', 'turtlebot3_world')
        self.declare_parameter('entity_name', 'person')
        self.declare_parameter('waypoints', [1.0, 0.0, 4.0, 0.0])
        self.declare_parameter('speed', 0.30)        # m/s
        self.declare_parameter('update_hz', 5.0)
        self.declare_parameter('enabled', True)

        p = self.get_parameter
        self.world = p('world_name').value
        self.entity = p('entity_name').value
        flat = list(p('waypoints').value)
        if len(flat) < 4 or len(flat) % 2:
            raise SystemExit('waypoints 需要偶数个、至少 4 个（x0,y0,x1,y1,...）')
        self.wps = [(float(flat[i]), float(flat[i + 1]))
                    for i in range(0, len(flat), 2)]
        self.speed = float(p('speed').value)
        self.enabled = bool(p('enabled').value)
        hz = max(0.5, float(p('update_hz').value))

        # 预计算每段长度与总长
        self.seg_len = []
        total = 0.0
        for i in range(len(self.wps)):
            a = self.wps[i]
            b = self.wps[(i + 1) % len(self.wps)]
            d = math.hypot(b[0] - a[0], b[1] - a[1])
            self.seg_len.append(d)
            total += d
        self.total = total
        if self.total < 1e-3:
            raise SystemExit('waypoints 总长度为零')
        self.dist = 0.0

        self.create_timer(1.0 / hz, self._tick)

        self.get_logger().info(
            f'LightDog person-sim ready: entity={self.entity} world={self.world} '
            f'waypoints={self.wps} speed={self.speed:.2f}m/s '
            f'loop={self.total:.2f}m enabled={self.enabled}')

    def _pose_at(self, dist):
        """按沿路径的累计距离返回 (x, y, yaw)。"""
        d = dist % self.total
        for i, seg in enumerate(self.seg_len):
            if d <= seg or i == len(self.seg_len) - 1:
                a = self.wps[i]
                b = self.wps[(i + 1) % len(self.wps)]
                t = 0.0 if seg < 1e-6 else d / seg
                x = a[0] + (b[0] - a[0]) * t
                y = a[1] + (b[1] - a[1]) * t
                yaw = math.atan2(b[1] - a[1], b[0] - a[0])
                return x, y, yaw
            d -= seg
        a = self.wps[0]
        return a[0], a[1], 0.0

    def _set_pose(self, x, y, yaw):
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        req = (f'name: "{self.entity}", '
               f'position: {{x: {x:.4f}, y: {y:.4f}, z: 0.0}}, '
               f'orientation: {{x: 0.0, y: 0.0, z: {qz:.6f}, w: {qw:.6f}}}')
        cmd = ['gz', 'service',
               '-s', f'/world/{self.world}/set_pose',
               '--reqtype', 'gz.msgs.Pose',
               '--reptype', 'gz.msgs.Boolean',
               '--timeout', '800',
               '--req', req]
        try:
            r = subprocess.run(cmd, capture_output=True, timeout=3.0)
            if r.returncode != 0:
                self.get_logger().warn(
                    f'set_pose 失败 rc={r.returncode}: '
                    f'{r.stderr.decode(errors="replace")[:160]}',
                    throttle_duration_sec=5.0)
        except subprocess.TimeoutExpired:
            self.get_logger().warn('set_pose 超时', throttle_duration_sec=5.0)
        except FileNotFoundError:
            self.get_logger().error('找不到 gz 命令，检查容器内 PATH')
            self.enabled = False

    def _tick(self):
        if not self.enabled:
            return
        # 用**实际**经过的时间积分，而不是写死 1/update_hz。
        # 定时器一旦被 set_pose 的子进程阻塞（每次 100~200ms），固定步长会让
        # 人走得比设定速度慢；更糟的是本机实时因子只有 0.22x，用墙钟算的人
        # 相对机器人会快 4.5 倍。这里取 ROS 时钟（use_sim_time: true），
        # 让人和车处在同一个时间基准里。
        now = self.get_clock().now()
        last = getattr(self, '_last_move', None)
        self._last_move = now
        if last is None:
            return
        dt = (now - last).nanoseconds * 1e-9
        if dt <= 0.0:
            return
        dt = min(dt, 1.0)       # 时钟跳变（重启 gazebo）时别让人瞬移
        self.dist += self.speed * dt
        x, y, yaw = self._pose_at(self.dist)
        self._set_pose(x, y, yaw)


def main(args=None):
    rclpy.init(args=args)
    node = PersonSim()
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
