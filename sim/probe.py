#!/usr/bin/env python3
"""P2 现场探针：只订阅、只统计，不改任何状态。"""
import sys
import time

import rclpy
from rclpy.node import Node


class Probe(Node):

    def __init__(self, secs):
        super().__init__('p2_probe')
        self.secs = secs
        self.t0 = time.monotonic()
        self.n = {}
        self.last = {}
        self.frames = []
        self.pos_times = []

        from std_msgs.msg import Bool, Float32
        from geometry_msgs.msg import PointStamped, Twist
        self.create_subscription(Bool, '/person/present', self._mk('present'), 10)
        self.create_subscription(PointStamped, '/person/position', self._mk('pos'), 10)
        self.create_subscription(Twist, '/cmd_vel_follow', self._mk('follow'), 10)
        self.create_subscription(Twist, '/cmd_vel_muxed', self._mk('mux'), 10)
        self.create_subscription(Float32, '/led/brightness', self._mk('led'), 10)

    def _mk(self, key):
        def cb(msg):
            self.n[key] = self.n.get(key, 0) + 1
            self.last[key] = msg
            if key == 'pos':
                now = time.monotonic()
                if self.pos_times:
                    self.frames.append(now - self.pos_times[-1])
                self.pos_times.append(now)
        return cb

    def done(self):
        return (time.monotonic() - self.t0) > self.secs

    def report(self):
        el = time.monotonic() - self.t0
        print(f'--- 观测 {el:.1f}s ---')
        for k in ('present', 'pos', 'follow', 'mux', 'led'):
            c = self.n.get(k, 0)
            print(f'{k:8s} {c:5d} 条  {c / el:5.2f} Hz')
        p = self.last.get('pos')
        if p is not None:
            print(f'pos     frame_id={p.header.frame_id!r} '
                  f'x={p.point.x:.3f} y={p.point.y:+.3f} z={p.point.z:.3f}')
        b = self.last.get('present')
        if b is not None:
            print(f'present 当前={bool(b.data)}')
        f = self.last.get('follow')
        if f is not None:
            print(f'follow  v={f.linear.x:+.3f} w={f.angular.z:+.3f}')
        m = self.last.get('mux')
        if m is not None:
            print(f'mux     v={m.linear.x:+.3f} w={m.angular.z:+.3f}')
        l = self.last.get('led')
        if l is not None:
            print(f'led     {l.data:.3f}')
        if self.frames:
            fs = sorted(self.frames)
            mid = fs[len(fs) // 2]
            print(f'pos 帧间隔 中位 {mid:.3f}s  最大 {fs[-1]:.3f}s  '
                  f'≈ {1.0 / mid:.2f} Hz')


def main():
    secs = float(sys.argv[1]) if len(sys.argv) > 1 else 15.0
    rclpy.init()
    node = Probe(secs)
    try:
        while rclpy.ok() and not node.done():
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    node.report()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
