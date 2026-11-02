#!/usr/bin/env python3
"""真机急停控制台 —— 查看 / 置位 / 解除急停闭锁。

为什么不能在 shell 里拼 `ros2 topic pub /safety/estop ... --once`：

  1. `--once` 发一条就退出，RELIABLE 的样本还没来得及被所有订阅者 ACK
     就随发布者一起析构了。实测 `-r 10 -t 6` 连发 5 轮，急停只到了 1 轮，
     而且出现过「mux 收到了、mission 没收到」—— 一个锁上、一个还自以为能动。
  2. 急停是**状态**不是事件。发完必须回读，确认锁存里真的是你要的值。

所以这里做三件事：发 → 回读锁存 → 回读 mux 实际生效的闸。
任一步不成立就退非零，绝不打印「已解除」了事。

用法：
    lightdog-estop                 # 等同 status
    lightdog-estop status
    lightdog-estop set             # 置位（立刻刹车）
    lightdog-estop clear --yes     # 解除（必须显式确认）
"""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from std_msgs.msg import Bool, String

ESTOP_TOPIC = '/safety/estop'
MUX_STATE_TOPIC = '/cmd_vel_mux_state'
# 与 lightdog_qos.SAFETY_QOS 必须逐字一致：QoS 不兼容时两边根本连不上，
# 而且不会报错 —— 只表现为「发出去没人收到」。deploy/ 目录独立于包，
# 所以这里内联而不是 import。
SAFETY_QOS = QoSProfile(
    depth=1,
    history=QoSHistoryPolicy.KEEP_LAST,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)

OK = '\033[32m'
BAD = '\033[31m'
WARN = '\033[33m'
END = '\033[0m'


class EstopConsole(Node):
    def __init__(self):
        super().__init__('lightdog_estop_cli')
        self.latched = None      # 话题上的锁存值
        self.mux_state = None    # mux 实际在用的闸
        self.pub = self.create_publisher(Bool, ESTOP_TOPIC, SAFETY_QOS)
        self.create_subscription(Bool, ESTOP_TOPIC, self._on_estop, SAFETY_QOS)
        self.create_subscription(String, MUX_STATE_TOPIC,
                                 self._on_mux, 10)

    def _on_estop(self, msg):
        self.latched = bool(msg.data)

    def _on_mux(self, msg):
        self.mux_state = msg.data

    def spin_for(self, sec):
        end = time.monotonic() + sec
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)


def wait_latched(node, want, timeout):
    """等锁存变成 want。返回 (成功, 实际值)。"""
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
        if node.latched is want:
            return True, node.latched
    return False, node.latched


def main():
    ap = argparse.ArgumentParser(
        prog='lightdog-estop',
        description='查看/设置/解除 LightDog 急停闭锁')
    ap.add_argument('action', nargs='?', default='status',
                    choices=['status', 'set', 'clear'])
    ap.add_argument('--yes', action='store_true',
                    help='解除急停时需要，避免手滑把锁解开')
    ap.add_argument('--timeout', type=float, default=6.0)
    args = ap.parse_args()

    rclpy.init()
    node = EstopConsole()
    try:
        # 先收 1.5s：TRANSIENT_LOCAL 的锁存样本进来得很快，但要等发现完成
        node.spin_for(1.5)

        if args.action == 'status':
            return report(node)

        want = (args.action == 'set')
        if not want and not args.yes:
            print(f'{BAD}拒绝解除急停：{END}需要显式加 --yes。'
                  f'（解开锁之前请确认车周围没人）')
            return 3

        subs = node.count_subscribers(ESTOP_TOPIC)
        if subs == 0:
            print(f'{WARN}!{END} /safety/estop 当前没有订阅者 '
                  f'—— twist_mux 可能没在跑，这次设置**不会**让车停下来')

        # 发。连发几次覆盖发现窗口，但真正的保证是后面的回读。
        for _ in range(5):
            node.pub.publish(Bool(data=want))
            node.spin_for(0.1)

        got, actual = wait_latched(node, want, args.timeout)
        if not got:
            print(f'{BAD}✗ 失败{END} 锁存没变成 {want}（实际 {actual}）。'
                  f'检查 twist_mux 是否在跑：systemctl status lightdog-safety-mux')
            return 1

        # 等 mux 反应。mux 20Hz 出状态，0.5s 足够。
        node.spin_for(0.8)
        if node.mux_state is None:
            print(f'{WARN}!{END} 锁存已写入 {want}，但收不到 {MUX_STATE_TOPIC} '
                  f'—— 无法确认 mux 实际生效')
            return 2
        if want:
            if node.mux_state != 'estop':
                print(f'{BAD}✗ 锁存是 true，但 mux 当前闸是 '
                      f'{node.mux_state!r} 而不是 estop')
                return 1
            print(f'{OK}✓{END} 急停已生效：锁存=true，mux 闸=estop（车不会再动）')
        else:
            if node.mux_state == 'estop':
                print(f'{BAD}✗ 锁存已解，但 mux 仍报 estop{END} —— '
                      f'它可能还没收到，稍等再 status 一次')
                return 1
            print(f'{OK}✓{END} 急停已解除：锁存=false，mux 闸={node.mux_state}')
        return report(node, brief=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def report(node, brief=False):
    latched = node.latched
    print()
    if latched is None:
        print(f'  锁存    : {WARN}读不到{END}（twist_mux 没跑，或话题还没发现）')
    elif latched:
        print(f'  锁存    : {BAD}已闭锁{END}（车不动是刻意的）')
    else:
        print(f'  锁存    : {OK}未闭锁{END}')

    print(f'  mux 闸  : {node.mux_state if node.mux_state else "读不到"}')
    if not brief and latched:
        print('\n  解除：lightdog-estop clear --yes')
    return 0


if __name__ == '__main__':
    sys.exit(main())
