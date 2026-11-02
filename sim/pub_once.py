#!/usr/bin/env python3
"""发**一条**消息，并在退出前让 publisher 多活一会儿。

## 为什么不用 `ros2 topic pub`

`ros2 topic pub --once` 在本机（Docker Desktop + WSL2，CPU 被仿真吃满）会丢消息：
发布进程在 DDS 发现完成之前就退出了。于是大家改成连发 6 条
（`-r 10 -t 6`），但那是另一个坑的开始：

* 这台机器上 `-r 10` 实际只能跑到 2~3Hz —— 6 条要 3~6 秒才发完；
* 收端看到的是「同一条指令被执行 6 次」：语音会重复播报、`dock` 会被连点、
  TTL 日志里出现 6 行一样的记录。

正确的做法不是"多发几遍"，而是**发一条、但让 DataWriter 活到被确认**：
DDS 的 RELIABLE 会在 writer 存活期间重传，1 秒足够覆盖发现 + 重传。
收端因此只会收到一次。

## 用法（容器内）

    pub_once.py <topic> <type> '<yaml-body>' [hold_sec] [durable]

    pub_once.py /mission/command std_msgs/msg/String '{data: "dock"}'
    pub_once.py /safety/estop    std_msgs/msg/Bool   '{data: true}' 1.0 1

`durable=1` 时用 TRANSIENT_LOCAL（只给急停这类"状态而非事件"的话题用：
它让之后才启动的节点也能继承当前值）。默认 VOLATILE —— 命令类话题必须
VOLATILE，否则一个后启动的语音节点会把上一条命令重放一遍（比如自己开去回充）。
"""

import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

import yaml

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Empty, Float32, String

TYPES = {
    'std_msgs/msg/String': String,
    'std_msgs/msg/Bool': Bool,
    'std_msgs/msg/Float32': Float32,
    'std_msgs/msg/Empty': Empty,
    'geometry_msgs/msg/Twist': Twist,
}


def main(argv):
    if len(argv) < 4:
        sys.stderr.write(__doc__)
        return 2
    topic, tname, body = argv[1], argv[2], argv[3]
    hold = float(argv[4]) if len(argv) > 4 else 1.0
    durable = len(argv) > 5 and argv[5] not in ('0', 'false', '')
    wait_subs = float(argv[6]) if len(argv) > 6 else 3.0

    cls = TYPES.get(tname)
    if cls is None:
        sys.stderr.write(f'不支持的消息类型 {tname!r}，可用：{sorted(TYPES)}\n')
        return 2

    try:
        fields = yaml.safe_load(body) or {}
    except yaml.YAMLError as e:
        sys.stderr.write(f'body 不是合法 YAML：{e}\n')
        return 2
    if not isinstance(fields, dict):
        sys.stderr.write('body 必须是一个映射，例如 {data: "dock"}\n')
        return 2

    msg = cls()
    for k, v in fields.items():
        if not hasattr(msg, k):
            sys.stderr.write(f'{tname} 没有字段 {k!r}\n')
            return 2
        setattr(msg, k, v)

    rclpy.init(args=None)
    node = Node('lightdog_pub_once')
    try:
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=(QoSDurabilityPolicy.TRANSIENT_LOCAL if durable
                        else QoSDurabilityPolicy.VOLATILE),
        )
        pub = node.create_publisher(cls, topic, qos)

        def spin_until(deadline):
            while node.get_clock().now().nanoseconds * 1e-9 < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)

        # 先等「至少有一个订阅者」。这台机器 DDS 发现经常要 1~2 秒，
        # 而原来的做法只转 0.3 秒就发，然后数订阅者得 0 —— 既误报失败，
        # 也等于把「发早了」这个老问题又抄了一遍。
        # 等不到也照发（锁存类/还没起的订阅者场景），由返回码让调用方判断。
        now = node.get_clock().now().nanoseconds * 1e-9
        spin_until(now + wait_subs)
        subs_before = node.count_subscribers(topic)

        pub.publish(msg)
        # 关键：发完不要马上退出，留时间让 RELIABLE 完成 ACK/重传。
        now = node.get_clock().now().nanoseconds * 1e-9
        spin_until(now + max(0.0, hold))
        subs = node.count_subscribers(topic)
        print(f'sent {topic} ({tname}) body={fields} '
              f'subscribers={subs} (发前 {subs_before})')
        return 0 if subs > 0 else 3
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main(sys.argv))
