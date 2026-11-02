#!/usr/bin/env python3
"""P5 补充验收：误报抑制 + 呼叫/确认链路。

A. 躺 3 秒就起身（弯腰/蹲下）→ 只到 SUSPECTED 就退回 NORMAL，不报警
B. 真实跌倒 → 拨打第一位联系人 → 手机端 ack → 之后不再外呼

时间都是**仿真秒**：本机 RTF≈0.42，所以 call_interval=20s 在墙钟上约 48s。
B 段整体约 2.5 分钟，别用短超时跑。

用法（容器内）：
    docker exec sim bash -lc 'source /opt/ros/jazzy/setup.bash; \
      source /home/ros/linorobot2_ws/install/setup.bash; \
      python3 /home/ros/linorobot2_ws/src/lightdog/tests/verify_p5_extra.py'
"""
import json
import math
import subprocess
import sys
import threading
import time
import urllib.request

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

WORLD, ENTITY, PX, PY = 'turtlebot3_world', 'person', 0.0, 0.5
FAIL = []


def check(name, ok, detail=''):
    print(('PASS  ' if ok else 'FAIL  ') + name + (f'   {detail}' if detail else ''))
    if not ok:
        FAIL.append(name)


def set_pose(x, y, z, roll=0.0):
    qx, qw = math.sin(roll / 2.0), math.cos(roll / 2.0)
    req = (f'name: "{ENTITY}", position: {{x: {x:.3f}, y: {y:.3f}, z: {z:.3f}}}, '
           f'orientation: {{x: {qx:.5f}, y: 0.0, z: 0.0, w: {qw:.5f}}}')
    return subprocess.run(['gz', 'service', '-s', f'/world/{WORLD}/set_pose',
                           '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
                           '--timeout', '2000', '--req', req],
                          capture_output=True, timeout=8).returncode == 0


def http(path, method='GET', body=None):
    data = json.dumps(body).encode() if body is not None else None
    req = urllib.request.Request('http://127.0.0.1:8000' + path, data=data, method=method,
                                 headers={'Content-Type': 'application/json'})
    with urllib.request.urlopen(req, timeout=10) as r:
        return json.loads(r.read())


class W(Node):
    def __init__(self):
        super().__init__('p5_extra')
        self.fall = None
        self.alert = None
        self.create_subscription(String, '/fall/state',
                                 lambda m: setattr(self, 'fall', m.data.strip()), 10)
        self.create_subscription(String, '/alert/state', self._on_alert, 10)

    def _on_alert(self, m):
        raw = m.data.strip()
        try:
            self.alert = json.loads(raw).get('state', raw)
        except Exception:
            self.alert = raw


def wait_until(pred, timeout, step=0.5):
    """返回耗时（秒），超时返回 None。"""
    t0 = time.time()
    while time.time() - t0 < timeout:
        if pred():
            return time.time() - t0
        time.sleep(step)
    return None


def attempts():
    return http('/api/v1/status')['alert']['call_attempts']


def main():
    rclpy.init()
    node = W()
    th = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    th.start()
    try:
        return run(node)
    finally:
        rclpy.shutdown()
        th.join(timeout=3)
        node.destroy_node()


def run(node):
    http('/api/v1/alarms/reset', 'POST', {})
    time.sleep(3)
    set_pose(PX, PY, 0.0, 0.0)
    wait_until(lambda: node.fall == 'standing', 15)
    print(f'基线：fall={node.fall} alert={node.alert}')

    # ---- A. 短促躺倒（弯腰/蹲下）不应报警 ----
    print('\n=== A. 躺倒 3s 后立刻起身（误报抑制）===')
    set_pose(PX, PY, 0.17, math.pi / 2)
    t = wait_until(lambda: node.fall == 'lying', 15)
    check('检测到 lying', t is not None, f'{t:.1f}s')
    t = wait_until(lambda: node.alert == 'SUSPECTED', 15)
    check('进入 SUSPECTED', t is not None, f'{t:.1f}s')
    time.sleep(3.0)                       # 只躺 3s（< suspect_hold 5s）
    set_pose(PX, PY, 0.0, 0.0)            # 起身
    t = wait_until(lambda: node.alert == 'NORMAL', 30)
    check('起身后回 NORMAL（未误报）', t is not None,
          f'{t:.1f}s' if t else f'卡在 {node.alert}')
    check('全程没进 CONFIRMING/ALARM', node.alert == 'NORMAL', f'当前 {node.alert}')

    # ---- B. 真实跌倒 -> 呼叫 -> ack -> 停止外呼 ----
    print('\n=== B. 真实跌倒 -> 拨打联系人 -> ack -> 停止外呼 ===')
    set_pose(PX, PY, 0.17, math.pi / 2)
    t = wait_until(lambda: node.alert == 'ALARM', 150, 1.0)
    check('进入 ALARM', t is not None, f'{t:.0f}s wall' if t else node.alert)
    if t is None:
        return 1

    t = wait_until(lambda: len(attempts()) >= 1, 20, 1.0)
    check('告警后拨打了第一位联系人', t is not None,
          json.dumps(attempts()[:1], ensure_ascii=False))
    n_before = len(attempts())

    http('/api/v1/alarms/ack', 'POST', {})
    t = wait_until(lambda: node.alert == 'ACKED', 15)
    check('ack 后进入 ACKED', t is not None, node.alert)

    print(f'=== 等 70s（> 1 个 call_interval）确认不再外呼（ack 前 {n_before} 次）===')
    time.sleep(70)
    n_after = len(attempts())
    check('ACKED 后不再新增呼叫', n_after == n_before, f'{n_before} -> {n_after} 次')

    print('=== 收尾：起身 + reset ===')
    set_pose(PX, PY, 0.0, 0.0)
    time.sleep(5)
    http('/api/v1/alarms/reset', 'POST', {})
    wait_until(lambda: node.alert == 'NORMAL', 20)
    check('回到 NORMAL', node.alert == 'NORMAL', node.alert)

    print('\n=== 汇总 ===')
    print('FAILED: ' + (', '.join(FAIL) if FAIL else 'none'))
    return 1 if FAIL else 0


if __name__ == '__main__':
    sys.exit(main())
