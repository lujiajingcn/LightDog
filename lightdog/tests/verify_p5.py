#!/usr/bin/env python3
"""P5 端到端验收：把「人」放倒，看状态机走完两级确认，再起身解除。

需要：
  - 仿真正在跑，P5 的 detector + alert_manager 已拉起
  - 场景里有名为 person 的实体，且在机器人相机视野内（默认正前方 1.2m）
  - P4 的 recorder 也在跑（否则事件没有视频，其余检查仍成立）

用法（容器内；--ros-args 让计时走仿真时钟）：
    docker exec sim bash -lc 'source /opt/ros/jazzy/setup.bash; \
      source /home/ros/linorobot2_ws/install/setup.bash; \
      python3 /home/ros/linorobot2_ws/src/lightdog/tests/verify_p5.py'

布局：本脚本假设机器人在 (1.2, 0.5) 朝 -x，人放在 (0.0, 0.5)。
换位置改 PX/PY 和下面的 robot yaw 即可（先 bash sim/p3.sh pose 看真实位姿）。
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
from std_msgs.msg import Float32, String

WORLD, ENTITY = 'turtlebot3_world', 'person'
PX, PY = 0.0, 0.5
FAIL = []


def check(name, ok, detail=''):
    print(('PASS  ' if ok else 'FAIL  ') + name + (f'   {detail}' if detail else ''))
    if not ok:
        FAIL.append(name)


def set_pose(x, y, z, roll=0.0):
    """躺倒 = roll 90° + z 抬到躯干半径（0.17m），否则身体有一半埋进地面。"""
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
        super().__init__('p5_acceptance')
        self.fall = None
        self.z = None
        self.alert = None
        self.create_subscription(String, '/fall/state', self._on_fall, 10)
        self.create_subscription(Float32, '/fall/z_p95', self._on_z, 10)
        self.create_subscription(String, '/alert/state', self._on_alert, 10)

    def _on_fall(self, m):
        self.fall = m.data.strip()

    def _on_z(self, m):
        self.z = float(m.data)

    def _on_alert(self, m):
        raw = m.data.strip()
        try:
            self.alert = json.loads(raw).get('state', raw)
        except Exception:
            self.alert = raw

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9


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
    print('=== 0. 复位到干净状态 ===')
    try:
        http('/api/v1/alarms/reset', 'POST', {})
    except Exception as e:
        print('  reset 失败（可能本来就不在告警）:', e)
    set_pose(PX, PY, 0.0, 0.0)
    time.sleep(6)
    print(f'  fall={node.fall} z_p95={node.z} alert={node.alert}')
    check('起始为 standing', node.fall == 'standing',
          f'z_p95={None if node.z is None else round(node.z, 2)}')
    check('起始告警为 NORMAL', node.alert == 'NORMAL', str(node.alert))

    print('\n=== 1. 让人躺倒（roll=90°, z=0.17）===')
    t_fall, sim_fall = time.time(), node.now()
    if not set_pose(PX, PY, 0.17, roll=math.pi / 2):
        check('set_pose 放倒', False, 'gz service 调用失败')
        return 1

    seen, last = {}, None
    deadline = time.time() + 150
    while time.time() < deadline:
        cur = (node.fall, node.alert)
        if cur != last:
            zt = None if node.z is None else round(node.z, 2)
            print(f'  [{time.time()-t_fall:6.1f}s wall / t=+{node.now()-sim_fall:5.1f}s sim] '
                  f'fall={cur[0]:8s} z_p95={zt} alert={cur[1]}')
            seen[cur[1]] = node.now() - sim_fall
            last = cur
        if node.alert == 'ALARM':
            break
        time.sleep(1.0)

    check('检测到 lying', node.fall == 'lying',
          f"z_p95={None if node.z is None else round(node.z, 2)}（<0.50 应判 lying）")
    check('进入 SUSPECTED', 'SUSPECTED' in seen, f"t=+{seen.get('SUSPECTED', 0):.1f}s")
    check('进入 CONFIRMING', 'CONFIRMING' in seen, f"t=+{seen.get('CONFIRMING', 0):.1f}s")
    check('进入 ALARM', 'ALARM' in seen, f"t=+{seen.get('ALARM', 0):.1f}s")
    if 'SUSPECTED' in seen and 'CONFIRMING' in seen:
        d = seen['CONFIRMING'] - seen['SUSPECTED']
        check('一级确认约 5s（疑躺 -> 问询）', 3.5 <= d <= 8.0, f'{d:.1f}s sim')
    if 'CONFIRMING' in seen and 'ALARM' in seen:
        d = seen['ALARM'] - seen['CONFIRMING']
        check('二级确认约 15s（问询 -> 告警）', 12.0 <= d <= 20.0, f'{d:.1f}s sim')

    print('\n=== 2. 告警事件落到 P4（等后置窗口 + 合成）===')
    t_alarm, ev, deadline = time.time(), None, time.time() + 120
    while time.time() < deadline:
        evs = http('/api/v1/events')['events']
        # 只看**这次**告警之后写的事件：历史里可能还有上一轮/手动测试的 fall 事件
        mine = [e for e in evs
                if e.get('type') == 'fall' and e.get('has_video')
                and e.get('wall', 0) >= t_alarm - 5]
        if mine:
            ev = mine[0]
            break
        time.sleep(4)

    check('生成 fall 事件（本次告警）', ev is not None,
          '' if ev is None else ev['id'])
    if ev:
        check('事件带视频', ev.get('has_video') is True,
              f"frames={ev.get('frames')} bytes={ev.get('bytes')} codec={ev.get('codec')}")
        check('事件窗口是 25+10s', ev.get('pre_sec') == 25.0 and ev.get('post_sec') == 10.0,
              f"pre={ev.get('pre_sec')} post={ev.get('post_sec')} dur={ev.get('duration')}")
        print(f"  事件：{ev['id']}  {ev.get('label')}  {ev.get('bytes', 0)/1024:.0f}KB")
        try:
            with urllib.request.urlopen(
                    'http://127.0.0.1:8000/api/v1/events/%s/video' % ev['id'],
                    timeout=30) as r:
                vb = r.read()
            open('/tmp/_evt.mp4', 'wb').write(vb)
            import cv2
            cap = cv2.VideoCapture('/tmp/_evt.mp4')
            n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
            ok, first = cap.read()
            cap.set(cv2.CAP_PROP_POS_FRAMES, n // 2)
            ok2, mid = cap.read()
            cap.set(cv2.CAP_PROP_POS_FRAMES, max(0, n - 2))
            ok3, tail = cap.read()
            cap.release()
            check('事件视频可解码', ok and n > 50, f'{n} 帧')
            if ok:
                cv2.imwrite('/data/_evt_first.jpg', first)
            if ok2:
                cv2.imwrite('/data/_evt_mid.jpg', mid)
            if ok3:
                cv2.imwrite('/data/_evt_tail.jpg', tail)
            print('  首/中/尾帧已存 /data/_evt_{first,mid,tail}.jpg，肉眼可核对'
                  '「首帧站着、中段躺地」')
        except Exception as e:
            check('事件视频下载', False, repr(e))

    print('\n=== 3. 手机端 /status 与 /alarms ===')
    s = http('/api/v1/status')
    a = s['alert']
    check('/status.alert = ALARM', a['state'] == 'ALARM',
          json.dumps({k: a[k] for k in ('state', 'alarm_id', 'contact_count',
                                        'call_attempts', 'rounds', 'call_enabled')},
                     ensure_ascii=False))
    check('/status.contacts 读到联系人', s['contacts'] >= 1, str(s['contacts']))
    check('/alarms 有当前告警', http('/api/v1/alarms')['current']['state'] == 'ALARM',
          f"历史 {len(http('/api/v1/alarms')['alarms'])} 条")

    print('\n=== 4. 人起身 -> RESOLVED -> NORMAL ===')
    t_up, sim_up = time.time(), node.now()
    set_pose(PX, PY, 0.0, roll=0.0)
    deadline, last = time.time() + 90, None
    while time.time() < deadline:
        if node.alert != last:
            print(f'  [{time.time()-t_up:6.1f}s wall / t=+{node.now()-sim_up:5.1f}s sim] '
                  f'fall={node.fall} alert={node.alert}')
            last = node.alert
        if node.alert == 'NORMAL':
            break
        time.sleep(1.0)
    check('起身后回到 NORMAL', node.alert == 'NORMAL', f'当前 {node.alert}')
    check('起身后判 standing', node.fall == 'standing',
          f'z_p95={None if node.z is None else round(node.z, 2)}')

    print('\n=== 汇总 ===')
    print('FAILED: ' + (', '.join(FAIL) if FAIL else 'none'))
    return 1 if FAIL else 0


if __name__ == '__main__':
    sys.exit(main())
