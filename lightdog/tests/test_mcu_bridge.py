"""下位机桥接（D3）的逻辑回归：钳位、急停闸、速度保持窗口。

不 import 那个模块本身 —— 它 `import termios`，宿主机（Windows）没有。
所有被测方法都用 ast 从源文件里抽出来执行，和其余测试同一套办法。

这三组是本轮真正会咬人的地方：

1. **停发 ≠ 停车**：`lightdog_follower` 让出控制权时「先发零速再停发」，
   如果桥接节点把最后一条速度照着发，车会带着它一直走。
2. **急停期间不能攒指令**：解急停的瞬间把几秒前的旧速度发出去，是最典型的惊车路径。
3. **NaN 不能透传**：NaN 参与比较恒为假，会绕过所有钳位直接变成「电机收到 NaN」。
"""

import math
import os
import sys
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from harness import (  # noqa: E402
    Bool, Float32, Msg, PKG, Twist, bind, extract_methods, source_of,
)

sys.path.insert(0, PKG)

import lightdog_mcu_proto as proto  # noqa: E402

BRIDGE = source_of('lightdog_mcu_bridge.py')

NS = {'math': math, 'time': time, 'proto': proto,
      'Bool': Bool, 'Float32': Float32, 'Twist': Twist}

#: 这三个是 @staticmethod：抽出来之后没有 self，**不能** bind（否则会多传一个 self）
STATIC = ('_clamp', '_pct_to_msg', '_yaw_to_quat')
METHODS = STATIC + ('_on_cmd', '_on_led', '_on_estop', '_cmd_to_send')


class FakeLog:
    def __init__(self):
        self.lines = []

    def _rec(self, level):
        def f(msg, **kw):
            self.lines.append((level, str(msg)))
        return f

    def __getattr__(self, name):
        return self._rec(name)

    def text(self, level=None):
        return '\n'.join(m for lv, m in self.lines if level is None or lv == level)


def make_bridge(estop=False):
    """只布置被测方法用到的属性。"""
    b = type('McuBridge', (), {})()
    b.estop = estop
    b.cmd_lin = 0.0
    b.cmd_ang = 0.0
    b.cmd_wall = 0.0
    b.cmd_hold = 0.30
    b.max_linear = 0.25
    b.max_angular = 0.70
    b.led = None
    b.led_sent = None
    b.sent = []
    b.log = FakeLog()
    b.get_logger = lambda: b.log
    b._send = lambda frame: (b.sent.append(frame), True)[1]
    funcs = extract_methods(BRIDGE, 'McuBridge', METHODS, ns=NS)
    for name in STATIC:
        setattr(b, name, funcs.pop(name))
    bind(b, funcs)
    return b


def make_twist(lin=0.0, ang=0.0):
    t = Twist()
    t.linear.x = lin
    t.angular.z = ang
    return t


class TestScalars(unittest.TestCase):

    def test_clamp_nan_is_zero(self):
        b = make_bridge()
        # NaN 进 < 和 > 都是假，原样透出去就是「电机收到 NaN」
        self.assertEqual(b._clamp(float('nan'), -0.25, 0.25), 0.0)

    def test_clamp_bounds(self):
        b = make_bridge()
        self.assertEqual(b._clamp(1.5, -0.25, 0.25), 0.25)
        self.assertEqual(b._clamp(-9.0, -0.25, 0.25), -0.25)
        self.assertEqual(b._clamp(0.1, -0.25, 0.25), 0.1)

    def test_pct_keeps_nan(self):
        b = make_bridge()
        v = b._pct_to_msg(float('nan'))
        self.assertNotEqual(v, v, 'NaN 必须原样传给 dock_manager，不能变 0（会触发低电回充）')

    def test_pct_clamped(self):
        b = make_bridge()
        self.assertEqual(b._pct_to_msg(1.2), 1.0)
        self.assertEqual(b._pct_to_msg(-0.1), 0.0)
        self.assertAlmostEqual(b._pct_to_msg(0.42), 0.42)

    def test_yaw_to_quat(self):
        b = make_bridge()
        z, w = b._yaw_to_quat(0.0)
        self.assertAlmostEqual(z, 0.0)
        self.assertAlmostEqual(w, 1.0)
        z, w = b._yaw_to_quat(math.pi / 2)
        self.assertAlmostEqual(z, math.sin(math.pi / 4))
        self.assertAlmostEqual(z * z + w * w, 1.0, places=9)

    def test_yaw_wraps_continuously(self):
        # 3.1415 和 -3.1415 是**同一个朝向**（差 2π）。四元数是双覆盖：
        # q 与 -q 表示同一旋转，所以判据看的是 |q1·q2| 而不是 q1·q2 ——
        # 写成 >0.999 会误判成「跳变」，而下游按最短路径插值只关心符号一致性。
        b = make_bridge()
        z1, w1 = b._yaw_to_quat(3.1415)
        z2, w2 = b._yaw_to_quat(-3.1415)
        self.assertGreater(abs(z1 * z2 + w1 * w2), 0.999)


class TestCmdPath(unittest.TestCase):

    def test_clamped_at_bridge(self):
        b = make_bridge()
        b._on_cmd(make_twist(1.5, -3.0))
        self.assertEqual(b.cmd_lin, 0.25)
        self.assertEqual(b.cmd_ang, -0.70)

    def test_estop_drops_command(self):
        """急停期间收到的速度必须丢掉，否则解急停瞬间会窜出去。"""
        b = make_bridge(estop=True)
        b.cmd_wall = 123.0
        b._on_cmd(make_twist(0.2, 0.1))
        self.assertEqual((b.cmd_lin, b.cmd_ang), (0.0, 0.0))
        self.assertEqual(b.cmd_wall, 123.0, '急停期间不该刷新「有新指令」的时刻')

    def test_hold_window_expires(self):
        b = make_bridge()
        now = 1000.0
        b.cmd_lin = 0.2
        b.cmd_ang = 0.1
        b.cmd_wall = now - 0.1
        self.assertEqual(b._cmd_to_send(now), (0.2, 0.1))
        b.cmd_wall = now - 0.31
        self.assertEqual(b._cmd_to_send(now), (0.0, 0.0),
                         '超过 cmd_hold 必须发零速 —— 「停发」不能等于「保持最后速度」')

    def test_estop_forces_zero(self):
        b = make_bridge(estop=True)
        b.cmd_lin = 0.2
        b.cmd_wall = 1000.0
        self.assertEqual(b._cmd_to_send(1000.0), (0.0, 0.0))

    def test_estop_frame_payload(self):
        b = make_bridge()
        b._on_estop(Msg(data=True))
        self.assertTrue(b.estop)
        self.assertEqual(len(b.sent), 1)
        frames = proto.Parser().feed(b.sent[0])
        self.assertEqual(frames[0].type, proto.T_ESTOP)
        self.assertTrue(proto.decode_estop(frames[0].payload))

    def test_estop_is_idempotent(self):
        """重复置位不该刷屏，也不该发第二帧 —— SAFETY_QOS 是状态不是事件。"""
        b = make_bridge()
        b._on_estop(Msg(data=True))
        b._on_estop(Msg(data=True))
        self.assertEqual(len(b.sent), 1)

    def test_estop_release_sends_frame(self):
        b = make_bridge(estop=True)
        b._on_estop(Msg(data=False))
        self.assertFalse(b.estop)
        self.assertEqual(len(b.sent), 1)
        frames = proto.Parser().feed(b.sent[0])
        self.assertFalse(proto.decode_estop(frames[0].payload))

    def test_led_nan_becomes_zero(self):
        b = make_bridge()
        b._on_led(Msg(data=float('nan')))
        self.assertEqual(b.led, 0.0)
        self.assertIn('NaN', b.log.text('error'))

    def test_led_clamped(self):
        b = make_bridge()
        b._on_led(Msg(data=1.7))
        self.assertEqual(b.led, 1.0)


if __name__ == '__main__':
    unittest.main(verbosity=2)
