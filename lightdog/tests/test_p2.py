"""P2 回归：人体检测 + 跟随 + 速度仲裁。

对应验收脚本 sim/p2.sh，但不需要容器：这里重放的是「数据应该怎么变」，
不是「Gazebo 里车走了多远」。物理效果（轮子转速 → 位移）仿真才验得了。

覆盖这一轮修的四个点：
  1. 相机断流后不能拿最后一帧继续判「有人」（幽灵跟随）
  2. 同一帧不重复检测（否则 10Hz 定时器对着一张旧图跑满 10 次）
  3. 仲裁优先级 dock > follow > nav，且同优先级按列表顺序
  4. 双窗口：源活着（timeout）但指令过期（hold）→ 必须发零速并标 stale
  5. 跟随器被 inhibit 后，若 dock_manager 长时间不再发标志要能自行恢复
"""

import os
import sys
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from harness import (  # noqa: E402
    Bool, Clock, Log, Msg, PointStamped, Pub, String, Twist,
    bind, extract_class, extract_funcs, extract_methods, source_of,
)

DETECTOR = source_of('lightdog_person_detector.py')
MUX = source_of('lightdog_twist_mux.py')
FOLLOWER = source_of('lightdog_follower.py')


class _Image:
    """只用于满足函数注解（rclpy 的 msg 类型在宿主机上不存在）。"""


class _PointStamped:
    """同上。"""


# ------------------------------------------------------------------ 被测对象

def make_detector(detection=(1.20, 0.0, 1.10, (10, 10, 40, 80))):
    d = type('Detector', (), {})()
    d.rgb = object()          # 占位：只要不是 None
    d.fx = 500.0
    d.rgb_wall = time.monotonic()
    d.rgb_seq = 1
    d._det_seq = -1
    d.frame_timeout = 1.5
    d.lost_timeout = 1.0
    d.present = False
    d.last_seen = None            # 墙钟时刻（time.monotonic()）
    d._warned_rgb_stall = False
    d._warned_no_input = False
    d.pub_present = Pub()
    d.pub_pos = Pub()
    d.clock = Clock()
    d.log = Log()
    d.get_clock = lambda: d.clock
    d.get_logger = lambda: d.log
    d.detect_calls = 0

    def _detect():
        d.detect_calls += 1
        return detection

    d._detect = _detect
    d._publish_position = lambda *a: d.pub_pos.publish(Msg(a[0]))
    bind(d, extract_methods(DETECTOR, 'PersonDetector',
                            {'_tick', '_person_expired',
                             '_on_rgb', '_on_depth'},
                            ns={'time': time, 'Bool': Bool, 'Image': _Image}))
    return d


def make_mux():
    Src = extract_class(MUX, 'CmdVelSource')
    m = type('Mux', (), {})()
    m.sources = [
        Src('dock', '/cmd_vel_dock', 30, 1.0, 0.50),
        Src('follow', '/cmd_vel_follow', 20, 0.6, 0.30),
        Src('nav', '/cmd_vel_smoothed', 10, 0.6, 0.30),
    ]
    m.zero_on_idle = True
    m.max_lin = 0.50
    m.max_ang = 1.50
    m._warned_clamp = False
    m.pub = Pub()
    m.pub_state = Pub()
    m.pub_beat = Pub()
    m.pub_estop = Pub()
    m.active_name = None
    m._stale = False
    m.estop = False
    m.hold = False
    m._hold_stamp = 0.0
    m.hold_timeout = 3.0
    m.log = Log()
    m.get_logger = lambda: m.log
    clamp = extract_funcs(MUX, {'clamp_speed'})['clamp_speed']
    bind(m, extract_methods(MUX, 'TwistMux',
                            {'_tick', '_pick', '_on_twist', '_gate',
                             '_on_estop', '_on_hold'},
                            ns={'time': time, 'Twist': Twist, 'String': String,
                                'Bool': Bool,
                                'CmdVelSource': Src, 'clamp_speed': clamp}))
    return m


def make_follower():
    f = type('Follower', (), {})()
    f.enabled = True
    f.inhibited = False
    f.inhibit_stamp = 0.0
    f.inhibit_timeout = 5.0
    f.person = PointStamped(1.5, 0.0)
    # 墙钟口径：last_present / pos_wall 都是 time.monotonic() 的值
    f.last_present = time.monotonic()
    f.pos_wall = time.monotonic()
    f.expected_frame = 'base_link'
    f._warned_frame = ''
    f._warned_pos = False
    # 调度层放行开关：None = 没收到过，按自身 enabled 工作
    f.enable_cmd = None
    f.enable_stamp = 0.0
    f.enable_timeout = 5.0
    f._warned_enable = False
    f._silent = False
    f.timeout = 1.5
    f.target = 1.20
    f.tol = 0.15
    f.stop_d = 0.65
    f.k_v = 0.8
    f.max_v = 0.25
    f.ang_tol = 0.10
    f.k_w = 1.2
    f.max_w = 1.0
    f.side_only = 0.9
    f.pub = Pub()
    f.log = Log()
    f.get_clock = lambda: f.clock
    f.get_logger = lambda: f.log
    bind(f, extract_methods(FOLLOWER, 'PersonFollower',
                            {'_tick', '_person_lost', '_check_frame',
                             '_follow_gate', '_on_present', '_on_pos',
                             '_on_inhibit', '_on_enable'},
                            ns={'time': time, 'math': __import__('math'),
                                'Twist': Twist, 'Bool': Bool,
                                'PointStamped': PointStamped}))
    return f


# ------------------------------------------------------------------ 检测器

class TestDetectorFreshness(unittest.TestCase):

    def test_same_frame_detected_once(self):
        d = make_detector()
        for _ in range(3):
            d._tick()
        self.assertEqual(d.detect_calls, 1, '同一帧被重复检测')
        self.assertEqual(d.pub_present.values, [True, True, True],
                         '重发次数不对：下游靠这个周期刷新，漏发会让跟随超时')

    def test_stalled_camera_reports_absent(self):
        d = make_detector()
        d._tick()
        self.assertTrue(d.pub_present.last.data)

        d.rgb_wall = time.monotonic() - 2.0     # 断流 2s > frame_timeout
        d.pub_present.clear()
        d._tick()

        self.assertEqual(d.pub_present.values, [False], '断流后仍在报「有人」')
        self.assertIsNone(d.last_seen, '断流后 last_seen 没清，恢复时会误判')
        self.assertEqual(d.detect_calls, 1, '断流后还在检测旧帧')
        self.assertIn('没有新帧', d.log.text('warn'))

    def test_recovers_after_new_frame(self):
        d = make_detector()
        d._tick()
        d.rgb_wall = time.monotonic() - 2.0
        d._tick()                               # 判定断流
        d._on_rgb(object())                     # 新帧到达
        self.assertFalse(d._warned_rgb_stall, '恢复后没复位告警标志，日志会只打一次')

        d.pub_present.clear()
        d._tick()
        self.assertEqual(d.detect_calls, 2)
        self.assertEqual(d.pub_present.values, [True])

    def test_no_camera_publishes_false_and_warns_once(self):
        """话题名写错/相机没起来时，下游必须立刻看到 false，且日志里要有一条。"""
        d = make_detector()
        d.rgb = None
        for _ in range(5):
            d._tick()
        self.assertEqual(d.pub_present.values, [False] * 5,
                         '没有相机时必须持续发 false（下游只有这个 Bool 可看）')
        self.assertEqual(d.log.text('error').count('尚未收到'), 1, '告警刷屏了')

    def test_republish_ages_out_present_flag(self):
        """相机还在出帧、但这一帧没检出人：重发的 present 也要按 lost_timeout 老化。"""
        d = make_detector()
        d._tick()
        self.assertTrue(d.pub_present.last.data)

        d._detect = lambda: None
        d._on_rgb(object())                     # 新帧，但检测失败
        d.pub_present.clear()
        d._tick()                               # 刚丢一帧，仍在 lost_timeout 窗口内
        self.assertEqual(d.pub_present.values, [True],
                         '单帧丢检不该立刻报无人（灯会闪）')

        d.last_seen = time.monotonic() - 2.0    # 已超过 lost_timeout=1.0
        d.pub_present.clear()
        d._tick()                               # 没有新帧 → 走重发分支
        self.assertEqual(d.pub_present.values, [False],
                         '重发分支没有老化，连续丢检期间 present 会粘在 true')


# ------------------------------------------------------------------ 仲裁

class TestTwistMux(unittest.TestCase):

    def _feed(self, m, name, v, age=0.0):
        src = next(s for s in m.sources if s.name == name)
        m._on_twist(Twist(), src)               # 建立 stamp
        src.msg.linear.x = v
        src.stamp = time.monotonic() - age
        return src

    def test_priority_follow_beats_nav(self):
        m = make_mux()
        self._feed(m, 'nav', 0.10)
        self._feed(m, 'follow', 0.20)
        m._tick()
        self.assertAlmostEqual(m.pub.last.linear.x, 0.20)
        self.assertEqual(m.pub_state.last.data, 'follow')

    def test_dock_beats_follow(self):
        m = make_mux()
        self._feed(m, 'follow', 0.20)
        self._feed(m, 'dock', 0.05)
        m._tick()
        self.assertAlmostEqual(m.pub.last.linear.x, 0.05)
        self.assertEqual(m.pub_state.last.data, 'dock')

    def test_stale_after_upstream_death(self):
        """关键回归：源还在 timeout 窗口内（没被判死），但指令已过 hold。
        改之前这里会把最后一条 0.20m/s 继续转发，真机上等于盲走 20cm。"""
        m = make_mux()
        self._feed(m, 'follow', 0.20, age=0.40)   # hold=0.30 < 0.40 < timeout=0.6
        m._tick()
        self.assertTrue(m.pub.last.is_zero(), '指令过期后仍在转发非零速度')
        self.assertEqual(m.pub_state.last.data, 'follow(stale)')
        self.assertIn('没更新指令', m.log.text('warn'))

    def test_source_gone_falls_back_to_lower_priority(self):
        m = make_mux()
        self._feed(m, 'follow', 0.20, age=0.9)    # 超出 timeout=0.6
        self._feed(m, 'nav', 0.10, age=0.05)
        m._tick()
        self.assertAlmostEqual(m.pub.last.linear.x, 0.10)
        self.assertEqual(m.pub_state.last.data, 'nav')

    def test_idle_outputs_zero(self):
        m = make_mux()
        m._tick()
        self.assertTrue(m.pub.last.is_zero())
        self.assertEqual(m.pub_state.last.data, 'idle')

    def test_hold_cannot_exceed_timeout(self):
        Src = extract_class(MUX, 'CmdVelSource')
        s = Src('x', '/x', 1, 0.30, 0.90)          # 配置写反
        self.assertLessEqual(s.hold, s.timeout)

    def test_output_is_clamped_to_platform_limits(self):
        """上游算错（P 控制增益写大、Nav2 给了个离谱目标）时，最后一道闸。"""
        m = make_mux()
        self._feed(m, 'follow', 3.0)
        src = next(s for s in m.sources if s.name == 'follow')
        src.msg.angular.z = -9.0
        m._tick()
        self.assertAlmostEqual(m.pub.last.linear.x, m.max_lin)
        self.assertAlmostEqual(m.pub.last.angular.z, -m.max_ang)
        self.assertIn('已钳制', m.log.text('warn'))

        m.log.lines.clear()                         # 只报一次，不刷屏
        m._tick()
        self.assertEqual(m.log.text('warn'), '')

    def test_normal_speed_is_not_touched(self):
        m = make_mux()
        self._feed(m, 'follow', 0.20)
        m._tick()
        self.assertAlmostEqual(m.pub.last.linear.x, 0.20)
        self.assertEqual(m.log.text('warn'), '')


class TestTwistMuxAdaptive(unittest.TestCase):
    """窗口必须按上游实测周期放大。仿真 0.21x 实时因子下跟随节点墙钟只有
    4.8Hz（间隔最大 0.32s），固定 0.30s 的 hold 会被正常发包反复触发 ——
    实测表现为仲裁每 0.3s 抖一次、机器人几乎不动。"""

    def _src(self, period_s, hold=0.30, timeout=0.60):
        Src = extract_class(MUX, 'CmdVelSource')
        s = Src('follow', '/cmd_vel_follow', 20, timeout, hold)
        # 灌几条消息把周期学出来
        t = 100.0
        for _ in range(6):
            s.touch(t, Twist())
            t += period_s
        self.assertAlmostEqual(s.period, period_s, places=3)
        return s, t - period_s          # stamp 落在最后一条消息上

    def test_slow_upstream_does_not_look_stale(self):
        s, stamp = self._src(0.21)      # 4.8Hz 墙钟
        now = stamp + 0.32              # 实测最大间隔，配置 hold=0.30 会误判
        self.assertTrue(s.is_fresh(now), '慢速上游被误判为 stale')
        self.assertTrue(s.is_active(now))

    def test_dead_upstream_expires_within_a_few_periods(self):
        s, stamp = self._src(0.20)
        # fresh 窗 = max(0.30, 2*0.20)=0.40；active 窗 = max(0.60, 4*0.20)=0.80
        self.assertFalse(s.is_fresh(stamp + 0.45), '上游死了还在转发旧指令')
        self.assertTrue(s.is_active(stamp + 0.45), '这时还应算「源活着但指令过期」')
        self.assertFalse(s.is_active(stamp + 0.85), '超时后应掉出活跃集')

    def test_fast_upstream_keeps_configured_window(self):
        """真机 RTF≈1、10Hz 时自适应不该放宽配置值，否则等于削弱安全边际。"""
        s, stamp = self._src(0.10)
        self.assertAlmostEqual(s.fresh_window(), 0.30, places=3)
        self.assertAlmostEqual(s.alive_window(), 0.60, places=3)


# ------------------------------------------------------------------ 跟随

class TestFollower(unittest.TestCase):

    def test_zero_when_person_lost(self):
        f = make_follower()
        # 两条判据都要过期：present 到期 与 坐标到期
        f.last_present = time.monotonic() - 3.0     # 超过 timeout=1.5
        f.pos_wall = time.monotonic() - 3.0
        f._tick()
        self.assertTrue(f.pub.last.is_zero())
        self.assertTrue(f._person_lost())

    def test_zero_when_position_frozen_but_present_true(self):
        """检测器还在发 present=True，但坐标不再更新（连续丢检在
        lost_timeout 窗口内就是这种表现）。只看 present 会拿旧坐标继续走。"""
        f = make_follower()
        f._on_present(Bool(True))                   # present 是新鲜的
        f.pos_wall = time.monotonic() - 3.0         # 但坐标已经 3s 没动
        f._tick()
        self.assertTrue(f.pub.last.is_zero(), '坐标过期后仍在按旧坐标发速度')
        self.assertTrue(f._person_lost())
        self.assertIn('坐标已', f.log.text('warn'))

    def test_zero_when_frame_mismatch(self):
        """检测器换了实现、改发 camera_link：坐标含义完全不同，必须停。"""
        f = make_follower()
        f.person = PointStamped(1.5, 0.0, frame_id='camera_link')
        f._tick()
        self.assertTrue(f.pub.last.is_zero(), '坐标系不符时仍在用该坐标算速度')
        self.assertIn('expected_frame', f.log.text('error'))

    def test_slash_prefixed_frame_is_accepted(self):
        """TF 树里常见 `/base_link` 写法，不该被判成不符。"""
        f = make_follower()
        f.person = PointStamped(1.5, 0.0, frame_id='/base_link')
        f._tick()
        self.assertGreater(f.pub.last.linear.x, 0.0)

    def test_forward_when_far(self):
        f = make_follower()
        f._on_present(Bool(True))
        f._tick()
        self.assertGreater(f.pub.last.linear.x, 0.0, '人更远时应该前进')
        self.assertLessEqual(f.pub.last.linear.x, f.max_v)

    def test_zero_when_inhibited(self):
        f = make_follower()
        f._tick()
        moving = f.pub.last.linear.x
        self.assertGreater(moving, 0.0)

        f._on_inhibit(Bool(True))
        f._tick()
        self.assertTrue(f.pub.last.is_zero(), '回充占用时跟随还在发速度')

    def test_inhibit_expires_when_dock_manager_dies(self):
        """dock_manager 被 kill 后 /dock/inhibit 不再更新。陈旧的 true
        不能把跟随永久锁死 —— 现象是「车完全不动，日志里什么都没有」。"""
        f = make_follower()
        f._on_inhibit(Bool(True))
        f.inhibit_stamp = time.monotonic() - 6.0    # > inhibit_timeout=5.0
        f._tick()
        self.assertFalse(f.inhibited, 'inhibit 没有自动失效')
        self.assertGreater(f.pub.last.linear.x, 0.0, '恢复后仍然不动')
        self.assertIn('判定回充节点已退出', f.log.text('error'))


if __name__ == '__main__':
    unittest.main(verbosity=2)
