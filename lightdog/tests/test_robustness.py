"""真机前的健壮性回归：外部输入、传感器失效、充电确认。

这三类问题在仿真里都**不会报错**，只在上真机后才暴露：

  1. 事件的 `type` 被拼进目录名 —— 带 `../` 就能写到 `events/` 之外；
     带 `/` 会在中间多建一级，表现为「事件列表里没有、磁盘却在涨」。
  2. 相机断流/被遮挡 —— 原地把「看不见」当成「人起来了」，一次遮挡就能把
     已经开始的跌倒确认流程抹掉；反过来，CONFIRMING 阶段传感器全丢时
     又必须走到 ALARM（fail-safe），不能静默复位。
  3. 停靠成功 ≠ 充上电 —— docking_server 只保证「停到位」，触点没接上时
     界面照样显示 DOCKED、电量一动不动。
  4. 检测器崩掉后 /person/present 不再有人发，最后那条 true 会永久粘住 ——
     自动灯会整夜亮着，而日志里没有任何异常。

运行方式与 test_p2/test_p3 相同（纯逻辑，不需要 ROS）。
"""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from harness import (  # noqa: E402
    Clock, Log, Msg, Pub, bind, extract_funcs, extract_methods, source_of,
)

REC = source_of('lightdog_recorder.py')
ALERT = source_of('lightdog_alert_manager.py')
DOCK = source_of('lightdog_dock_manager.py')
LIGHT = source_of('lightdog_light_auto.py')

_safe = extract_funcs(REC, {'safe_token', 'clamp_sec'})
safe_token = _safe['safe_token']
clamp_sec = _safe['clamp_sec']

SUSPECTED, CONFIRMING, ALARM, NORMAL = (
    'SUSPECTED', 'CONFIRMING', 'ALARM', 'NORMAL')
# 真类上是类属性，fake 实例得自己带上（tick 里写的是 self.SUSPECTED）
STATES = {'NORMAL': 'NORMAL', 'SUSPECTED': 'SUSPECTED',
          'CONFIRMING': 'CONFIRMING', 'ALARM': 'ALARM',
          'ACKED': 'ACKED', 'RESOLVED': 'RESOLVED'}


# ------------------------------------------------------------------ 事件输入

class TestEventInput(unittest.TestCase):

    def test_type_cannot_escape_events_dir(self):
        """type 无论传什么，拼出来的目录必须还在 events/ 下面。"""
        root = os.path.abspath('/data/events')
        for raw in ('../../evil', '..', 'a/b', '..\\..\\x', '/etc/passwd',
                    '', None, '跌倒/fall', 'x' * 200, 'seg_000001'):
            tok = safe_token(raw)
            self.assertTrue(tok, f'{raw!r} 净化后为空')
            self.assertTrue(tok.isascii(), f'{raw!r} -> {tok!r} 含非 ASCII')
            self.assertNotIn('/', tok)
            self.assertNotIn('\\', tok)
            self.assertNotIn('.', tok)
            self.assertLessEqual(len(tok), 24)

            eid = 'evt_20260101_000000_%s' % tok
            full = os.path.abspath(os.path.join(root, eid))
            self.assertTrue(
                full.startswith(root + os.sep),
                f'{raw!r} -> {full} 跑到 events/ 之外了')

    def test_window_is_clamped(self):
        self.assertEqual(clamp_sec(1e9), 300.0)
        self.assertEqual(clamp_sec(-5), 0.0)
        self.assertEqual(clamp_sec('abc', default=10.0), 10.0)
        self.assertEqual(clamp_sec(float('nan'), default=10.0), 10.0)
        self.assertEqual(clamp_sec(None, default=25.0), 25.0)
        self.assertEqual(clamp_sec(25), 25.0)


# -------------------------------------------------------------- 告警状态机

def make_alert():
    d = type('AlertManager', (), {})()
    d.state = NORMAL
    d.z_p95 = None
    d.last_lying = None
    d.last_standing = None
    d.last_absent = None
    d.absent_since = None
    d.standing_since = None
    d.clock = Clock()
    d.since = 0.0
    d.suspect_hold = 5.0
    d.lying_gap = 1.0
    d.absent_grace = 10.0
    d.confirm_window = 15.0
    d.recover_hold = 3.0
    d.resolved_hold = 5.0
    d.auto_reset = True
    d.is_test = False
    d.transitions = []
    d.log = Log()
    d.get_clock = lambda: d.clock
    d.get_logger = lambda: d.log
    for k, v in STATES.items():
        setattr(d, k, v)

    def set_state(new, why=''):
        if new == d.state:
            return
        d.transitions.append((new, why))
        d.state = new
        d.since = d._now()

    d._set_state = set_state
    d._say = lambda text: None
    d._publish = lambda: None
    d._maybe_call = lambda: None

    def start_alarm(test=False):
        d.transitions.append((ALARM, 'start_alarm'))
        d.state = ALARM
        d.since = d._now()

    d._start_alarm = start_alarm
    bind(d, extract_methods(
        ALERT, 'AlertManager',
        {'_on_fall', '_is_lying', '_is_standing', '_is_absent', '_now', '_age'},
        ns={'String': Msg}))
    return d


def make_alert_with_tick():
    """tick 单独抽（它依赖 _is_* 与 _set_state，两者都已绑好）。"""
    d = make_alert()
    bind(d, extract_methods(ALERT, 'AlertManager', {'_tick'}, ns={}))
    return d


class TestAlertSensorLoss(unittest.TestCase):

    def test_suspect_holds_when_view_blocked(self):
        """SUSPECTED 遇到 unknown（蹲下/遮挡）不能复位。"""
        d = make_alert_with_tick()
        d._on_fall(Msg('lying'))
        d._tick()
        self.assertEqual(d.state, SUSPECTED)

        d.clock.advance(0.5)
        d._on_fall(Msg('unknown'))          # 被挡住了，不是起身
        d.clock.advance(2.0)                # 已超过 lying_gap
        d._tick()
        self.assertEqual(d.state, SUSPECTED,
                         'unknown 被当成了「人已起身」，确认流程被抹掉')

    def test_suspect_resets_after_continuous_absent(self):
        """明确连续无人超过 absent_grace 才复位（人真的走了）。"""
        d = make_alert_with_tick()
        d._on_fall(Msg('lying'))
        d._tick()
        self.assertEqual(d.state, SUSPECTED)

        for _ in range(6):                  # 前 6 秒：还在宽限期内
            d.clock.advance(1.0)
            d._on_fall(Msg('absent'))
            d._tick()
        self.assertEqual(d.state, SUSPECTED)

        for _ in range(7):                  # 累计 13s > absent_grace(10s)
            d.clock.advance(1.0)
            d._on_fall(Msg('absent'))
            d._tick()
        self.assertEqual(d.state, NORMAL)

    def test_confirming_still_alarms_when_sensor_dies(self):
        """已经进入 CONFIRMING 后传感器全丢，必须 fail-safe 进 ALARM。"""
        d = make_alert_with_tick()
        d._on_fall(Msg('lying'))
        d._tick()
        self.assertEqual(d.state, SUSPECTED)

        # 检测器每帧都在发 lying；持续 5 秒后进 CONFIRMING
        for _ in range(6):
            d.clock.advance(1.0)
            d._on_fall(Msg('lying'))
            d._tick()
        self.assertEqual(d.state, CONFIRMING)

        d.clock.advance(16.0)               # 期间没有任何 /fall/state 消息
        d._tick()
        self.assertEqual(d.state, ALARM,
                         '传感器丢失把 CONFIRMING 静默复位了（应该 fail-safe）')


# ------------------------------------------------------------------ 充电确认

class TestChargingCheck(unittest.TestCase):

    def _make(self):
        d = type('DockManager', (), {})()
        d.verify_charging = True
        d.charge_check_timeout = 30.0
        d._dock_ratio_at = None
        d._dock_check_at = 0.0
        d._charging_checked = False
        d.log = Log()
        d.get_logger = lambda: d.log
        bind(d, extract_methods(DOCK, 'DockManager', {'_check_charging'}, ns={}))
        return d

    def test_flat_ratio_is_reported_once(self):
        d = self._make()
        d._check_charging(0.0, 0.40)        # 记基线
        d._check_charging(10.0, 0.40)       # 还没到超时
        self.assertEqual(d.log.text(), '')

        d._check_charging(31.0, 0.40)
        self.assertIn('没有上涨', d.log.text())

        d.log.lines.clear()                 # 只报一次，不能每拍刷屏
        d._check_charging(60.0, 0.40)
        self.assertEqual(d.log.text(), '')

    def test_rising_ratio_confirms_charging(self):
        d = self._make()
        d._check_charging(0.0, 0.400)
        d._check_charging(5.0, 0.410)
        self.assertIn('确认在充电', d.log.text())

    def test_verification_can_be_disabled(self):
        d = self._make()
        d.verify_charging = False
        d._check_charging(0.0, 0.40)
        d._check_charging(999.0, 0.40)
        self.assertEqual(d.log.text(), '')


# ------------------------------------------------------------------ 自动灯
#
# 「人走了灯不灭」是这一层最典型的失效：检测器崩掉之后没有人再来推翻最后那条
# present=True，节点自己也不会怀疑消息已经停了 —— 灯整夜亮着，用户第二天早上
# 才发现，而日志里一句异常都没有。

class FakeWall:
    """可控墙钟。抽出来的方法用的是注入的 `time`，所以能把秒数快进。"""

    def __init__(self, start=1000.0):
        self.t = float(start)

    def monotonic(self):
        return self.t


def make_light():
    d = type('LightAuto', (), {})()
    wall = FakeWall()
    d.on_level = 0.85
    d.idle_level = 0.0
    d.fade_delay = 5.0
    d.fade_time = 2.0
    d.present_timeout = 3.0
    d.enabled = True
    d.present = False
    d.last_seen = None
    d.msg_wall = 0.0
    d.last_t = None
    d._warned_stall = False
    d.level = d.idle_level
    d.target = d.idle_level
    d.override = None
    d.pub = Pub()
    d.log = Log()
    d.wall = wall
    d.get_logger = lambda: d.log
    bind(d, extract_methods(
        LIGHT, 'LightAuto', {'_tick', '_on_present', '_on_mode'},
        ns={'time': wall, 'math': math, 'Bool': Msg, 'Float32': Msg,
            'String': Msg}))
    return d


def run_light(d, secs, present=None, step=0.1):
    """快进 secs 秒；present 为 None 表示「检测器已死，一条消息都不发」。"""
    for _ in range(max(1, int(round(secs / step)))):
        d.wall.t += step
        if present is not None:
            d._on_present(Msg(present))
        d._tick()


class TestLightAutoSurvivesDetectorDeath(unittest.TestCase):

    def test_light_follows_person_and_fades(self):
        d = make_light()
        run_light(d, 6.0, present=True)
        self.assertAlmostEqual(d.level, 0.85, places=6)

        run_light(d, 4.0, present=False)        # 还在 fade_delay(5s) 内
        self.assertAlmostEqual(d.level, 0.85, places=6, msg='刚刚离开就灭灯，会闪')

        run_light(d, 9.0, present=False)        # 越过延迟 + 过渡
        self.assertAlmostEqual(d.level, 0.0, places=6)

    def test_dead_detector_fades_instead_of_staying_on(self):
        d = make_light()
        run_light(d, 6.0, present=True)
        self.assertAlmostEqual(d.level, 0.85, places=6)

        run_light(d, 20.0)                      # 检测器崩了：一条消息也没有
        self.assertAlmostEqual(d.level, 0.0, places=6,
                               msg='检测器死后灯一直亮着（present=True 永久粘住）')
        self.assertIn('没收到 /person/present', d.log.text('warn'))

    def test_quiet_boot_does_not_warn(self):
        """开机还没收到消息是正常启动顺序，不该报「检测器已退出」。"""
        d = make_light()
        run_light(d, 5.0)
        self.assertAlmostEqual(d.level, 0.0, places=6)
        self.assertEqual(d.log.text('warn'), '')


if __name__ == '__main__':
    unittest.main(verbosity=2)