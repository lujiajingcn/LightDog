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

import json
import math
import os
import queue
import socket
import sys
import threading
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from harness import (  # noqa: E402
    Clock, Log, Msg, Pub, bind, extract_const, extract_funcs, extract_methods,
    source_of,
)

REC = source_of('lightdog_recorder.py')
ALERT = source_of('lightdog_alert_manager.py')
DOCK = source_of('lightdog_dock_manager.py')
LIGHT = source_of('lightdog_light_auto.py')

_safe = extract_funcs(REC, {'safe_token', 'clamp_sec', 'even'})
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


# ---------------------------------------------------------------- 呼叫轮次

def make_caller(contacts, interval=0.0):
    """只装 _maybe_call 需要的那几个属性，不碰 rclpy。"""
    import shlex
    import time as _time

    d = type('AlertManager', (), {})()
    d.state = ALARM
    d.ACKED = 'ACKED'
    d.is_test = False
    d.last_call = None
    d.call_interval = interval
    d.max_rounds = 3
    d.rounds = 0
    d.call_idx = 0
    d.call_attempts = []
    d.call_command = ''
    d.clock = Clock()
    d.log = Log()
    d.get_clock = lambda: d.clock
    d.get_logger = lambda: d.log
    d._load_contacts = lambda: contacts
    d._say = lambda text: None
    d._publish = lambda: None
    bind(d, extract_methods(ALERT, 'AlertManager', {'_maybe_call', '_now'},
                            ns={'time': _time, 'shlex': shlex, 'subprocess': None}))
    return d


class TestCallRounds(unittest.TestCase):
    """告警呼叫的轮次编排。

    2 个联系人、max_rounds=3 时，标签应当是
    1,1,2,2,3,3（每个联系人都打满一轮），而不是 1,2,2,3 ——
    后者看告警记录会以为第一轮漏掉了一个人。
    """

    CONTACTS = [{'name': '张阿姨', 'phone': '13800000001'},
                {'name': '社区值班', 'phone': '13800000002'}]

    def test_round_labels_cover_every_contact(self):
        d = make_caller(self.CONTACTS)
        for _ in range(4):
            d._maybe_call()
        labels = [a['round'] for a in d.call_attempts]
        self.assertEqual(labels, [1, 1, 2, 2],
                         '轮次标签与「第几轮」不一致，告警记录会误导')

    def test_stops_after_max_rounds(self):
        d = make_caller(self.CONTACTS)
        for _ in range(20):
            d._maybe_call()
        n = len(self.CONTACTS) * d.max_rounds
        self.assertEqual(len(d.call_attempts), n,
                         '呼叫次数超过了 max_rounds × 联系人数')
        self.assertEqual(d.rounds, d.max_rounds)

    def test_last_round_is_not_dropped(self):
        """轮次判据若写成 >=，最后一轮会被整体跳过。"""
        d = make_caller(self.CONTACTS)
        for _ in range(20):
            d._maybe_call()
        self.assertEqual(d.call_attempts[-1]['name'], '社区值班')


# ---------------------------------------------------------------- 告警历史

SERVER = source_of('lightdog_server.py')


def make_server(tmpdir, live_alert):
    """只装 alarms() 需要的东西：一个 jsonl 文件 + 一条实时告警。"""
    d = type('LightDogServer', (), {})()
    d.data_dir = tmpdir
    d.alert = live_alert
    d.event_of_alarm = lambda aid: None
    d.newest_event_since = lambda t: None
    bind(d, extract_methods(SERVER, 'LightDogServer', {'alarms'},
                            ns={'os': os, 'json': __import__('json')}))
    return d


class TestAlarmHistoryConsistency(unittest.TestCase):
    """历史列表与实时状态必须说同一句话。

    jsonl 只在 (alarm_id, state) 变化时落一行，所以告警进行中时最后一行
    是「刚进 ALARM」的快照，呼叫记录还没进去 —— 列表显示「呼叫 0 次」而
    /status 已经有 3 次，手机端两个页面对不上。
    """

    def setUp(self):
        import tempfile
        self.tmp = tempfile.mkdtemp()
        self.path = os.path.join(self.tmp, 'alarms.jsonl')

    def _write(self, rows):
        import json
        with open(self.path, 'w', encoding='utf-8') as f:
            for r in rows:
                f.write(json.dumps(r, ensure_ascii=False) + '\n')

    def test_live_calls_show_up_in_history(self):
        self._write([{'alarm_id': 'alm_1', 'state': 'ALARM',
                      'alarm_wall': 100.0, 'call_attempts': [], 'rounds': 0}])
        live = {'alarm_id': 'alm_1', 'state': 'ALARM',
                'call_attempts': [{'name': '张阿姨', 'round': 1}], 'rounds': 1}
        got = make_server(self.tmp, live).alarms()
        self.assertEqual(len(got), 1)
        self.assertEqual(len(got[0]['call_attempts']), 1,
                         '进行中的告警在历史里显示 0 次呼叫')
        self.assertEqual(got[0]['rounds'], 1)

    def test_finished_alarm_is_not_overwritten(self):
        """已结束（NORMAL）的告警不能被上一条实时消息篡改。"""
        self._write([{'alarm_id': 'alm_1', 'state': 'NORMAL',
                      'alarm_wall': 100.0,
                      'call_attempts': [{'name': '甲'}, {'name': '乙'}],
                      'rounds': 1}])
        live = {'alarm_id': 'alm_1', 'state': 'NORMAL', 'call_attempts': [],
                'rounds': 0}
        got = make_server(self.tmp, live).alarms()
        self.assertEqual(len(got[0]['call_attempts']), 2,
                         '已结束的告警被实时消息清空了呼叫记录')


# ---------------------------------------------------------------- HTTP 层
#
# 这一层的坑都是「协议上合法、但没人写」的角落：
#   * HEAD 被 do_HEAD 直接转调 do_GET，于是**所有** GET 分支都会跑一遍，
#     包括两个 while True 的推流分支 —— curl -I 会挂到超时，而且真的写帧；
#   * 目录也能通过 getsize()，随后 open() 抛 IsADirectoryError → 500 + traceback；
#   * 相机不出帧时推流循环一个字节都不写，客户端断开永远触发不了 BrokenPipe。

MIME = extract_const(SERVER, 'MIME')


class _WFile:
    def __init__(self):
        self.data = b''

    def write(self, b):
        self.data += b

    def flush(self):
        pass


class _Hub:
    """数调用次数：HEAD 不该进推流循环，所以次数必须是 0。"""

    def __init__(self):
        self.calls = 0

    def wait_newer(self, *a, **k):
        self.calls += 1
        if self.calls > 1:
            raise AssertionError('HEAD 请求进了推流循环')
        return None, 0, 0


def make_handler(command='GET', path='/x'):
    h = type('Handler', (), {})()
    h.command = command
    h.path = path
    h.headers = {}
    h.close_connection = False
    h.wfile = _WFile()
    h.sent = []
    h.hdrs = {}
    h.send_response = lambda code: h.sent.append(code)
    h.send_header = lambda k, v: h.hdrs.__setitem__(k, v)
    h.end_headers = lambda: None
    h.hub = _Hub()
    h.node = type('N', (), {'stream_fps': 10.0, 'hub': h.hub})()
    bind(h, extract_methods(SERVER, 'Handler',
                            {'_send', '_json', '_err', '_file', '_stream',
                             '_peer_closed'},
                            ns={'json': json, 'os': os, 'MIME': MIME,
                                'time': __import__('time'), 'select': __import__('select'),
                                'socket': socket}))
    return h


class TestHttpEdgeCases(unittest.TestCase):

    def test_directory_is_404_not_500(self):
        """`/api/v1/snapshots/..` 不能打到 500：目录 getsize 是成功的。"""
        import tempfile
        d = tempfile.mkdtemp()
        h = make_handler()
        h._file(d)
        self.assertEqual(h.sent[-1], 404,
                         '目录被当成文件打开了（会抛 IsADirectoryError → 500）')

    def test_head_on_stream_returns_immediately(self):
        h = make_handler(command='HEAD')
        h._stream()
        self.assertEqual(h.sent[-1], 200)
        self.assertEqual(h.hub.calls, 0,
                         'HEAD 进了 MJPEG 推流循环：curl -I 会挂到超时')
        self.assertEqual(h.wfile.data, b'',
                         'HEAD 响应里写了 body（协议错误）')

    def test_head_on_file_sends_headers_only(self):
        import tempfile
        d = tempfile.mkdtemp()
        p = os.path.join(d, 'a.jpg')
        with open(p, 'wb') as f:
            f.write(b'\xff\xd8' + b'x' * 100)
        h = make_handler(command='HEAD')
        h._file(p)
        self.assertEqual(h.sent[-1], 200)
        self.assertEqual(h.wfile.data, b'')
        self.assertEqual(h.hdrs.get('Content-Length'), '102')

    def test_peer_closed_detects_hangup(self):
        """对端「静默」和「挂了」必须区分开 —— 静默不能杀，挂了不能留。

        注意带上「有字节待读」这一档：那种情况是请求还没读完，不是断开。
        （真正的请求字节在进 handler 之前就被 http.server 读走了，所以
        响应阶段一般是「无数据 + EOF」，也就是下面 dead 那一对。）
        """
        h = make_handler()
        alive = socket.socketpair()
        dead = socket.socketpair()
        try:
            h.connection = alive[0]
            self.assertFalse(h._peer_closed(), '对端还活着却被判成断开')
            alive[1].sendall(b'x')
            self.assertFalse(h._peer_closed(),
                             '有数据待读（请求还没读完）不能当成断开')
            h.connection = dead[0]
            dead[1].close()
            self.assertTrue(h._peer_closed(),
                            '对端已关闭却探测不到 → 相机静默时线程会永久泄漏')
        finally:
            for s in (alive[0], alive[1], dead[0]):
                try:
                    s.close()
                except OSError:
                    pass


# ---------------------------------------------------------------- 录像写盘
#
# 这一段在仿真里几乎不会出问题（相机 1.5Hz、磁盘永远是空的），真机上才炸：
#   * SD 卡写满 → VideoWriter.write 抛异常 → 句柄不释放 + 半个 .part.mp4 留在
#     clips/ 里，而 `_gc` 只按索引清理，这个文件永远不会被删；
#   * 编码跟不上采集 → 队列无限长 → 内存按段上涨（一段约 12MB），最后 OOM。


class _NoBlockQueue(queue.Queue):
    """满队列上**阻塞** put 一律判失败。

    旧实现用的是 `put()`（无界队列时看不出问题）。测试里把队列设成有界，
    阻塞 put 会把测试进程一起挂死 —— 那种失败方式比断言失败更难查。
    这里让它在满队列 + block=True 时直接抛断言错误；`put_nowait`（block=False）
    仍然正常抛 queue.Full，新实现的语义不受影响。
    """

    def put(self, item, block=True, timeout=None):
        if block and self.full():
            raise AssertionError(
                '_flush_segment 用了阻塞 put：队列满会把 ROS 回调线程堵死')
        return super().put(item, block, timeout)


class _FakeWriter:
    def __init__(self, fail_after=None):
        self.fail_after = fail_after
        self.n = 0
        self.released = False

    def write(self, frame):
        self.n += 1
        if self.fail_after is not None and self.n > self.fail_after:
            raise IOError('No space left on device')

    def release(self):
        self.released = True


class _Frame:
    def __init__(self, h=4, w=4):
        self.shape = (h, w, 3)


def make_recorder(tmpdir, writer):
    r = type('Recorder', (), {})()
    r.clips_dir = tmpdir
    r.codec = 'avc1'
    r.session = 1
    r.seq = 0
    r.cur = []
    r.cur_t0 = None
    r.cur_t1 = None
    r._lock = threading.Lock()
    r._flush_q = _NoBlockQueue(maxsize=2)
    r._dropped_segments = 0
    r._decode = lambda buf: _Frame()
    r._fit = lambda frame, w, h: frame
    r._open_writer = lambda path, fps, size: (writer, 'avc1')
    r._append_index = lambda rec: None
    r.get_logger = lambda: Log()
    bind(r, extract_methods(REC, 'Recorder', {'_flush_segment', '_write_segment'},
                            ns={'os': os, 'queue': queue, 'time': time,
                                'json': json, 'even': _safe['even']}))
    return r


class TestRecorderWritePressure(unittest.TestCase):

    def test_write_failure_releases_writer_and_cleans_part(self):
        import tempfile
        d = tempfile.mkdtemp()
        w = _FakeWriter(fail_after=1)
        r = make_recorder(d, w)
        job = {'seq': 0, 'frames': [(0.0, b'a'), (0.1, b'b')],
               't0': 0.0, 't1': 0.1, 'wall': 0.0, 'codec': 'avc1'}
        with self.assertRaises(IOError):
            r._write_segment(job)
        self.assertTrue(w.released, '写失败时没 release → 文件句柄泄漏')
        self.assertEqual(os.listdir(d), [],
                         'clips/ 里留下了 .part.mp4（_gc 只按索引清，永远删不掉）')

    def test_slow_disk_drops_oldest_segment_instead_of_growing(self):
        import tempfile
        r = make_recorder(tempfile.mkdtemp(), _FakeWriter())
        for i in range(3):
            r.cur = [(float(i), b'x')]
            r.cur_t0 = r.cur_t1 = float(i)
            r._flush_segment()
        self.assertEqual(r._flush_q.qsize(), 2,
                         '队列没有上限：写盘跟不上时内存会一直涨')
        self.assertEqual(r._dropped_segments, 1, '丢弃没记账，手机端看不到')
        seqs = [r._flush_q.get_nowait()['seq'] for _ in range(2)]
        self.assertEqual(seqs, [1, 2], '丢的不是最老的段（录像应该保最近的）')


# ---------------------------------------------------------------- 告警命令

class TestAlertCommands(unittest.TestCase):
    """手机端 / 语音发来的三个命令。

    `test` 是个危险命令：它会把 alarm_id、呼叫记录全部重置，并把 is_test 置真，
    而 `_maybe_call` 见到 is_test 一律直接返回 —— 在真实跌倒告警**进行中**
    误发一次「模拟告警」，等于把正在打的求救电话全部静音。
    """

    def make(self, state, is_test=False):
        d = type('AlertManager', (), {})()
        d.state = state
        d.NORMAL, d.SUSPECTED, d.CONFIRMING = 'NORMAL', 'SUSPECTED', 'CONFIRMING'
        d.ALARM, d.ACKED, d.RESOLVED = 'ALARM', 'ACKED', 'RESOLVED'
        d.is_test = is_test
        d.alarm_id = 'alm_real'
        d.log = Log()
        d.get_logger = lambda: d.log
        d.said = []
        d.set_state = []
        d.started = []
        d._say = lambda text: d.said.append(text)
        d._set_state = lambda new, why='': d.set_state.append((new, why))
        d._reset = lambda why: None
        d._start_alarm = lambda test=False: d.started.append(test)
        bind(d, extract_methods(ALERT, 'AlertManager', {'_on_command'},
                                ns={'Msg': Msg, 'String': Msg}))
        return d

    def test_simulated_alarm_refused_while_real_alarm_active(self):
        for st in ('CONFIRMING', 'ALARM', 'ACKED'):
            d = self.make(st)
            d._on_command(Msg(data='test'))
            self.assertEqual(d.started, [],
                             f'{st} 状态下「模拟告警」把真实告警的呼叫静音了')
            self.assertEqual(d.alarm_id, 'alm_real', '真实告警 id 被测试覆盖')

    def test_simulated_alarm_allowed_when_idle(self):
        d = self.make('NORMAL')
        d._on_command(Msg(data='test'))
        self.assertEqual(d.started, [True], '空闲时「模拟告警」应当照常生效')

    def test_ack_voice_matches_behaviour(self):
        """ack 之后不再拨号，语音就不能说「正在联系家人」。"""
        d = self.make('ALARM')
        d._on_command(Msg(data='ack'))
        self.assertEqual(d.set_state[0][0], 'ACKED')
        self.assertTrue(d.said, 'ack 之后没有任何语音反馈')
        self.assertNotIn('正在联系', d.said[0],
                         '语音承诺「正在联系家人」，但 ACKED 之后一个电话都不打')


if __name__ == '__main__':
    unittest.main(verbosity=2)