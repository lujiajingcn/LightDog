"""外呼通道回归（D2 自建 SIP）：AMI 协议、结果映射、非阻塞、ack 挂断。

用一个**真的 socket** 假 Asterisk AMI 服务器，不是 mock：

- 半包/粘包、事件交错（`CoreShowChannels` 的事件和 `OriginateResponse` 混在
  一起）、连接被对端关闭 —— 这些只有真 socket 才测得出来。
- 把 socket mock 掉，就等于把唯一值得测的那层测没了：真机上出错的一定是
  协议层，而不是「有没有调用过某个方法」。

覆盖的两类问题：

1. **结果语义**：接通 / 无人接听 / 忙 / 拥塞 / 被拒 / 不回事件，六种要能分开。
   「没人接」被记成「外呼失败」，会让「链路坏了」和「家里没人」在日志里
   长得一模一样。
2. **非阻塞与挂断**：`_tick` 里同步拨号会卡住 5Hz 状态机；`ack` 只停「下一个」
   不停「这一通」会让手机响满 45 秒。这两条各自都有回归。
"""

import os
import socket
import sys
import threading
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from harness import (  # noqa: E402
    Msg, PKG, bind, extract_methods, source_of,
)

sys.path.insert(0, PKG)

import lightdog_call as LC  # noqa: E402

ALERT = source_of('lightdog_alert_manager.py')


# ---------------------------------------------------------------- 假 AMI

def _parse(raw):
    out = {}
    for line in raw.replace('\r\n', '\n').split('\n'):
        if ':' in line:
            k, v = line.split(':', 1)
            out[k.strip().lower()] = v.strip()
    return out


class FakeAmi(threading.Thread):
    """够用的假 Asterisk。

    参数直接对应「这一通电话会发生什么」：
      reason=0  -> 接通（Response: Success）
      reason=3  -> 无人接听（Response: Failure）
      reason=5  -> 忙
      respond=False -> 永远不回 OriginateResponse（验证本地超时）
    """

    def __init__(self, secret='s3cret', reason=0, respond=True,
                 event_delay=0.0, reject=False):
        super().__init__(daemon=True)
        self.secret = secret
        self.reason = reason
        self.respond = respond
        self.event_delay = event_delay
        self.reject = reject
        self.actions = []
        self.channels = {}          # 频道名 -> 是否还活着
        self._pending = {}          # 频道名 -> ActionID（用来回 Reason=1）
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._conns = []
        self.sock = socket.socket()
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('127.0.0.1', 0))
        self.sock.listen(8)
        self.sock.settimeout(0.2)
        self.port = self.sock.getsockname()[1]

    # -- 生命周期 -----------------------------------------------------

    def stop(self):
        self._stop.set()
        try:
            self.sock.close()
        except OSError:
            pass
        with self._lock:
            conns = list(self._conns)
        for c in conns:
            try:
                c.close()
            except OSError:
                pass

    def run(self):
        while not self._stop.is_set():
            try:
                conn, _ = self.sock.accept()
            except (socket.timeout, OSError):
                continue
            with self._lock:
                self._conns.append(conn)
            threading.Thread(target=self._serve, args=(conn,), daemon=True).start()

    def names(self):
        with self._lock:
            return [a.get('action') for a in self.actions]

    def _send(self, conn, text):
        # 只有 banner 是「单行、不带空行结尾」的消息；其余都是完整消息。
        # 这里多补一个 \r\n 会把 banner 变成空消息，客户端登录会读到 {}。
        if not text.endswith('\r\n'):
            text = text + '\r\n'
        try:
            conn.sendall(text.encode())
        except OSError:
            pass

    def _serve(self, conn):
        self._send(conn, 'Asterisk Call Manager/5.0.3\r\n')
        buf = b''
        while not self._stop.is_set():
            idx = buf.find(b'\r\n\r\n')
            if idx < 0:
                conn.settimeout(0.2)
                try:
                    chunk = conn.recv(4096)
                except socket.timeout:
                    continue
                except OSError:
                    return
                if not chunk:
                    return
                buf += chunk
                continue
            raw, buf = buf[:idx], buf[idx + 4:]
            self._dispatch(conn, _parse(raw.decode('utf-8', 'replace')))

    def _dispatch(self, conn, f):
        # 名字不能叫 _handle：threading.Thread 在 3.13 起有同名内部属性，
        # 覆盖它会让 _serve 线程在第一次收到 Action 时抛
        # `'_ThreadHandle' object is not callable`，而客户端只会表现成「AMI 超时」。
        action = f.get('action', '').lower()
        with self._lock:
            self.actions.append(f)
        if action == 'login':
            if f.get('secret') == self.secret:
                self._send(conn, 'Response: Success\r\n'
                                 'Message: Authentication accepted\r\n\r\n')
            else:
                self._send(conn, 'Response: Error\r\n'
                                 'Message: Authentication failed\r\n\r\n')
        elif action == 'originate':
            self._on_originate(conn, f)
        elif action == 'coreshowchannels':
            self._send(conn, 'Response: Success\r\nEventList: start\r\n'
                             'Message: Channels will follow\r\n\r\n')
            with self._lock:
                alive = [c for c, up in self.channels.items() if up]
            for ch in alive:
                # 中间混一条无关事件，验证客户端会跳过不相关消息
                self._send(conn, 'Event: Newexten\r\nChannel: %s\r\n'
                                 'Application: Wait\r\n\r\n' % ch)
                self._send(conn, 'Event: CoreShowChannel\r\nChannel: %s\r\n'
                                 'State: Ring\r\n\r\n' % ch)
            self._send(conn, 'Event: CoreShowChannelsComplete\r\n'
                             'EventList: Complete\r\nListItems: %d\r\n\r\n' % len(alive))
        elif action == 'hangup':
            ch = f.get('channel', '')
            with self._lock:
                self.channels[ch] = False
                pending = self._pending.pop(ch, None)
            self._send(conn, 'Response: Success\r\nMessage: Channel Hungup\r\n\r\n')
            if pending:
                # 真的 Asterisk 在响铃中被挂断时，会给**发起 originate 的那条
                # 连接**发 Reason=1（AST_CONTROL_HANGUP）的 OriginateResponse。
                # 挂在别处（比如 ack 用的第二条 AMI 连接）等于没发：
                # 工作线程永远等不到它，「ack 后多久停止响铃」就测不出来。
                action_id, orig_conn = pending
                self._event(orig_conn, action_id, 'Failure', 1, ch)

    def _on_originate(self, conn, f):
        if self.reject:
            self._send(conn, 'Response: Error\r\n'
                             'Message: No such context\r\n\r\n')
            return
        self._send(conn, 'Response: Success\r\n'
                         'Message: Originate successfully queued\r\n\r\n')
        base = f.get('channel', '')
        channel = f'{base}-0000000{len(self.channels) + 1}'
        with self._lock:
            self.channels[channel] = True
            self._pending[channel] = (f.get('actionid', ''), conn)
        if not self.respond:
            return
        threading.Thread(target=self._later_answer,
                         args=(conn, f.get('actionid', ''), channel),
                         daemon=True).start()

    def _later_answer(self, conn, action_id, channel):
        if self.event_delay:
            time.sleep(self.event_delay)
        self._event(conn, action_id,
                    'Success' if self.reason in (0, 4) else 'Failure',
                    self.reason, channel)

    def _event(self, conn, action_id, resp, reason, channel):
        self._send(conn, ('Event: OriginateResponse\r\nActionID: %s\r\n'
                          'Response: %s\r\nChannel: %s\r\nUniqueid: 1.2\r\n'
                          'Reason: %d\r\n\r\n'
                          % (action_id, resp, channel, reason)))


# ---------------------------------------------------------------- 工具

def cfg(fake, **kw):
    base = dict(host='127.0.0.1', port=fake.port, username='lightdog',
                secret=fake.secret, ring_timeout=3.0, max_call=0.6,
                connect_timeout=2.0, event_margin=0.5)
    base.update(kw)
    return LC.SipConfig(**base)


def wait_result(dialer, timeout=4.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        got = dialer.poll()
        if got:
            return got[0]
        time.sleep(0.02)
    raise AssertionError(f'{timeout}s 内没拿到呼叫结果')


class AmiTest(unittest.TestCase):

    def setUp(self):
        self.fake = None
        self.dialer = None

    def tearDown(self):
        if self.dialer is not None:
            self.dialer.close()
        if self.fake is not None:
            self.fake.stop()

    def fake_up(self, **kw):
        self.fake = FakeAmi(**kw)
        self.fake.start()
        return self.fake

    # -- 协议层 -------------------------------------------------------

    def test_login_ok(self):
        f = self.fake_up()
        c = LC.AmiClient('127.0.0.1', f.port, 'lightdog', f.secret).connect()
        try:
            self.assertIn('login', f.names())
            self.assertEqual(f.actions[0].get('events'), 'call,originate')
        finally:
            c.close()

    def test_login_bad_secret(self):
        f = self.fake_up()
        with self.assertRaises(LC.AmiAuthError) as cm:
            LC.AmiClient('127.0.0.1', f.port, 'lightdog', 'wrong').connect()
        self.assertIn('Authentication', str(cm.exception))

    def test_connection_refused(self):
        # 一个刚关掉的端口：连不上必须立刻报错，而不是静默返回
        s = socket.socket()
        s.bind(('127.0.0.1', 0))
        port = s.getsockname()[1]
        s.close()
        with self.assertRaises(LC.AmiError):
            LC.AmiClient('127.0.0.1', port, 'x', 'y', timeout=1.0).connect()

    # -- 结果映射 -----------------------------------------------------

    def _one_call(self, **fake_kw):
        f = self.fake_up(**fake_kw)
        self.dialer = LC.SipDialer(cfg(f))
        self.assertTrue(self.dialer.dial('1001', '张阿姨', tag='t1'))
        return wait_result(self.dialer)

    def test_answered(self):
        r = self._one_call(reason=0)
        self.assertEqual(r.outcome, LC.ANSWERED)
        self.assertTrue(r.placed)
        self.assertTrue(r.channel.startswith('PJSIP/1001-'))
        self.assertIn('已接通', r.text())

    def test_no_answer_is_not_failure(self):
        # 这条是本轮的核心：没人接是正常业务，不是链路故障。
        r = self._one_call(reason=3)
        self.assertEqual(r.outcome, LC.NO_ANSWER)
        self.assertEqual(r.reason, 3)
        self.assertTrue(r.placed, '响过了铃就算打出去了')
        self.assertFalse(r.answered)

    def test_busy(self):
        self.assertEqual(self._one_call(reason=5).outcome, LC.BUSY)

    def test_congestion(self):
        self.assertEqual(self._one_call(reason=8).outcome, LC.CONGESTION)

    def test_originate_rejected(self):
        r = self._one_call(reject=True)
        self.assertEqual(r.outcome, LC.FAILED)
        self.assertFalse(r.placed, 'AMI 拒绝了就不能算「已打进电话」')
        self.assertIn('No such context', r.detail)

    def test_no_event_times_out(self):
        f = self.fake_up(respond=False)
        self.dialer = LC.SipDialer(cfg(f, ring_timeout=0.3, event_margin=0.3))
        self.dialer.dial('1001', '张阿姨', tag='t1')
        r = wait_result(self.dialer)
        self.assertEqual(r.outcome, LC.TIMEOUT)
        self.assertFalse(r.placed)

    # -- 并发与挂断 ---------------------------------------------------

    def test_second_dial_rejected_while_busy(self):
        f = self.fake_up(reason=0)
        self.dialer = LC.SipDialer(cfg(f, max_call=5.0))
        self.assertTrue(self.dialer.dial('1001', 'A', tag='t1'))
        time.sleep(0.2)
        self.assertTrue(self.dialer.busy)
        self.assertFalse(self.dialer.dial('1002', 'B', tag='t2'),
                         '同时拨两个联系人会让「谁接通了」变得暧昧')
        self.dialer.hangup('ack')
        wait_result(self.dialer)

    def test_ack_hangs_up_ringing_call(self):
        # 响铃超时给 30 秒；如果 ack 不能真挂断，这个用例会跑到超时。
        f = self.fake_up(respond=False)
        self.dialer = LC.SipDialer(cfg(f, ring_timeout=30.0, event_margin=30.0))
        self.dialer.dial('1001', '张阿姨', tag='t1')
        time.sleep(0.3)                         # 让频道先建起来
        t0 = time.monotonic()
        self.assertTrue(self.dialer.hangup('ack'))
        r = wait_result(self.dialer, timeout=6.0)
        self.assertEqual(r.outcome, LC.CANCELLED)
        self.assertEqual(r.hung_up_by, 'ack')
        self.assertLess(time.monotonic() - t0, 4.0,
                        'ack 后必须很快停止响铃，不能等 ring_timeout')
        self.assertIn('hangup', f.names())

    def test_ack_hangs_up_answered_call(self):
        f = self.fake_up(reason=0)
        self.dialer = LC.SipDialer(cfg(f, max_call=30.0))
        self.dialer.dial('1001', '张阿姨', tag='t1')
        time.sleep(0.4)                         # 等 OriginateResponse
        self.assertTrue(self.dialer.busy)
        self.dialer.hangup('ack')
        r = wait_result(self.dialer, timeout=6.0)
        self.assertEqual(r.outcome, LC.ANSWERED)
        self.assertEqual(r.hung_up_by, 'ack')
        self.assertGreater(r.end_wall, 0.0)
        self.assertIn('已确认后挂断', r.text())

    def test_max_call_hangs_up(self):
        f = self.fake_up(reason=0)
        self.dialer = LC.SipDialer(cfg(f, max_call=0.4))
        self.dialer.dial('1001', 'A', tag='t1')
        r = wait_result(self.dialer, timeout=6.0)
        self.assertEqual(r.outcome, LC.ANSWERED)
        self.assertEqual(r.hung_up_by, 'timeout')

    # -- 参数校验 -----------------------------------------------------

    def test_outcome_text_covers_all(self):
        for name in dir(LC):
            if name.isupper() and isinstance(getattr(LC, name), str):
                v = getattr(LC, name)
                if v in LC.OUTCOME_TEXT or name == 'OUTCOME_TEXT':
                    continue
                self.assertIn(v, LC.OUTCOME_TEXT,
                              f'常量 {name}={v} 没有对应的中文说明')

    def test_channel_template(self):
        c = LC.SipConfig(channel_template='PJSIP/trunk/{number}')
        self.assertEqual(c.channel_of('13900000000'), 'PJSIP/trunk/13900000000')


# ---------------------------------------------------------------- 节点侧

class FakeDialer:
    """假的拨号通道，只记调用、按需吐出结果。"""

    def __init__(self, results=None):
        self.enabled = True
        self.busy = False
        self.calls = []
        self.hungups = []
        self._results = list(results or [])

    def dial(self, number, name, tag='', attempt=1):
        self.calls.append((number, name, tag, attempt))
        return True

    def poll(self):
        out, self._results = self._results, []
        return out

    def hangup(self, why='ack'):
        self.hungups.append(why)
        return True

    def describe(self):
        return 'fake'


def fake_self(dialer=None):
    """把 _maybe_call/_drain_calls 需要的状态摆出来（显式，见 harness 说明）。"""

    class S:
        pass

    s = S()
    s.state = 'ALARM'
    s.ALARM = 'ALARM'
    s.ACKED = 'ACKED'
    s.CONFIRMING = 'CONFIRMING'
    s.is_test = False
    s.last_call = None
    s.call_interval = 20.0
    s.rounds = 0
    s.max_rounds = 3
    s.call_idx = 0
    s.call_attempts = []
    s.alarm_id = 'alm_test'
    s.contacts_path = '/nonexistent/contacts.json'
    s._dialer = dialer
    s.log = []
    s.said = []
    s.published = 0
    s.clock = [100.0]
    s.contacts = [{'name': '张阿姨', 'phone': '1001'},
                  {'name': '李叔', 'phone': '1002'}]

    s._now = lambda: s.clock[0]
    s._load_contacts = lambda: s.contacts

    class L:
        def info(self, m, **k):
            s.log.append(('info', str(m)))

        def warn(self, m, **k):
            s.log.append(('warn', str(m)))

        error = info

    s.get_logger = lambda: L()
    s._say = lambda t: s.said.append(t)
    s._publish = lambda: setattr(s, 'published', s.published + 1)

    # `_maybe_call` 里用了 time.time()（呼叫记录记的是墙钟，不是 ROS 时钟），
    # 抽出来的函数体没有模块全局，必须显式传进去。
    meth = extract_methods(ALERT, 'AlertManager',
                           ['_maybe_call', '_drain_calls', '_mark_call',
                            '_hangup_call'], ns={'time': time})
    bind(s, meth)
    return s


class NodeTest(unittest.TestCase):

    def test_dial_is_nonblocking(self):
        """_maybe_call 不能等结果 —— 它跑在 5Hz 的 _tick 里。"""
        d = FakeDialer()
        s = fake_self(d)
        t0 = time.monotonic()
        s._maybe_call()
        self.assertLess(time.monotonic() - t0, 0.1)
        self.assertEqual(len(s.call_attempts), 1)
        a = s.call_attempts[0]
        self.assertEqual(a['outcome'], 'dialing')
        self.assertFalse(a['dialed'])
        self.assertEqual(d.calls[0][:2], ('1001', '张阿姨'))

    def test_result_backfills_attempt(self):
        d = FakeDialer()
        s = fake_self(d)
        s._maybe_call()
        tag = s.call_attempts[0]['tag']
        r = LC.DialResult('1001', '张阿姨', tag=tag, outcome=LC.NO_ANSWER,
                          reason=3, end_wall=time.time())
        d._results = [r]
        s._drain_calls()
        a = s.call_attempts[0]
        self.assertEqual(a['outcome'], LC.NO_ANSWER)
        self.assertEqual(a['outcome_text'], '无人接听')
        self.assertTrue(a['dialed'])
        self.assertGreater(s.published, 0)

    def test_unowned_result_warns(self):
        d = FakeDialer()
        s = fake_self(d)
        s._maybe_call()
        d._results = [LC.DialResult('1001', 'X', tag='stale#9',
                                    outcome=LC.BUSY)]
        s._drain_calls()
        self.assertTrue(any('无主' in m for _, m in s.log))

    def test_test_alarm_never_dials(self):
        d = FakeDialer()
        s = fake_self(d)
        s.is_test = True
        s._maybe_call()
        self.assertEqual(d.calls, [])
        self.assertIn('测试告警', '\n'.join(m for _, m in s.log))

    def test_busy_dialer_skips_round(self):
        d = FakeDialer()
        d.busy = True
        s = fake_self(d)
        s._maybe_call()
        self.assertEqual(d.calls, [])
        self.assertEqual(s.call_attempts, [])

    def test_interval_respected(self):
        d = FakeDialer()
        s = fake_self(d)
        s._maybe_call()
        s.clock[0] += 5.0            # < call_interval
        s._maybe_call()
        self.assertEqual(len(d.calls), 1)
        s.clock[0] += 20.0
        s._maybe_call()
        self.assertEqual(len(d.calls), 2)

    def test_rounds_stop(self):
        d = FakeDialer()
        s = fake_self(d)
        s.max_rounds = 1
        for i in range(6):
            s._maybe_call()
            s.clock[0] += 21.0
        self.assertEqual(len(d.calls), 2, '2 个联系人 × 1 轮')
        self.assertEqual(s.rounds, 1)

    def test_hangup_passthrough(self):
        d = FakeDialer()
        s = fake_self(d)
        self.assertTrue(s._hangup_call('ack'))
        self.assertEqual(d.hungups, ['ack'])
        s._dialer = None
        self.assertFalse(s._hangup_call('ack'), '没有外呼通道时安静返回')

    def test_ack_command_hangs_up(self):
        """ack 必须挂断正在响的那一通，而不只是「不再拨下一个」。"""
        d = FakeDialer()
        s = fake_self(d)
        s.since = 0.0
        s._set_state = lambda new, why='': setattr(s, 'state', new)
        s._start_alarm = lambda test=False: None
        meth = extract_methods(ALERT, 'AlertManager', ['_on_command'],
                               ns={'String': Msg})
        bind(s, meth)
        s._on_command(Msg('ack'))
        self.assertEqual(s.state, 'ACKED')
        self.assertEqual(d.hungups, ['ack'])
        self.assertIn('已收到确认，不再重复呼叫', s.said)

    def test_resolve_call_mode(self):
        f = extract_methods(ALERT, 'AlertManager', ['_resolve_call_mode'])
        resolve = f['_resolve_call_mode']
        self.assertEqual(resolve('auto', 'secret', False), 'sip')
        self.assertEqual(resolve('auto', '', True), 'command')
        self.assertEqual(resolve('auto', '', False), 'log')
        self.assertEqual(resolve('', 'secret', False), 'sip')
        self.assertEqual(resolve('off', 'secret', True), 'log')
        self.assertEqual(resolve('SIP', '', False), 'sip')
        self.assertEqual(resolve('nonsense', 'secret', True), 'log',
                         '非法取值一律退回仅记录')


# ---------------------------------------------------------------- 命令通道

class CommandTest(unittest.TestCase):

    def test_no_shell_and_placed(self):
        import subprocess
        rec = {}

        def fake_run(argv, **kw):
            rec['argv'] = argv
            rec['kw'] = kw

            class R:
                returncode = 0
                stderr = b''
            return R()

        orig = subprocess.run
        subprocess.run = fake_run
        try:
            d = LC.CommandDialer('dialer --to {phone} --name {name}')
            d.dial('1001', '张阿姨', attempt=2)
            r = None
            for _ in range(100):
                got = d.poll()
                if got:
                    r = got[0]
                    break
                time.sleep(0.02)
        finally:
            subprocess.run = orig
        self.assertIsNotNone(r)
        self.assertEqual(r.outcome, LC.PLACED)
        self.assertIsInstance(rec['argv'], list, '必须是 argv 列表，不能是字符串')
        self.assertEqual(rec['argv'], ['dialer', '--to', '1001', '--name', '张阿姨'])
        self.assertNotIn('shell', rec['kw'])
        self.assertFalse(r.answered, '命令通道拿不到是否接听，不能冒充「已接通」')

    def test_failure_carries_rc(self):
        import subprocess

        def fake_run(argv, **kw):
            class R:
                returncode = 3
                stderr = b'boom'
            return R()

        orig = subprocess.run
        subprocess.run = fake_run
        try:
            d = LC.CommandDialer('x {phone}')
            d.dial('1001', 'A')
            r = None
            for _ in range(100):
                got = d.poll()
                if got:
                    r = got[0]
                    break
                time.sleep(0.02)
        finally:
            subprocess.run = orig
        self.assertEqual(r.outcome, LC.FAILED)
        self.assertIn('rc=3', r.detail)


if __name__ == '__main__':
    unittest.main(verbosity=2)
