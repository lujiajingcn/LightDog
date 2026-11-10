#!/usr/bin/env python3
"""2026-11-10 全工程审查的回归。

每条用例对应一个**先复现、再确认、才修**的缺陷。刻意按「缺陷会怎么骗过
你」来写，而不是按文件名罗列：假绿、静默失效、协议头注入、ISR 里喂狗，
这些都不会抛异常，只会安静地给出错误结论。

宿主机没有 ROS / Gazebo：能抽成纯逻辑的用 harness 从源码抽方法，剩下的用
源码不变量断言（改回去就会红）与真实临时目录行为测试。
"""

import contextlib
import csv
import importlib.util
import io
import json
import os
import re
import sys
import tempfile
import threading
import types
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from harness import PKG, bind, extract_funcs, extract_methods, source_of  # noqa: E402

REPO = os.path.dirname(PKG)
CALL = source_of('lightdog_call.py')
SERVER = source_of('lightdog_server.py')
P2 = source_of('lightdog_person_detector.py')
P5 = source_of('lightdog_fall_detector.py')
BRIDGE = source_of('lightdog_mcu_bridge.py')
RECORDER = source_of('lightdog_recorder.py')
VOICE = source_of('lightdog_voice.py')
ACCEPTANCE = os.path.join(REPO, 'deploy', 'acceptance.py')
SOAK_LOAD = os.path.join(REPO, 'sim', 'soak_load.py')
FIRMWARE = os.path.join(REPO, 'firmware')


def _load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


call = _load(CALL, 'ld_call_review')


class _Sock:
    def __init__(self):
        self.sent = b''

    def sendall(self, b):
        self.sent += b

    def settimeout(self, _t):
        pass

    def recv(self, _n):
        return b''

    def close(self):
        pass


def _ami(incoming=b''):
    a = call.AmiClient('127.0.0.1', 5038, 'u', 'p', timeout=0.2)
    a._sock = _Sock()
    a._buf = incoming
    return a


# ---------------------------------------------------------------- AMI


class TestAmiMessageTypes(unittest.TestCase):
    """同一条 TCP 上 Event 与 Response 交错，且 Event 自带 Response 头。"""

    EVENT_FIRST = (
        b'Event: OriginateResponse\r\n'
        b'Response: Failure\r\n'
        b'ActionID: ld-1\r\n'
        b'Reason: 3\r\n\r\n'
        b'Response: Success\r\n'
        b'ActionID: ld-1\r\n'
        b'Message: Originate successfully queued\r\n\r\n')

    def test_event_with_response_header_is_not_mistaken_for_action_response(self):
        a = _ami(self.EVENT_FIRST)
        resp = a.action({'action': 'originate', 'actionid': 'ld-1'})
        self.assertEqual(resp.get('response', '').lower(), 'success')
        self.assertNotIn('event', resp)
        # 被让过去的事件必须暂存，否则 wait_event 会白等到超时
        self.assertEqual(len(a._events), 1)
        ev = a.wait_event('originateresponse', 'ld-1', 0.2)
        self.assertEqual(ev.get('reason'), '3')

    def test_action_fields_reject_crlf_injection(self):
        a = _ami()
        with self.assertRaises(call.AmiError):
            a.action({'action': 'originate',
                      'channel': 'PJSIP/1001\r\nAction: Originate\r\nChannel: X'})
        self.assertEqual(a._sock.sent, b'', '注入的报文一个字节都不能发出去')


# ---------------------------------------------------------------- HTTP 服务


class _Err(Exception):
    """_err() 在 Handler 里其实是写响应后返回 None，测试里让它抛出来好断言。"""

    def __init__(self, code, msg):
        super().__init__(f'{code}: {msg}')
        self.code, self.msg = code, msg


class _Logger:
    def info(self, *_a, **_k):
        pass

    def warn(self, *_a, **_k):
        pass

    def error(self, *_a, **_k):
        pass


class TestServerHardening(unittest.TestCase):
    """手机端可写的内容不能弄坏后端，也不能污染 AMI 报文。"""

    @staticmethod
    def _handler(contacts_path=None, clips_dir=None, meta=None):
        saved = []
        node = types.SimpleNamespace(
            save_contacts=lambda cs: saved.append(list(cs)),
            get_logger=lambda: _Logger())
        obj = types.SimpleNamespace(
            node=node,
            _contacts_lock=threading.Lock(),
            contacts_path=contacts_path or '',
            clips_dir=clips_dir or '',
            _clip_index=lambda: meta or {},
            _err=lambda code, msg: (_ for _ in ()).throw(_Err(code, msg)))
        return obj, saved

    def test_phone_number_with_crlf_is_rejected(self):
        fns = extract_methods(SERVER, 'Handler', ['_put_contacts'],
                              ns={'json': json, 're': re})
        obj, saved = self._handler()
        raw = json.dumps({'contacts': [
            {'name': 'x', 'phone': '1001\r\nAction: Originate\r\nChannel: Y'}
        ]}).encode()
        with self.assertRaises(_Err) as cm:
            bind(obj, fns)._put_contacts(raw)
        self.assertEqual(cm.exception.code, 400)
        self.assertEqual(saved, [], '非法号码绝不能落盘')

    def test_concurrent_contact_saves_do_not_share_a_tmp_file(self):
        fns = extract_methods(SERVER, 'LightDogServer', ['save_contacts'],
                              ns={'json': json, 'os': os, 'tempfile': tempfile,
                                  'threading': threading})
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, 'contacts.json')
            obj, _ = self._handler(path)
            fn = bind(obj, fns).save_contacts
            errors = []

            def worker(i):
                try:
                    for _ in range(8):
                        fn([{'name': str(i), 'phone': '1001'}])
                except Exception as e:            # noqa: BLE001
                    errors.append(e)

            ts = [threading.Thread(target=worker, args=(i,)) for i in range(6)]
            for t in ts:
                t.start()
            for t in ts:
                t.join()
            self.assertEqual(errors, [], f'并发写失败：{errors}')
            with open(path, encoding='utf-8') as f:
                data = json.load(f)
            self.assertEqual(len(data['contacts']), 1)
            self.assertFalse([n for n in os.listdir(d) if n.endswith('.tmp')])

    def test_bad_clip_timestamp_does_not_500_the_whole_list(self):
        fns = extract_methods(SERVER, 'LightDogServer', ['_clips'],
                              ns={'json': json, 'os': os})
        with tempfile.TemporaryDirectory() as d:
            with open(os.path.join(d, 'seg_0001.mp4'), 'wb') as f:
                f.write(b'x' * 16)
            obj, _ = self._handler(clips_dir=d, meta={
                'seg_0001.mp4': {'file': 'seg_0001.mp4',
                                 't0': None, 't1': 'garbage'}})
            out = bind(obj, fns)._clips()
            self.assertEqual(len(out), 1)
            self.assertEqual(out[0]['bytes'], 16)
            self.assertNotIn('duration', out[0])

    def test_write_path_rejects_chunked_and_sets_body_timeout(self):
        src = open(SERVER, encoding='utf-8').read()
        self.assertIn("Transfer-Encoding", src)
        self.assertIn('BODY_TIMEOUT', src)
        self.assertIn('self.connection.settimeout(self.BODY_TIMEOUT)', src)
        # 超时必须只包住读 body，不能把 SSE/MJPEG 长连接也掐了
        self.assertIn('self.connection.settimeout(None)', src)

    def test_web_template_escapes_event_type(self):
        src = open(os.path.join(PKG, 'web', 'app.js'), encoding='utf-8').read()
        self.assertIn('${esc(ev.type)}', src)
        self.assertNotIn('>${ev.type}</span>', src)


# ---------------------------------------------------------------- 感知


class TestPerceptionDecoding(unittest.TestCase):
    def test_fall_detector_accepts_four_channel_images(self):
        fns = extract_funcs(P5, ['image_to_rgb'],
                            ns={'np': np, 'Image': object})
        h, w = 2, 2
        rgba = np.zeros((h, w, 4), dtype=np.uint8)
        rgba[0, 0] = [10, 20, 30, 255]
        msg = types.SimpleNamespace(height=h, width=w, encoding='rgba8',
                                    data=rgba.tobytes())
        out = fns['image_to_rgb'](msg)
        self.assertIsNotNone(out, 'P2 认 rgba8，P5 不认就会整条链路静默')
        self.assertEqual(out.shape, (h, w, 3))
        self.assertEqual(list(out[0, 0]), [10, 20, 30])
        msg.encoding = 'bgra8'
        self.assertEqual(list(fns['image_to_rgb'](msg)[0, 0]), [30, 20, 10])
        msg.encoding = 'yuv422'
        self.assertIsNone(fns['image_to_rgb'](msg))

    def test_decode_failure_must_emit_unknown_not_return(self):
        src = open(P5, encoding='utf-8').read()
        seg = src[src.index('rgb = image_to_rgb(self.rgb)'):]
        seg = seg[:seg.index('if rgb.shape[:2] != dep.shape[:2]')]
        self.assertIn('self._emit_unknown()', seg)
        self.assertIn('self.get_logger().error', seg)

    def test_p2_camera_info_reads_fy(self):
        fns = extract_methods(P2, 'PersonDetector', ['_on_info'],
                              ns={'math': __import__('math'),
                                  'CameraInfo': object})
        obj = types.SimpleNamespace(fx=None, fy=None, cx=None, cy=None)
        k = [600.0, 0, 320, 0, 780.0, 240, 0, 0, 1]
        bind(obj, fns)._on_info(types.SimpleNamespace(k=k))
        self.assertEqual((obj.fx, obj.fy), (600.0, 780.0))
        obj2 = types.SimpleNamespace(fx=1.0, fy=2.0, cx=0, cy=0)
        bind(obj2, fns)._on_info(types.SimpleNamespace(k=[0.0] + k[1:]))
        self.assertEqual((obj2.fx, obj2.fy), (1.0, 2.0),
                         '内参非法时整个 _on_info 必须原样返回')

    def test_bare_bool_depth_mode_is_not_reversed(self):
        fns = extract_funcs(P5, ['normalize_fall_mode'])
        self.assertEqual(fns['normalize_fall_mode'](False)[0], 'off')
        self.assertEqual(fns['normalize_fall_mode'](True)[0], 'on')


# ---------------------------------------------------------------- 串口 / 录像


class TestSerialWrite(unittest.TestCase):
    """O_NONBLOCK 下 os.write 可能部分写或抛 EAGAIN。"""

    @staticmethod
    def _link(fake_os):
        fns = extract_methods(BRIDGE, 'SerialLink', ['write'], ns={'os': fake_os})
        obj = types.SimpleNamespace(fd=3, log=lambda *_a, **_k: None,
                                    _warned_full=False)
        return bind(obj, fns), obj

    def test_partial_write_is_retried_until_complete(self):
        class Os:
            def __init__(self):
                self.calls = 0

            def write(self, _fd, data):
                self.calls += 1
                return min(3, len(data))

        o = Os()
        link, _ = self._link(o)
        self.assertTrue(link.write(b'0123456789'))
        self.assertGreaterEqual(o.calls, 4, '部分写不能当成整帧写完')

    def test_eagain_is_not_a_link_failure(self):
        class Os:
            def __init__(self):
                self.closed = False

            def write(self, _fd, _data):
                raise BlockingIOError('EAGAIN')

            def close(self):
                self.closed = True

        o = Os()
        link, obj = self._link(o)
        obj.close = lambda: setattr(o, 'closed', True)
        self.assertIsNone(link.write(b'abc'), '缓冲满应返回 None 而不是 False')
        self.assertFalse(o.closed, 'EAGAIN 不能触发重连')

    def test_real_os_error_still_closes_the_link(self):
        class Os:
            def write(self, _fd, _data):
                raise OSError('device gone')

        link, obj = self._link(Os())
        closed = []
        obj.close = lambda: closed.append(1)
        self.assertFalse(link.write(b'abc'))
        self.assertEqual(closed, [1])


class TestRecorderPaths(unittest.TestCase):
    def _clip_path(self, d):
        fns = extract_methods(RECORDER, 'Recorder', ['_clip_path'],
                              ns={'os': os})
        obj = types.SimpleNamespace(clips_dir=d)
        return bind(obj, fns)._clip_path

    def test_index_cannot_delete_outside_clips_dir(self):
        fn = self._clip_path('/tmp/ld_clips')
        for bad in ('../../etc/passwd', '/etc/passwd', '..', '.',
                    'sub/seg.mp4', 'seg\\bad.mp4', 'a\x00b.mp4', '', None):
            self.assertIsNone(fn({'file': bad}), f'{bad!r} 不该拼出路径')
        self.assertTrue(fn({'file': 'seg_0001.mp4'}).endswith('seg_0001.mp4'))

    def test_symlink_guard_and_bounded_assemblers(self):
        src = open(RECORDER, encoding='utf-8').read()
        self.assertIn('os.path.islink(d)', src)
        self.assertIn('if os.path.islink(path):', src)
        self.assertIn('_assemble_sem.acquire(blocking=False)', src)
        self.assertIn('len(self.pending) >= self.MAX_PENDING', src)


# ---------------------------------------------------------------- 验收 / 浸泡


class TestAcceptanceNoPartialPass(unittest.TestCase):
    def test_only_subset_can_not_print_all_passed(self):
        fns = extract_methods(ACCEPTANCE, 'Report',
                              ['verdict', 'count', 'unacked_skips', 'missing'],
                              ns={'time': __import__('time'),
                                  'PASS': 'PASS', 'FAIL': 'FAIL',
                                  'SKIP': 'SKIP', 'MANUAL': '待人工'})
        r = types.SimpleNamespace(
            rows=[{'no': 3, 'title': 'x', 'status': 'PASS',
                   'detail': '', 'evidence': ''}],
            expected={1, 2, 3}, only={3})
        bound = bind(r, fns)
        self.assertEqual(bound.missing(), [1, 2])
        self.assertEqual(bound.verdict(set()), 2,
                         '只跑 1/3 条却判 0 = 假绿')
        r.rows = [dict(r.rows[0], no=n) for n in (1, 2, 3)]
        self.assertEqual(bound.verdict(set()), 0)

    def test_check3_requires_real_cmd_samples(self):
        src = open(ACCEPTANCE, encoding='utf-8').read()
        seg = src[src.index('def check_3('):src.index('def check_4(')]
        self.assertIn('cmd_n < 3', seg)
        self.assertIn('速度出口样本', seg)


class TestSoakLoadRosAccounting(unittest.TestCase):
    def test_missing_ros_publisher_is_a_failure_not_a_silent_skip(self):
        sl = _load(SOAK_LOAD, 'ld_soak_load_review')
        args = types.SimpleNamespace(host='127.0.0.1', port=1, token='',
                                     duration=1.0, cycle=1.0, max_fail=0.05,
                                     no_ros=False)
        L = sl.Load(args)
        L.pub('/voice/command', 'x')
        self.assertGreaterEqual(L.stat.get('ROS:/voice/command', [0, 0])[1], 1)
        # --no-ros 是显式选择，不该被记成失败
        args.no_ros = True
        L2 = sl.Load(args)
        L2.pub('/voice/command', 'x')
        self.assertEqual(L2.stat.get('ROS:/voice/command'), None)

    def test_sse_requires_more_than_one_byte(self):
        src = open(SOAK_LOAD, encoding='utf-8').read()
        seg = src[src.index("self.mark('SSE 长连接"):]
        self.assertIn('min_bytes=32', seg[:200])


class TestSoakInconclusive(unittest.TestCase):
    """「时间跨度够长」不等于「拍数够」。

    旧判据只在源码里找 `if inconclusive:` 字符串：把结论分支改回 `if False:`
    照样全绿（实测 MISSED），因为 soak.py 还有一个追加 warn 的同名分支。
    这里改成直接喂一份「跨 1 小时但只有 5 拍」的 CSV 给 report()，
    看它敢不敢打出「未发现泄漏（rc=0）」。
    """

    def _report(self, times, min_duration=1800.0):
        soak = _load(os.path.join(REPO, 'deploy', 'soak.py'),
                     'ld_soak_report_review')
        d = tempfile.TemporaryDirectory()
        self.addCleanup(d.cleanup)
        path = os.path.join(d.name, 'soak.csv')
        # 与 run() 写出的是同一套列：load_csv 少一列就会 KeyError。
        with open(path, 'w', newline='', encoding='utf-8') as f:
            w = csv.DictWriter(f, fieldnames=soak.COLUMNS)
            w.writeheader()
            for i, t in enumerate(times):
                w.writerow({
                    't': t, 'wall': '2026-11-10T00:00:00',
                    'pid': 4242, 'name': 'lightdog_dummy.py',
                    'rss_kb': 100000, 'vsz_kb': 200000,
                    'threads': 8, 'fds': 20, 'fd_sock': 4, 'fd_pipe': 2,
                    'fd_anon': 3, 'fd_file': 11,
                    'cpu_s': 0.1 * i, 'start': 12345,
                    'fd_limit': 1048576, 'resolved': 1,
                })
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            rc = soak.report(path, min_duration)
        return rc, buf.getvalue()

    def test_report_does_not_claim_no_leak_when_samples_are_too_few(self):
        # 5 拍跨 3600s：dur 远超 --min-duration，但每条序列都不足 6 拍，
        # judge() 只能返回 INCONCLUSIVE。旧实现会因为「没有 LEAK」给 rc=0。
        rc, out = self._report([0.0, 900.0, 1800.0, 2700.0, 3600.0])
        self.assertEqual(rc, 2, '拍数不足却给了绿灯：\n' + out)
        self.assertIn('采样不足 6 拍', out)
        self.assertNotIn('未发现泄漏（rc=0）', out)

    def test_report_gives_zero_when_samples_are_enough(self):
        # 对照组：同样的 1 小时跨度、7 拍全平线，确实应该 rc=0。
        # 没有这条，上一条可以靠「report 永远返回 2」作弊通过。
        rc, out = self._report([i * 600.0 for i in range(7)])
        self.assertEqual(rc, 0, '拍数够且无泄漏时不该拦：\n' + out)
        self.assertIn('未发现泄漏（rc=0）', out)


# ---------------------------------------------------------------- 固件 / 部署静态不变量


class TestFirmwareInvariants(unittest.TestCase):
    """C 自测需要 Linux 上的 make/cc；这些不变量保证改动不会被悄悄退回。"""

    @staticmethod
    def _src(*parts):
        with open(os.path.join(FIRMWARE, *parts), encoding='utf-8') as f:
            return f.read()

    def test_iwdg_is_fed_from_tick_not_from_rx_isr(self):
        app = self._src('app', 'lightdog_app.c')
        self.assertEqual(app.count('iwdg_feed()'), 1)
        self.assertGreater(app.index('iwdg_feed()'),
                           app.index('void ld_app_tick('))
        seg = app[app.index('static void handle_frame('):
                  app.index('void ld_app_rx(')]
        self.assertNotIn('iwdg_feed()', seg, 'RX ISR 里喂狗 = 没有看门狗')
        self.assertIn('LD_WD_TIMEOUT_MS', app)

    def test_downlink_lengths_match_python_decoder(self):
        app = self._src('app', 'lightdog_app.c')
        self.assertIn('f->len == 8u', app)
        self.assertIn('f->len == 4u', app)
        self.assertIn('f->len != 1u', app)
        self.assertNotIn('f->len >= 8u', app)
        self.assertNotIn('f->len >= 4u', app)

    def test_parser_never_consumes_frames_it_cannot_deliver(self):
        proto = self._src('core', 'lightdog_proto.c')
        self.assertIn('count >= max_out', proto)
        self.assertIn('len > 0u', proto)
        app = self._src('app', 'lightdog_app.c')
        self.assertIn('ld_parser_push(&app->parser, buf, (size_t)len, frames, 4)',
                      app)
        self.assertIn('len = 0u', app)

    def test_led_pwm_does_not_share_the_right_wheel_channel(self):
        port = self._src('port', 'stm32f4', 'lightdog_hal_stm32f4.c')
        led = port[port.index('static void hal_led_set('):
                   port.index('/* ---------------------------------------------------------------- 串口')]
        self.assertNotIn('TIM_CHANNEL_4', led)
        self.assertIn('LD_LED_CHANNEL', led)

    def test_selftests_are_wired_into_make_test(self):
        mk = self._src('Makefile')
        line = [x for x in mk.splitlines() if x.startswith('test:')][0]
        self.assertIn('cross_stream', line)
        t = self._src('test', 'test_app.c')
        self.assertIn('test_iwdg_is_fed_by_control_loop_only();', t)
        self.assertIn('test_many_frames_in_one_rx_are_all_handled();', t)
        self.assertIn('跨过 state_period_ms 必须周期重发', t)


class TestDeploymentInvariants(unittest.TestCase):
    @staticmethod
    def _src(*parts):
        with open(os.path.join(REPO, *parts), encoding='utf-8') as f:
            return f.read()

    def test_pipefail_command_substitutions_are_guarded(self):
        for rel in (('deploy', 'sip', 'make_prompt.sh'),
                    ('deploy', 'install.sh')):
            src = self._src(*rel)
            self.assertIn('|| true', src,
                          f'{"/".join(rel)} 的 set -e + pipefail 陷阱没兜住')

    def test_launch_yaml_regex_really_matches(self):
        t = open(os.path.join(PKG, 'tests', 'test_consistency.py'),
                 encoding='utf-8').read()
        self.assertTrue(r"['\"]config['\"]\s*,\s*" in t,
                        'PathJoinSubstitution 那种两段式写法没被正则覆盖')
        self.assertIn('self.assertGreater(hits, 0', t,
                      '没有这条断言，正则再次空转依旧全绿')

    def test_entrypoint_monitors_all_enabled_launches(self):
        e = self._src('sim', 'entrypoint.sh')
        self.assertIn('MONITORED+=(', e)
        self.assertIn('wait -n "${MONITORED[@]}"', e)
        c = self._src('sim', 'docker-compose.yaml')
        self.assertIn('lightdog_server.py', c)

    def test_battery_off_really_stops_the_injection(self):
        p = self._src('sim', 'p3.sh')
        seg = p[p.index('if [ "$v" = "off" ]'):]
        self.assertIn('pkill -f', seg[:400])

    def test_compose_yaml_parses(self):
        import yaml
        data = yaml.safe_load(self._src('sim', 'docker-compose.yaml'))
        hc = data['services']['sim']['healthcheck']['test']
        self.assertIn('${ENABLE_P4:-1}', hc[1],
                      '$$ 被 compose 转义后应还原成单个 $')


if __name__ == '__main__':
    unittest.main(verbosity=2)
