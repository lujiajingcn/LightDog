"""载荷驱动器（sim/soak_load.py）的离线回归。

这个文件只测一件事：**什么算成功**。

资源曲线平不平是 `deploy/soak.py` 的事；这里决定的是「那些曲线是不是在一次都
没真正跑起来的情况下画出来的」。会漏的正是这一格 —— 推流线程死掉之后，连接
照样建得起来、200 照样回得来，只是后面一个字节都没有。如果这被记成成功，
2 小时跑完失败率 0%，报告写「载荷执行正常」，而手机屏幕是黑的。

`sim/` 不在包里（容器只挂 `src/lightdog`），所以这组用例在容器里会跳过。
"""

import importlib.util
import os
import socket
import sys
import threading
import time
import unittest
from contextlib import redirect_stdout
from io import StringIO

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from harness import PKG  # noqa: E402

REPO_ROOT = os.path.dirname(PKG)
LOAD = os.path.join(REPO_ROOT, 'sim', 'soak_load.py')
HAS_LOAD = os.path.isfile(LOAD)


def _import_load():
    spec = importlib.util.spec_from_file_location('lightdog_soak_load', LOAD)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


sl = _import_load() if HAS_LOAD else None


class _Args(object):
    """Load.__init__ 只读这几个字段，不必起完整的 argparse。"""
    host = '127.0.0.1'
    token = ''
    duration = 10.0
    cycle = 1.0
    max_fail = 0.05
    no_ros = True


def _server(reply):
    """起一个只回一次的服务端，返回 (srv, port)。

    reply=None 表示**接受连接但什么都不发** —— 推流线程死掉之后就是这个形态。
    """
    srv = socket.socket()
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(('127.0.0.1', 0))
    srv.listen(4)
    port = srv.getsockname()[1]

    def serve():
        while True:
            try:
                c, _ = srv.accept()
            except OSError:
                return
            try:
                c.recv(4096)
                if reply:
                    c.sendall(reply)
                time.sleep(1.0)
            except OSError:
                pass
            finally:
                try:
                    c.close()
                except OSError:
                    pass

    threading.Thread(target=serve, daemon=True).start()
    return srv, port


@unittest.skipUnless(HAS_LOAD, '找不到 sim/soak_load.py（容器只挂包目录）')
class TestStreamCounting(unittest.TestCase):
    """`stream()` 判的是响应体字节数，不是「连上了没有」。"""

    def _stream(self, reply, secs=0.6, min_bytes=1024):
        srv, port = _server(reply)
        try:
            a = _Args()
            a.port = port
            load = sl.Load(a)
            n = load.stream('MJPEG', '/x', secs, 400 * 1024, min_bytes=min_bytes)
        finally:
            srv.close()
        return n, load.stat

    def test_body_bytes_exclude_http_header(self):
        body = b'x' * 2000
        n, stat = self._stream(
            b'HTTP/1.1 200 OK\r\nContent-Type: image/jpeg\r\n\r\n' + body)
        self.assertEqual(n, len(body),
                         '把 HTTP 头算进了 body，字节数偏大')
        self.assertEqual(stat['MJPEG'], [1, 0])

    def test_silent_server_is_a_failure(self):
        """accept 之后一字节不发：这正是推流线程死掉的形态。"""
        n, stat = self._stream(None)
        self.assertEqual(n, 0)
        self.assertEqual(stat['MJPEG'], [0, 1],
                         '连上但零字节被记成了成功')

    def test_header_only_is_a_failure(self):
        """回了 200 但不发帧：比完全沉默更容易骗过「连接成功」这种判据。"""
        n, stat = self._stream(
            b'HTTP/1.1 200 OK\r\nContent-Type: image/jpeg\r\n\r\n')
        self.assertEqual(n, 0)
        self.assertEqual(stat['MJPEG'], [0, 1])

    def test_sse_retry_line_counts_as_alive(self):
        """SSE 建连后服务端先写 `retry: 3000`，那 13 个字节就是「流活着」的证据。"""
        n, stat = self._stream(
            b'HTTP/1.1 200 OK\r\nContent-Type: text/event-stream\r\n\r\n'
            b'retry: 3000\n\n', min_bytes=1)
        self.assertGreater(n, 0)
        self.assertEqual(stat['MJPEG'], [1, 0])


@unittest.skipUnless(HAS_LOAD, '找不到 sim/soak_load.py（容器只挂包目录）')
class TestSummaryVerdict(unittest.TestCase):
    """汇总的退出码。载荷一次都没发出去时不能报「正常」。"""

    def _summary(self, kind_ok, kind_fail):
        a = _Args()
        a.port = 1
        load = sl.Load(a)
        load.t0 = time.monotonic() - 60.0
        load.cycles = 1
        if kind_ok or kind_fail:
            load.stat['MJPEG'] = [kind_ok, kind_fail]
        buf = StringIO()
        with redirect_stdout(buf):
            rc = load.summary()
        return rc, buf.getvalue()

    def test_zero_requests_is_not_success(self):
        rc, out = self._summary(0, 0)
        self.assertEqual(rc, 3)
        self.assertIn('没测着', out)

    def test_high_failure_rate_fails(self):
        rc, out = self._summary(1, 9)
        self.assertEqual(rc, 1)
        self.assertIn('不能算通过', out)

    def test_clean_run_passes(self):
        rc, out = self._summary(100, 0)
        self.assertEqual(rc, 0)
        self.assertIn('正常', out)
