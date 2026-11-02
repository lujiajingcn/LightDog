#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""浸泡测试的载荷驱动器：把「一天里的正常操作」压缩成周期，反复做 N 小时。

## 为什么需要它

「让系统空跑两小时」什么都证明不了：录像线程可能根本没开始写段，SSE 连接
一个都没建过，事件切片一次都没切过。真正会泄漏的路径全在**连接建立/销毁**
和**队列进出**上，不压它们，2 小时后当然一切正常。

所以这里压的是本项目里每一条「会持有资源」的路径：

| 动作 | 压的是哪段代码 |
|---|---|
| `GET /api/v1/status` 新连接 | `http.server` 每连接一个线程（ThreadingHTTPServer） |
| SSE 长连接 + 突然断开 | `_peer_closed()` 那条 select+MSG_PEEK 分支；断不掉就是线程泄漏 |
| MJPEG 读一半就 RST | 推流循环的写失败退出；这是手机掉线的真实形态 |
| `POST /api/v1/events` | recorder 切事件片 + `_flush_q` 的 `put_nowait` 丢段逻辑 |
| `/voice/command` | 语音语义 → 调度动作 → TTS 子进程（每次 spawn 一个进程） |
| `/light/mode` on/off | 灯光节点的渐变状态机（定时器每帧一次） |
| `/alert/command test` + HTTP ack | 告警状态机、`call_attempts`、语音播报 |

## 两个刻意的选择

1. **不用 `ros2 topic pub --once`**：短命进程在 DDS 发现完成前就退出，急停都
   发不进去（`sim/pub_once.py` 的注释里有完整记录）。这里 publisher 全程持有。
2. **断开分「优雅」和「RST」两种**：只测优雅关闭会漏掉「客户端直接消失」，
   而那正是 WiFi 断线的形态 —— 服务端要在写失败时自己收线程。

## 失败率是结论的一部分

如果 2890 个请求全 500，资源曲线依然平得像一条直线，但那不叫稳定，叫全坏。
所以这里统计每一类动作的成功/失败，失败率超过 `--max-fail` 直接返回非零。

## 用法（容器内）

    python3 /sim/soak_load.py --duration 7200 --cycle 30
"""

import argparse
import json
import signal
import socket
import struct
import sys
import time
import traceback
from datetime import datetime

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
except ImportError:                        # 允许 --no-ros 在无 ROS 环境跑 HTTP 部分
    rclpy = None
    Node = object
    String = None


def _now():
    return datetime.now().strftime('%H:%M:%S')


class Load(object):

    def __init__(self, args):
        self.host = args.host
        self.port = args.port
        self.token = args.token
        self.dur = args.duration
        self.cycle_sec = args.cycle
        self.max_fail = args.max_fail
        self.stat = {}                     # kind -> [ok, fail]
        self.errs = []                     # 前若干条错误原文，出问题时要有据可查
        self.t0 = time.monotonic()
        self.cycles = 0
        self.stop = False
        self.node = None
        self.pubs = {}

    # ---------------- 记账

    def ok(self, kind, n=1):
        self.stat.setdefault(kind, [0, 0])[0] += n

    def fail(self, kind, msg):
        self.stat.setdefault(kind, [0, 0])[1] += 1
        if len(self.errs) < 12:
            self.errs.append('[%s] %s: %s' % (_now(), kind, msg))

    def mark(self, text):
        el = time.monotonic() - self.t0
        print('[%s +%6.1fs] %s' % (_now(), el, text), flush=True)

    # ---------------- HTTP

    def _conn(self, timeout=5.0):
        c = socket.create_connection((self.host, self.port), timeout=timeout)
        return c

    def _req(self, sock, method, path, body=None):
        hdr = ('%s %s HTTP/1.1\r\nHost: %s:%d\r\nConnection: close\r\n'
               % (method, path, self.host, self.port))
        if self.token:
            hdr += 'X-Auth-Token: %s\r\n' % self.token
        if body is not None:
            hdr += 'Content-Type: application/json\r\n'
            hdr += 'Content-Length: %d\r\n' % len(body)
        sock.sendall(hdr.encode() + b'\r\n' + (body or b''))

    def get(self, path, timeout=5.0):
        try:
            s = self._conn(timeout)
            self._req(s, 'GET', path)
            data = b''
            while True:
                b = s.recv(65536)
                if not b:
                    break
                data += b
                if len(data) > 4 * 1024 * 1024:
                    break
            s.close()
            code = data.split(b'\r\n', 1)[0]
            if b' 200 ' not in code:
                self.fail('GET ' + path, code.decode('latin-1', 'replace')[:60])
                return None
            self.ok('GET ' + path)
            head, _, payload = data.partition(b'\r\n\r\n')
            # chunked 只在部分接口出现，这里不解析 —— 需要 JSON 的地方用下面那个
            return payload if b'Transfer-Encoding: chunked' not in head else None
        except Exception as e:
            self.fail('GET ' + path, e)
            return None

    def post(self, path, obj):
        body = json.dumps(obj).encode()
        try:
            s = self._conn()
            self._req(s, 'POST', path, body)
            data = b''
            while True:
                b = s.recv(65536)
                if not b:
                    break
                data += b
                if len(data) > 1024 * 1024:
                    break
            s.close()
            code = data.split(b'\r\n', 1)[0]
            # 2xx 都算成功：/api/v1/alarms/ack 返回 202（命令是异步下发的），
            # 只认 200 会把它记成失败，而失败率是要进结论的。
            if b' 2' not in code[:12]:
                self.fail('POST ' + path, code.decode('latin-1', 'replace')[:60])
                return False
            self.ok('POST ' + path)
            return True
        except Exception as e:
            self.fail('POST ' + path, e)
            return False

    @staticmethod
    def _rst(sock):
        """发 RST 而不是 FIN：模拟手机掉网。服务端 read 会直接抛 ECONNRESET，
        而不是收到一个干净的 EOF —— 两条路径的清理代码不一样。"""
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_LINGER,
                            struct.pack('ii', 1, 0))
        except OSError:
            pass
        try:
            sock.close()
        except OSError:
            pass

    def stream(self, kind, path, secs, max_bytes, rst=True, min_bytes=1):
        """建一个长连接，读一会儿，然后要么优雅关要么 RST。

        `min_bytes` 是**响应体**字节数的下限（不含 HTTP 头）。这一条是必须的：
        推流线程死掉之后连接照样建得起来、200 照样回得来，只是后面一个字节都
        没有。原先不看字节数，这种「连上但没数据」会被记成成功 —— 2 小时跑完
        失败率 0%，报告写「载荷执行正常」，而手机屏幕上是一片黑。
        """
        s = None
        try:
            s = self._conn(timeout=secs + 5.0)
            self._req(s, 'GET', path)
            s.settimeout(0.5)
            got = 0
            body = 0
            buf = b''
            hdr_done = False
            deadline = time.monotonic() + secs
            while time.monotonic() < deadline and got < max_bytes:
                try:
                    b = s.recv(65536)
                except socket.timeout:
                    continue
                if not b:
                    break
                got += len(b)
                if not hdr_done:
                    # 头可能跨包到达，攒着找 \r\n\r\n，别假设「第一次 recv 就是完整头」
                    buf += b
                    idx = buf.find(b'\r\n\r\n')
                    if idx >= 0:
                        hdr_done = True
                        body += len(buf) - idx - 4
                        buf = b''
                    elif len(buf) > 65536:
                        hdr_done = True       # 头大得离谱，剩下的都当 body
                        buf = b''
                else:
                    body += len(b)
            if body < min_bytes:
                self.fail(kind, '%ds 内响应体只有 %d 字节（要求 ≥%d）：'
                                '连上了但没有数据' % (int(secs), body, min_bytes))
                return body
            self.ok(kind)
            return body
        except Exception as e:
            self.fail(kind, e)
            return 0
        finally:
            if s is not None:
                if rst:
                    self._rst(s)
                else:
                    try:
                        s.close()
                    except OSError:
                        pass

    # ---------------- ROS

    def start_ros(self):
        if rclpy is None or self.node is not None:
            return
        rclpy.init()
        self.node = rclpy.create_node('soak_load')
        for topic in ('/voice/command', '/light/mode', '/alert/command'):
            self.pubs[topic] = self.node.create_publisher(String, topic, 10)

    def pub(self, topic, text, settle=0.4):
        """发一条并 spin 一会儿。不 spin 就等于发完立刻析构，消息可能没出去。"""
        p = self.pubs.get(topic)
        if p is None:
            return
        m = String()
        m.data = text
        p.publish(m)
        t_end = time.monotonic() + settle
        while time.monotonic() < t_end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    # ---------------- 一个周期

    def cycle(self, i):
        self.mark('周期 #%d：HTTP 短连接' % i)
        for _ in range(5):
            self.get('/api/v1/status')
        self.get('/')
        self.get('/api/v1/health')
        self.get('/api/v1/alarms')

        self.mark('SSE 长连接 8s 后 RST')
        self.stream('SSE', '/api/v1/events/stream', 8.0, 1 << 20, rst=True)

        self.mark('MJPEG 读 4s 后 RST')
        # 至少一帧的开头：4 秒里一个字节都没有，说明推流线程已经死了
        self.stream('MJPEG', '/api/v1/stream.mjpg', 4.0, 400 * 1024,
                    rst=True, min_bytes=1024)

        # 每三轮一次优雅关闭：两条清理路径都要走到
        if i % 3 == 0:
            self.stream('SSE-graceful', '/api/v1/events/stream', 3.0, 1 << 20,
                        rst=False)

        self.mark('快照')
        self.get('/api/v1/snapshot.jpg')

        if i % 3 == 1:
            self.mark('事件切片（recorder 写盘）')
            self.post('/api/v1/events',
                      {'type': 'manual', 'label': '浸泡测试 #%d' % i})

        self.mark('语音 / 灯光')
        self.pub('/voice/command', '把灯打开' if i % 2 == 0 else '关灯')
        time.sleep(1.0)
        self.pub('/light/mode', 'on' if i % 2 == 0 else 'off')
        time.sleep(1.0)

        if i % 4 == 2:
            self.mark('告警：test → HTTP ack → reset')
            self.pub('/alert/command', 'test')
            time.sleep(5.0)
            # 注意 URL 里是**动作**不是告警 id：`/api/v1/alarms/<ack|test|reset>`。
            # 这两条走 HTTP 而不是话题，是为了同时压「手机点按钮」那条路。
            self.post('/api/v1/alarms/ack', {})
            time.sleep(2.0)
            self.post('/api/v1/alarms/reset', {})
            time.sleep(1.0)

    def run(self):
        self.start_ros()
        self.mark('载荷开始：目标 %s:%d，时长 %.0fs，周期 %.0fs'
                  % (self.host, self.port, self.dur, self.cycle_sec))
        i = 0
        try:
            while not self.stop:
                el = time.monotonic() - self.t0
                if el >= self.dur:
                    break
                self.cycle(i)
                i += 1
                self.cycles = i
                # 睡到下一周期起点。留出的空档不是浪费：录像是按 10 秒切段的，
                # 一直压着反而看不到「没有请求时它会不会自己停住」。
                nxt = self.t0 + i * self.cycle_sec
                while not self.stop:
                    gap = nxt - time.monotonic()
                    if gap <= 0:
                        break
                    time.sleep(min(gap, 0.5))
        except KeyboardInterrupt:
            self.mark('收到中断')
        return self.summary()

    def summary(self):
        el = time.monotonic() - self.t0
        print('')
        print('=' * 72)
        print('载荷汇总   运行 %.1f 分钟 / 完成 %d 个周期'
              % (el / 60.0, self.cycles))
        print('=' * 72)
        tot_ok = tot_fail = 0
        for k in sorted(self.stat):
            o, f = self.stat[k]
            tot_ok += o
            tot_fail += f
            flag = '  <-- 有失败' if f else ''
            print('%-28s 成功 %5d  失败 %5d%s' % (k, o, f, flag))
        print('-' * 72)
        for e in self.errs:
            print('  ' + e)
        n = tot_ok + tot_fail
        rate = (float(tot_fail) / n * 100.0) if n else 100.0
        print('-' * 72)
        print('总计 %d 次请求，失败 %d 次（%.1f%%）' % (n, tot_fail, rate))
        if n == 0:
            print('一次请求都没发出去 —— 这不是「稳定」，是没测着。')
            return 3
        if rate > self.max_fail * 100.0:
            print('失败率超过 %.0f%%：资源曲线再平也不能算通过。' % (self.max_fail * 100))
            return 1
        print('载荷执行正常。')
        return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description='浸泡测试的载荷驱动器')
    ap.add_argument('--duration', type=float, default=7200.0)
    ap.add_argument('--cycle', type=float, default=30.0)
    ap.add_argument('--host', default='127.0.0.1')
    ap.add_argument('--port', type=int, default=8000)
    ap.add_argument('--token', default='')
    ap.add_argument('--max-fail', type=float, default=0.05,
                    help='允许的失败率（0.05 = 5%%）')
    ap.add_argument('--no-ros', action='store_true', help='只压 HTTP，不发 ROS 消息')
    args = ap.parse_args(argv)

    if args.no_ros:
        global rclpy
        rclpy = None

    L = Load(args)

    def _stop(_s, _f):
        L.stop = True
    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    try:
        return L.run()
    except Exception:
        traceback.print_exc()
        return 1
    finally:
        if L.node is not None:
            try:
                L.node.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()
            except Exception:
                pass


if __name__ == '__main__':
    sys.exit(main())
