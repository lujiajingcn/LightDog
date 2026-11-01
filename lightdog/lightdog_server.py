#!/usr/bin/env python3
"""
LightDog P4：手机互联后端

## 为什么是纯标准库

这个容器**没有外网**（pypi / apt 都不通，`ffmpeg` 都没有）。所以 Flask / FastAPI /
aiohttp 一个都装不上，`http.server` 是唯一选择。这不是妥协 —— 对一个只服务
「家里的手机看机器人画面」的端点来说，标准库完全够用，而且没有依赖就没有供应链。

## 一条进程，两个世界

    rclpy 节点  跑在后台线程，订阅相机 / 电量 / 回充 / 跟随状态
    HTTPServer  跑在主线程，给手机提供 HTTP

两者通过一把锁共享「最新一帧 JPEG」。刻意不做成两个进程：相机帧走 ROS 话题跨进程
复制 640x480x3 大约 0.9MB/帧，在一个进程里直接拿到更省事，也少一层失败点。

## 接口

    GET  /                              手机 H5 页面
    GET  /api/v1/health                 健康检查（唯一免鉴权）
    GET  /api/v1/status                 电量 / 状态 / 存储 / 相机
    GET  /api/v1/stream.mjpg            实时 MJPEG（multipart/x-mixed-replace）
    GET  /api/v1/snapshot.jpg           当前单帧（MJPEG 不可用时的降级方案）
    GET  /api/v1/events                 事件列表
    POST /api/v1/events                 触发事件 {type,label,pre,post,note}
    GET  /api/v1/events/<id>            事件元数据
    GET  /api/v1/events/<id>/video      事件视频（支持 Range）
    GET  /api/v1/events/<id>/thumb.jpg  事件缩略图
    GET  /api/v1/clips                  环形分段列表
    GET  /api/v1/clips/<name>           分段视频（支持 Range）
    GET  /api/v1/snapshots              快照列表
    GET  /api/v1/snapshots/<name>       快照
    GET  /api/v1/contacts               紧急联系人
    PUT  /api/v1/contacts               覆盖紧急联系人

## 鉴权

`auth_token` 为空 = 不鉴权（仿真默认）。真机部署必须设置，否则同一个 WiFi 下
任何人都能看摄像头。token 可以用三种方式带：

    ?token=xxx          （`<img>` / `<video>` 这种没法带 header 的必须用这个）
    X-Auth-Token: xxx
    Authorization: Bearer xxx

## Range 为什么必须实现

iOS Safari 播 `<video>` 会先发 `Range: bytes=0-1` 探一下，服务端回 `200` 而不是
`206` 时它直接拒绝播放（且不报错，只是黑屏）。这是「手机上看录像」最容易踩的坑。
"""

import json
import os
import shutil
import socket
import sys
import threading
import time
import urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from ament_index_python.packages import get_package_share_directory

from sensor_msgs.msg import BatteryState, Image
from std_msgs.msg import Bool, Float32, String


MIME = {
    '.html': 'text/html; charset=utf-8',
    '.js': 'application/javascript; charset=utf-8',
    '.css': 'text/css; charset=utf-8',
    '.json': 'application/json; charset=utf-8',
    '.jpg': 'image/jpeg',
    '.jpeg': 'image/jpeg',
    '.png': 'image/png',
    '.mp4': 'video/mp4',
    '.ico': 'image/x-icon',
}


def image_to_bgr(msg: Image):
    """sensor_msgs/Image -> (H, W, 3) BGR；与 recorder 保持同一套解码逻辑。"""
    h, w = msg.height, msg.width
    if h <= 0 or w <= 0:
        return None
    enc = msg.encoding.lower()
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    if enc in ('rgb8', 'bgr8'):
        if buf.size < h * w * 3:
            return None
        a = buf[:h * w * 3].reshape(h, w, 3)
        return a.copy() if enc == 'bgr8' else a[:, :, ::-1].copy()
    if enc in ('rgba8', 'bgra8'):
        if buf.size < h * w * 4:
            return None
        a = buf[:h * w * 4].reshape(h, w, 4)[:, :, :3]
        return a.copy() if enc == 'bgra8' else a[:, :, ::-1].copy()
    if enc in ('mono8', '8uc1'):
        if buf.size < h * w:
            return None
        return cv2.cvtColor(buf[:h * w].reshape(h, w), cv2.COLOR_GRAY2BGR)
    return None


class FrameHub:
    """相机帧的单一存放点：ROS 回调写，HTTP 线程读。"""

    def __init__(self, quality=75):
        self._cv = threading.Condition()
        self._jpeg = None
        self._seq = 0
        self._t = None
        self._fps = 0.0
        self._last = None
        self._quality = quality
        self._count = 0
        self._width = 0
        self._height = 0

    def put(self, bgr):
        ok, buf = cv2.imencode('.jpg', bgr,
                               [int(cv2.IMWRITE_JPEG_QUALITY), self._quality])
        if not ok:
            return
        now = time.time()
        with self._cv:
            if self._last is not None:
                dt = now - self._last
                if dt > 1e-3:
                    inst = 1.0 / dt
                    self._fps = inst if self._fps <= 0 else (self._fps * 0.8 + inst * 0.2)
            self._last = now
            self._jpeg = buf.tobytes()
            self._seq += 1
            self._t = now
            self._count += 1
            self._height, self._width = bgr.shape[0], bgr.shape[1]
            self._cv.notify_all()

    def get(self):
        with self._cv:
            return self._jpeg, self._seq, self._t

    def wait_newer(self, last_seq, timeout=2.0):
        """等到比 last_seq 更新的帧；超时返回当前值（用于超时续命/断线检测）。"""
        deadline = time.time() + timeout
        with self._cv:
            while self._seq <= last_seq:
                left = deadline - time.time()
                if left <= 0:
                    break
                self._cv.wait(left)
            return self._jpeg, self._seq, self._t

    def stats(self):
        with self._cv:
            age = (time.time() - self._t) if self._t else -1.0
            return {
                'has_frame': self._jpeg is not None,
                'fps': round(self._fps, 2),
                'age': round(age, 2),
                'frames': self._count,
                'width': self._width,
                'height': self._height,
            }


class LightDogServer(Node):

    def __init__(self):
        super().__init__('lightdog_server')

        self.declare_parameter('http_host', '0.0.0.0')
        self.declare_parameter('http_port', 8000)
        self.declare_parameter('auth_token', '')
        self.declare_parameter('data_dir', '/data')
        self.declare_parameter('web_dir', '')
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('battery_topic', '/battery_state')
        self.declare_parameter('alert_topic', '/alert/state')
        self.declare_parameter('jpeg_quality', 75)
        self.declare_parameter('stream_fps', 10.0)      # 推流上限（不是相机帧率）
        self.declare_parameter('device_name', 'LightDog')

        p = self.get_parameter
        self.host = str(p('http_host').value)
        self.port = int(p('http_port').value)
        self.token = str(p('auth_token').value)
        self.data_dir = str(p('data_dir').value)
        self.image_topic = str(p('image_topic').value)
        self.quality = int(p('jpeg_quality').value)
        self.stream_fps = max(0.5, float(p('stream_fps').value))
        self.device_name = str(p('device_name').value)

        web = str(p('web_dir').value)
        if not web:
            try:
                web = os.path.join(get_package_share_directory('lightdog'), 'web')
            except Exception:
                web = ''
        self.web_dir = web

        self.clips_dir = os.path.join(self.data_dir, 'clips')
        self.events_dir = os.path.join(self.data_dir, 'events')
        self.snaps_dir = os.path.join(self.data_dir, 'snapshots')
        self.contacts_path = os.path.join(self.data_dir, 'contacts.json')
        for d in (self.clips_dir, self.events_dir, self.snaps_dir):
            os.makedirs(d, exist_ok=True)

        self.hub = FrameHub(quality=self.quality)
        self.t0 = time.time()

        # 从其它节点收集来的「软状态」
        self.recorder = {}
        self.dock = {}
        self.battery = None          # /dock/battery_ratio（P3 算好的）
        self.battery_raw = None      # /battery_state 兜底，P3 没起时也能显示电量
        self.person_present = False
        self.mux_source = ''
        self.alert = {}              # /alert/state（P5 告警状态机）
        self._alert_logged = ''      # 已经落盘过的最新 (alarm_id, state)

        sensor_qos = QoSProfile(depth=1,
                                reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Image, self.image_topic, self._on_image, sensor_qos)
        self.create_subscription(BatteryState, str(p('battery_topic').value),
                                 self._on_battery_state, 5)
        self.create_subscription(String, '/recorder/state', self._on_recorder, 5)
        self.create_subscription(String, '/dock/state', self._on_dock, 5)
        self.create_subscription(Float32, '/dock/battery_ratio', self._on_battery, 5)
        self.create_subscription(Bool, '/person/present', self._on_person, 5)
        self.create_subscription(String, '/cmd_vel_mux_state', self._on_mux, 5)
        self.create_subscription(String, str(p('alert_topic').value), self._on_alert, 10)

        self.pub_event = self.create_publisher(String, '/recorder/event', 10)
        self.pub_alert_cmd = self.create_publisher(String, '/alert/command', 10)

        self.get_logger().info(
            f'LightDog server: http://{self._lan_ip()}:{self.port}/ '
            f'web={self.web_dir or "(缺失)"} data={self.data_dir} '
            f'auth={"on" if self.token else "OFF"}')

    # ---------- ROS 回调 ----------

    def _on_image(self, msg: Image):
        img = image_to_bgr(msg)
        if img is not None:
            self.hub.put(img)

    def _on_recorder(self, msg: String):
        try:
            self.recorder = json.loads(msg.data)
        except Exception:
            pass

    def _on_dock(self, msg: String):
        self.dock['state'] = msg.data.strip('"')

    def _on_battery(self, msg: Float32):
        self.battery = float(msg.data)

    def _on_battery_state(self, msg: BatteryState):
        """兜底电量。P3 的 dock_manager 没起时，/dock/battery_ratio 没有发布者，
        手机上就会一直显示 `--`；这里直接读 Gazebo 桥出来的原始电池。

        优先级与 dock_manager 保持一致：charge/capacity 最准（Ah），
        percentage 只有驱动不报容量时才用。
        """
        if msg.capacity > 0.0 and 0.0 <= msg.charge <= msg.capacity:
            self.battery_raw = float(msg.charge / msg.capacity)
        elif 0.0 <= msg.percentage <= 1.0:
            self.battery_raw = float(msg.percentage)

    def _on_person(self, msg: Bool):
        self.person_present = bool(msg.data)

    def _on_mux(self, msg: String):
        self.mux_source = msg.data.strip('"')

    def _on_alert(self, msg: String):
        try:
            a = json.loads(msg.data)
        except Exception:
            return
        if not isinstance(a, dict):
            return
        self.alert = a

        # 告警事件日志：只在 (alarm_id, state) 变化时落一行，
        # 否则 1Hz 的状态上报会把 alarms.jsonl 写爆。
        key = '%s|%s' % (a.get('alarm_id', ''), a.get('state', ''))
        if key == self._alert_logged:
            return
        st = a.get('state', '')
        # NORMAL 也要落一行：人工 reset 是从 ALARM 直接跳 NORMAL 的（不经过
        # RESOLVED），不记的话这条告警在历史里会永远停在"告警中"。
        if st in ('ALARM', 'ACKED', 'RESOLVED', 'NORMAL') and a.get('alarm_id'):
            self._alert_logged = key
            self._append_alarm(a)

    def _append_alarm(self, a):
        rec = dict(a)
        rec['logged_wall'] = round(time.time(), 3)
        path = os.path.join(self.data_dir, 'alarms.jsonl')
        try:
            with open(path, 'a', encoding='utf-8') as f:
                f.write(json.dumps(rec, ensure_ascii=False) + '\n')
            self.get_logger().warn(
                f'告警落盘：{a.get("alarm_id")} -> {a.get("state")} '
                f'（联系人 {a.get("contact_count", 0)} 个）')
        except Exception as e:
            self.get_logger().error(f'写 {path} 失败：{e}')

    # ---------- 汇总 ----------

    def _lan_ip(self):
        """取本机对外 IP（用来打印「手机该访问哪个地址」）。

        连一个不存在的地址不会真的发包，只是让内核选出默认路由的源地址 ——
        比遍历网卡列表再猜哪个是 WiFi 可靠。
        """
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.settimeout(0.2)
            s.connect(('10.255.255.255', 1))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except Exception:
            return '127.0.0.1'

    def _disk(self):
        try:
            u = shutil.disk_usage(self.data_dir)
            return {'free_mb': round(u.free / 1048576.0, 1),
                    'total_mb': round(u.total / 1048576.0, 1)}
        except Exception:
            return {}

    def event_of_alarm(self, alarm_id):
        """按 `note` 字段精确找回某条告警对应的那段视频。

        为什么不能只按时间猜：`newest_event_since` 找的是「wall 之后最新的一个」，
        对"正在响的告警"够用，但翻历史记录时会串到后面那次告警的视频上去。
        `lightdog_alert_manager` 触发切片时会带上 `note=alert_manager:<alarm_id>`，
        recorder 原样存进 meta.json —— 拿它做唯一关联就没这个问题。
        """
        if not alarm_id:
            return ''
        want = 'alert_manager:%s' % alarm_id
        try:
            for name in os.listdir(self.events_dir):
                d = os.path.join(self.events_dir, name)
                # 必须有 video.mp4：告警刚触发时目录已建、视频还在合成
                if not os.path.isfile(os.path.join(d, 'video.mp4')):
                    continue
                try:
                    with open(os.path.join(d, 'meta.json'), 'r', encoding='utf-8') as f:
                        m = json.load(f)
                except Exception:
                    continue
                if m.get('note') == want:
                    return str(m.get('id') or name)
        except FileNotFoundError:
            pass
        return ''

    def current_alert(self):
        """当前告警状态，并补上 `event_id`。

        告警视频是异步的（recorder 要等 post 窗口才合成完），所以 event_id
        不能在告警那一刻定死，只能在有人来查的时候现查一次 —— 视频好了自然就出现。

        `/status` 和 `/api/v1/alarms` 都走这里：两个接口各算一份的话，
        手机端轮询 /status 拿到了 event_id、切到事件页却发现 `current` 里没有，
        这种"同一个东西两副面孔"的 bug 最难查。
        """
        a = dict(self.alert) if self.alert else {}
        # 告警中才去查视频：既省 IO，也避免 NORMAL 时无谓地扫事件目录
        if a.get('state') in ('ALARM', 'ACKED') and not a.get('event_id'):
            ev = (self.event_of_alarm(a.get('alarm_id'))
                  or self.newest_event_since(float(a.get('alarm_wall') or 0) - 1.0))
            if ev:
                a['event_id'] = ev
        return a

    def status(self):
        cs, _ = self.load_contacts()
        alert = self.current_alert()
        st = {
            'device': {
                'name': self.device_name,
                'time': time.time(),
                'time_str': time.strftime('%Y-%m-%d %H:%M:%S'),
                'uptime_s': round(time.time() - self.t0, 1),
                'lan_ip': self._lan_ip(),
                'port': self.port,
            },
            'camera': self.hub.stats(),
            'battery': {'ratio': self.battery if self.battery is not None
                        else self.battery_raw},
            'dock': {'state': self.dock.get('state', '')},
            'person': {'present': self.person_present},
            'motion': {'source': self.mux_source},
            'recorder': self.recorder,
            'storage': dict(self._disk(), clips=len(self._clips())),
            'contacts': len(cs),
            'alert': alert,
        }
        return st

    # ---------- 文件索引 ----------

    def _clips(self):
        out = []
        try:
            for name in os.listdir(self.clips_dir):
                if not (name.startswith('seg_') and name.endswith('.mp4')):
                    continue
                try:
                    p = os.path.join(self.clips_dir, name)
                    out.append({'file': name, 'bytes': os.path.getsize(p),
                                'mtime': os.path.getmtime(p)})
                except OSError:
                    continue
        except FileNotFoundError:
            pass
        out.sort(key=lambda c: c['file'])
        return out

    def _events(self):
        out = []
        try:
            names = sorted(os.listdir(self.events_dir), reverse=True)
        except FileNotFoundError:
            return out
        for name in names:
            meta_p = os.path.join(self.events_dir, name, 'meta.json')
            if not os.path.isfile(meta_p):
                continue
            try:
                with open(meta_p, 'r', encoding='utf-8') as f:
                    meta = json.load(f)
            except Exception:
                continue
            meta.setdefault('id', name)
            out.append(meta)
        out.sort(key=lambda m: m.get('wall', 0), reverse=True)
        return out

    def event_dir(self, eid):
        """只接受纯名字，挡掉 `../` 之类的路径穿越。"""
        if not eid or '/' in eid or '\\' in eid or eid in ('.', '..'):
            return None
        d = os.path.join(self.events_dir, eid)
        return d if os.path.isdir(d) else None

    def newest_event_since(self, wall):
        """找 wall 之后生成的最新事件（必须有 video.mp4）。

        P5 的告警视频是异步的：告警在 t 时刻触发，recorder 要等到 t+post
        才合成完。所以不能在告警时就把 event_id 定下来 —— 手机轮询 status 时
        现查，等视频好了自然就出现了。
        """
        best = None
        try:
            for name in os.listdir(self.events_dir):
                d = os.path.join(self.events_dir, name)
                if not os.path.isfile(os.path.join(d, 'video.mp4')):
                    continue
                try:
                    with open(os.path.join(d, 'meta.json'), 'r', encoding='utf-8') as f:
                        m = json.load(f)
                except Exception:
                    continue
                w = float(m.get('wall') or 0.0)
                if w >= wall and (best is None or w > best[0]):
                    best = (w, str(m.get('id') or name))
        except FileNotFoundError:
            pass
        return best[1] if best else ''

    def alarms(self):
        """把 alarms.jsonl 这个事件日志按 alarm_id 折叠成"每条告警的最终状态"。"""
        path = os.path.join(self.data_dir, 'alarms.jsonl')
        by_id = {}
        order = []
        try:
            with open(path, 'r', encoding='utf-8') as f:
                for line in f:
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        r = json.loads(line)
                    except Exception:
                        continue
                    aid = r.get('alarm_id') or ''
                    if not aid:
                        continue
                    if aid not in by_id:
                        order.append(aid)
                        by_id[aid] = dict(r)
                    else:
                        prev = by_id[aid]
                        prev['state'] = r.get('state', prev['state'])
                        prev['call_attempts'] = r.get('call_attempts', prev.get('call_attempts', []))
                        prev['rounds'] = r.get('rounds', prev.get('rounds', 0))
        except FileNotFoundError:
            return []
        out = [by_id[a] for a in order]
        out.reverse()                       # 最新的在前
        for a in out:
            if a.get('event_id'):
                continue
            # 先按 note 精确关联（历史记录也能拿到视频），
            # 老记录没有 note 时才退回"按时间找最近一条"。
            ev = self.event_of_alarm(a.get('alarm_id'))
            if not ev and a.get('state') in ('ALARM', 'ACKED'):
                ev = self.newest_event_since(float(a.get('alarm_wall') or 0) - 1.0)
            if ev:
                a['event_id'] = ev
        return out

    # ---------- 联系人 ----------

    def load_contacts(self):
        try:
            with open(self.contacts_path, 'r', encoding='utf-8') as f:
                data = json.load(f)
            if isinstance(data, list):
                return data, None
            if isinstance(data, dict) and isinstance(data.get('contacts'), list):
                return data['contacts'], None
        except FileNotFoundError:
            return [], None
        except Exception as e:
            return [], str(e)
        return [], None

    def save_contacts(self, contacts):
        tmp = self.contacts_path + '.part'
        with open(tmp, 'w', encoding='utf-8') as f:
            json.dump({'contacts': contacts}, f, ensure_ascii=False, indent=2)
        os.replace(tmp, self.contacts_path)


# ======================================================================
# HTTP 层
# ======================================================================

class Handler(BaseHTTPRequestHandler):
    server_version = 'LightDog/0.4'
    protocol_version = 'HTTP/1.1'

    # ---- 基础设施 ----

    @property
    def node(self) -> LightDogServer:
        return self.server.node

    def log_message(self, fmt, *args):
        # http.server 默认往 stderr 打访问日志，太吵；真正的异常我们自己在下面打
        pass

    def _query(self):
        q = urllib.parse.urlparse(self.path).query
        return urllib.parse.parse_qs(q, keep_blank_values=True)

    def _authed(self):
        tok = self.node.token
        if not tok:
            return True
        q = self._query()
        if q.get('token', [''])[0] == tok:
            return True
        if self.headers.get('X-Auth-Token', '') == tok:
            return True
        a = self.headers.get('Authorization', '')
        if a.startswith('Bearer ') and a[7:].strip() == tok:
            return True
        return False

    def _send(self, code, body=b'', ctype='application/json; charset=utf-8',
              extra=None):
        if isinstance(body, str):
            body = body.encode('utf-8')
        self.send_response(code)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(body)))
        # 手机端是同源访问，加 CORS 主要是方便直接在电脑浏览器里调接口
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Headers', 'X-Auth-Token, Authorization, Content-Type')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, PUT, OPTIONS')
        for k, v in (extra or {}).items():
            self.send_header(k, v)
        self.end_headers()
        if self.command != 'HEAD':
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass

    def _json(self, obj, code=200):
        self._send(code, json.dumps(obj, ensure_ascii=False))

    def _err(self, code, msg):
        self._json({'error': msg, 'code': code}, code)

    def _file(self, path, ctype=None, cache_sec=0):
        """带 Range 的静态文件响应。视频回放全靠这个。"""
        try:
            size = os.path.getsize(path)
        except OSError:
            return self._err(404, 'not found')
        ctype = ctype or MIME.get(os.path.splitext(path)[1].lower(),
                                  'application/octet-stream')
        rng = self.headers.get('Range', '')
        start, end = 0, size - 1
        partial = False

        if rng.startswith('bytes='):
            spec = rng[6:].split(',')[0].strip()
            a, _, b = spec.partition('-')
            try:
                if a == '' and b != '':
                    # bytes=-N：最后 N 字节
                    n = int(b)
                    start = max(0, size - n)
                else:
                    start = int(a) if a else 0
                    if b:
                        end = min(int(b), size - 1)
            except ValueError:
                return self._err(416, 'bad range')
            if start > end or start >= size:
                self.send_response(416)
                self.send_header('Content-Range', f'bytes */{size}')
                self.send_header('Content-Length', '0')
                self.end_headers()
                return
            partial = True

        length = end - start + 1
        self.send_response(206 if partial else 200)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(length))
        # iOS 的 <video> 没有 Accept-Ranges 就不肯播
        self.send_header('Accept-Ranges', 'bytes')
        if partial:
            self.send_header('Content-Range', f'bytes {start}-{end}/{size}')
        self.send_header('Cache-Control',
                         f'public, max-age={cache_sec}' if cache_sec else 'no-cache')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        if self.command == 'HEAD':
            return
        try:
            with open(path, 'rb') as f:
                f.seek(start)
                left = length
                while left > 0:
                    chunk = f.read(min(65536, left))
                    if not chunk:
                        break
                    self.wfile.write(chunk)
                    left -= len(chunk)
        except (BrokenPipeError, ConnectionResetError):
            pass

    # ---- 路由 ----

    def do_OPTIONS(self):
        self._send(204)

    def do_HEAD(self):
        self.do_GET()

    def do_GET(self):
        try:
            self._route_get()
        except (BrokenPipeError, ConnectionResetError):
            pass
        except Exception as e:
            self.node.get_logger().error(f'GET {self.path} 异常：{e}')
            try:
                self._err(500, str(e))
            except Exception:
                pass

    def do_PUT(self):
        self._handle_write('PUT')

    def do_POST(self):
        self._handle_write('POST')

    def _route_get(self):
        parsed = urllib.parse.urlparse(self.path)
        path = urllib.parse.unquote(parsed.path)

        # 静态页（首页 / js / css）：不需要鉴权，否则手机第一次打开就是空白
        if not path.startswith('/api/'):
            return self._static(path)

        if path == '/api/v1/health':
            return self._json({'ok': True, 'device': self.node.device_name,
                               'uptime_s': round(time.time() - self.node.t0, 1)})

        if not self._authed():
            return self._err(401, 'unauthorized')

        if path == '/api/v1/status':
            return self._json(self.node.status())

        if path == '/api/v1/stream.mjpg':
            return self._stream()

        if path == '/api/v1/snapshot.jpg':
            jpeg, _, _ = self.node.hub.get()
            if jpeg is None:
                return self._err(503, 'no frame yet')
            return self._send(200, jpeg, 'image/jpeg')

        if path == '/api/v1/events':
            return self._json({'events': self.node._events()})

        if path == '/api/v1/alarms':
            return self._json({'alarms': self.node.alarms(),
                               'current': self.node.current_alert()})

        if path == '/api/v1/clips':
            return self._json({'clips': self.node._clips()})

        if path == '/api/v1/contacts':
            cs, err = self.node.load_contacts()
            body = {'contacts': cs}
            if err:
                body['warning'] = err
            return self._json(body)

        if path == '/api/v1/snapshots':
            items = []
            try:
                for name in sorted(os.listdir(self.node.snaps_dir), reverse=True):
                    if not name.lower().endswith('.jpg'):
                        continue
                    p = os.path.join(self.node.snaps_dir, name)
                    items.append({'file': name, 'bytes': os.path.getsize(p),
                                  'mtime': os.path.getmtime(p)})
            except FileNotFoundError:
                pass
            return self._json({'snapshots': items})

        if path.startswith('/api/v1/snapshots/'):
            name = path[len('/api/v1/snapshots/'):]
            if '/' in name or '\\' in name:
                return self._err(400, 'bad name')
            return self._file(os.path.join(self.node.snaps_dir, name), cache_sec=3600)

        if path.startswith('/api/v1/clips/'):
            name = path[len('/api/v1/clips/'):]
            if '/' in name or '\\' in name:
                return self._err(400, 'bad name')
            return self._file(os.path.join(self.node.clips_dir, name), cache_sec=3600)

        if path.startswith('/api/v1/events/'):
            rest = path[len('/api/v1/events/'):]
            parts = [x for x in rest.split('/') if x]
            d = self.node.event_dir(parts[0]) if parts else None
            if d is None:
                return self._err(404, 'event not found')
            if len(parts) == 1:
                meta_p = os.path.join(d, 'meta.json')
                if not os.path.isfile(meta_p):
                    return self._err(404, 'meta not found')
                with open(meta_p, 'r', encoding='utf-8') as f:
                    return self._json(json.load(f))
            if parts[1] in ('video', 'video.mp4'):
                return self._file(os.path.join(d, 'video.mp4'), 'video/mp4')
            if parts[1] in ('thumb', 'thumb.jpg'):
                return self._file(os.path.join(d, 'thumb.jpg'), cache_sec=3600)
            return self._err(404, 'unknown event sub-resource')

        return self._err(404, 'no such endpoint')

    def _static(self, path):
        root = self.node.web_dir
        if not root or not os.path.isdir(root):
            return self._send(500, 'web dir missing: %r' % root,
                              'text/plain; charset=utf-8')
        rel = 'index.html' if path in ('/', '') else path.lstrip('/')
        # 路径穿越防护：先做**词法**规整再判越界。
        #
        # 这里不能用 os.path.realpath 判越界：colcon 的 install(DIRECTORY) 是
        # 「目录真建、文件逐个做符号链接」，所以 web/index.html 的真实路径是
        # src/lightdog/web/index.html —— 按 realpath 比较会把正常的静态文件
        # 全判成越界（实测 400）。词法规整能正确挡掉 ../，符号链接交给 open()
        # 跟随即可（它本来就在我们自己的包目录里）。
        root_abs = os.path.abspath(root)
        full = os.path.normpath(os.path.join(root_abs, rel))
        if full != root_abs and not full.startswith(root_abs + os.sep):
            return self._err(400, 'bad path')
        if not os.path.isfile(full):
            return self._send(404, 'not found', 'text/plain; charset=utf-8')
        return self._file(full)

    def _handle_write(self, method):
        try:
            parsed = urllib.parse.urlparse(self.path)
            path = urllib.parse.unquote(parsed.path)
            n = int(self.headers.get('Content-Length') or 0)
            raw = self.rfile.read(n) if n > 0 else b''
            if not path.startswith('/api/'):
                return self._err(404, 'no such endpoint')
            if not self._authed():
                return self._err(401, 'unauthorized')

            if path == '/api/v1/events' and method == 'POST':
                return self._post_event(raw)

            if path.startswith('/api/v1/alarms/') and method == 'POST':
                return self._post_alarm_cmd(path[len('/api/v1/alarms/'):].strip('/'))

            if path == '/api/v1/contacts' and method in ('PUT', 'POST'):
                return self._put_contacts(raw)

            return self._err(404, 'no such endpoint')
        except (BrokenPipeError, ConnectionResetError):
            pass
        except Exception as e:
            self.node.get_logger().error(f'{method} {self.path} 异常：{e}')
            try:
                self._err(500, str(e))
            except Exception:
                pass

    def _post_event(self, raw):
        try:
            req = json.loads(raw or b'{}')
        except Exception:
            req = {}
        if not isinstance(req, dict):
            req = {}
        req.setdefault('type', 'manual')
        req.setdefault('label', '手动触发')
        # 交给 recorder 去做实际切片：后端不碰视频，只当触发器
        msg = String()
        msg.data = json.dumps(req, ensure_ascii=False)
        self.node.pub_event.publish(msg)
        self.node.get_logger().info(f'HTTP 触发事件：{msg.data}')
        return self._json({'ok': True, 'queued': req}, 202)

    def _post_alarm_cmd(self, action):
        """手机端对告警的三个动作：确认已知晓 / 测试 / 复位。

        走 ROS 话题转给 `lightdog_alert_manager`，而不是让后端直接改状态 ——
        状态机的唯一所有者必须是那个节点，否则两边各记一份迟早对不上。
        """
        mapping = {'ack': 'ack', 'test': 'test', 'reset': 'reset'}
        cmd = mapping.get(action)
        if cmd is None:
            return self._err(400, 'unknown alarm action (ack|test|reset)')
        m = String()
        m.data = cmd
        self.node.pub_alert_cmd.publish(m)
        self.node.get_logger().info(f'HTTP 下发告警命令：{cmd}')
        return self._json({'ok': True, 'command': cmd}, 202)

    def _put_contacts(self, raw):
        try:
            data = json.loads(raw or b'{}')
        except Exception:
            return self._err(400, 'bad json')
        cs = data.get('contacts') if isinstance(data, dict) else data
        if not isinstance(cs, list):
            return self._err(400, 'contacts must be a list')
        clean = []
        for c in cs[:16]:            # 上限 16 个，防止被灌爆
            if not isinstance(c, dict):
                continue
            name = str(c.get('name', '')).strip()[:32]
            phone = str(c.get('phone', '')).strip()[:32]
            if not phone:
                continue
            clean.append({'name': name or '紧急联系人', 'phone': phone,
                          'primary': bool(c.get('primary', not clean))})
        try:
            self.node.save_contacts(clean)
        except Exception as e:
            return self._err(500, f'save failed: {e}')
        self.node.get_logger().info(f'联系人已更新：{len(clean)} 个')
        return self._json({'ok': True, 'contacts': clean})

    # ---- MJPEG ----

    def _stream(self):
        """multipart/x-mixed-replace 推流。

        帧率上限是 `stream_fps`，但真正决定节奏的是相机：1.5Hz 的相机
        在 10Hz 的推流里只会重复 7 次同一张图，所以这里用 `wait_newer` 等新帧，
        有新帧才发 —— 既不浪费带宽，客户端收到的也永远是最新画面。
        """
        boundary = 'lightdogframe'
        # 流没有 Content-Length，必须显式关连接，否则 HTTP/1.1 的 keep-alive
        # 会让 serve_forever 卡在这个请求上等下一条报文
        self.close_connection = True
        self.send_response(200)
        self.send_header('Content-Type',
                         f'multipart/x-mixed-replace; boundary={boundary}')
        self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
        self.send_header('Pragma', 'no-cache')
        self.send_header('Connection', 'close')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()

        last_seq = -1
        min_dt = 1.0 / self.node.stream_fps
        last_t = 0.0
        try:
            while True:
                jpeg, seq, _ = self.node.hub.wait_newer(last_seq, timeout=2.0)
                if jpeg is None:
                    time.sleep(0.2)
                    continue
                if seq == last_seq:
                    # 相机静默：继续等，靠客户端断开触发 BrokenPipe 退出
                    continue
                dt = time.time() - last_t
                if dt < min_dt:
                    time.sleep(min_dt - dt)
                head = (f'--{boundary}\r\n'
                        f'Content-Type: image/jpeg\r\n'
                        f'Content-Length: {len(jpeg)}\r\n\r\n').encode('ascii')
                self.wfile.write(head)
                self.wfile.write(jpeg)
                self.wfile.write(b'\r\n')
                self.wfile.flush()
                last_seq = seq
                last_t = time.time()
        except (BrokenPipeError, ConnectionResetError, socket.timeout, OSError):
            return


def main(args=None):
    rclpy.init(args=args)
    node = LightDogServer()

    httpd = ThreadingHTTPServer((node.host, node.port), Handler)
    httpd.daemon_threads = True
    httpd.node = node

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    node.get_logger().info(f'监听 {node.host}:{node.port}')
    if node.token:
        node.get_logger().info(
            f'手机访问：http://{node._lan_ip()}:{node.port}/?token={node.token}')
    else:
        node.get_logger().warn(
            f'未设置 auth_token —— 同一网络下任何人都能看画面。'
            f'仿真无妨，真机务必设置。手机访问：http://{node._lan_ip()}:{node.port}/')

    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        httpd.shutdown()
        httpd.server_close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
