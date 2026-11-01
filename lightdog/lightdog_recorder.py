#!/usr/bin/env python3
"""
LightDog P4：摄像头录像（环形分段 + 事件切片）

## 这个节点负责什么

    订阅 /camera/color/image_raw   sensor_msgs/Image
    订阅 /recorder/event           std_msgs/String（JSON，事件类型 + pre/post 秒数）
    订阅 /recorder/snapshot        std_msgs/Empty
    发布 /recorder/state           std_msgs/String（JSON，1Hz）

    落盘 <data_dir>/clips/seg_XXXXXX.mp4    环形分段，滚掉最老的
         <data_dir>/clips.jsonl              分段索引（append-only，一行一段）
         <data_dir>/events/<id>/video.mp4    事件切片（含事发前 pre 秒）
         <data_dir>/events/<id>/meta.json
         <data_dir>/events/<id>/thumb.jpg
         <data_dir>/snapshots/<ts>.jpg

## 三个关键设计

**1. 分段写盘，而不是一路录到底。**
SD 卡掉电、进程被杀、卡写满，都只毁掉当前正在写的这一段（默认 10 秒），
不会毁掉整段录像。这是行车记录仪的标准做法，也是它能长期跑下去的原因。

**2. 事件视频必须带「事发前」。**
看护场景里真正有用的是「老人是怎么倒下的」，而不是「倒下之后躺在地上」。
所以环形缓冲是必需品：事件触发时往前捞 `pre` 秒。这也是本节点存在的最大理由。

**3. 帧率按实测写，不能写死。**
仿真相机是 640x480 软渲染，实测只有 1.5 Hz（真机 30 Hz）。如果按 30 写头，
回放会变成 20 倍速的幻灯片；按段内 `帧数 / 实际时长` 写，回放速度才和真实一致。

## 事件触发方式

`ros2 topic pub --once /recorder/event std_msgs/msg/String \
   '{data: "{\"type\":\"fall\",\"label\":\"检测到跌倒\"}"}'`

`pre` / `post` 不填就用参数默认值。P5 的跌倒检测直接往这个话题发就行，
不需要知道录像的任何内部细节 —— 这层解耦是刻意的。

## 编码

容器里没有 ffmpeg 命令行，但 OpenCV 4.6 自带 FFMPEG 后端，`avc1`（H.264）可用。
选 avc1 而不是 mp4v 的唯一理由是**手机**：iOS Safari 不支持 MPEG-4 Part 2，
只认 H.264。打不开时自动降级到 mp4v，并打日志说明。
"""

import json
import os
import queue
import sys
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from std_msgs.msg import Empty, String


# ---------- 图像解码（与 person_detector 一致：不依赖 cv_bridge） ----------

def image_to_bgr(msg: Image):
    """sensor_msgs/Image -> (H, W, 3) uint8 BGR；无法识别返回 None。

    cv2.VideoWriter 吃 BGR，所以这里一步到位转成 BGR，省掉下游再翻一次。
    """
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
        g = buf[:h * w].reshape(h, w)
        return cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)
    return None


def even(n: int) -> int:
    """H.264 / yuv420p 要求宽高为偶数，奇数裁 1 像素。"""
    return n - (n % 2)


class Recorder(Node):

    def __init__(self):
        super().__init__('lightdog_recorder')

        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('data_dir', '/data')
        self.declare_parameter('segment_sec', 10.0)      # 单段时长（ROS 秒 = 画面秒数）
        self.declare_parameter('retain_sec', 1800.0)     # 环形保留时长（**墙钟**秒）
        self.declare_parameter('min_segments', 6)        # 至少保留几段，防止 retain 太小
        self.declare_parameter('max_disk_mb', 512.0)     # 磁盘配额（只算 clips/）
        self.declare_parameter('pre_sec', 10.0)          # 事件默认前置
        self.declare_parameter('post_sec', 10.0)         # 事件默认后置
        self.declare_parameter('max_event_frames', 900)  # 单个事件视频最多几帧
        self.declare_parameter('codec', 'avc1')
        self.declare_parameter('jpeg_quality', 80)
        self.declare_parameter('recording', True)
        self.declare_parameter('state_topic', '/recorder/state')

        p = self.get_parameter
        self.image_topic = str(p('image_topic').value)
        self.data_dir = str(p('data_dir').value)
        self.segment_sec = float(p('segment_sec').value)
        self.retain_sec = float(p('retain_sec').value)
        self.min_segments = int(p('min_segments').value)
        self.max_disk_mb = float(p('max_disk_mb').value)
        self.pre_sec = float(p('pre_sec').value)
        self.post_sec = float(p('post_sec').value)
        self.max_event_frames = int(p('max_event_frames').value)
        self.codec = str(p('codec').value)
        self.jpeg_quality = int(p('jpeg_quality').value)
        self.recording = bool(p('recording').value)

        self.clips_dir = os.path.join(self.data_dir, 'clips')
        self.events_dir = os.path.join(self.data_dir, 'events')
        self.snaps_dir = os.path.join(self.data_dir, 'snapshots')
        self.index_path = os.path.join(self.data_dir, 'clips.jsonl')
        for d in (self.clips_dir, self.events_dir, self.snaps_dir):
            os.makedirs(d, exist_ok=True)

        # 段索引：list[{file, seq, t0, t1, wall, frames, fps, bytes, session}]
        self.clips = []
        self.seq = 0
        # 会话标识 = 本节点启动那一刻的墙钟秒。
        #
        # 为什么需要它：分段的时间戳用的是 ROS 时间，而 ROS 时间**不保证单调跨重启**。
        # 仿真里每次重启 gazebo，/clock 都从 0 开始；真机上 NTP 校时、或者底盘
        # 供电重启后时钟回退，也会出现同样的事。实测踩到：容器重启后 sim time
        # 从 92 掉回 7，事件窗口 [36,56] 把重启前录的 seg_000000(t0=37) 也匹配进去了，
        # 结果一个 20 秒的事件剪出 621 帧、混杂两次运行的画面。
        #
        # 所以事件切片只在**同一会话**内取段：跨重启的画面不拼进同一个事件。
        self.session = int(time.time())
        self._load_index()

        # 写入串行化：图像回调只负责攒帧，编码在单独线程里做，
        # 否则真机 30Hz 下编码会把回调堵死、直接丢帧。
        self._flush_q = queue.Queue()
        self._flush_thread = threading.Thread(target=self._flush_worker, daemon=True)
        self._flush_thread.start()

        self._lock = threading.Lock()
        self.cur = []                # [(ros_t, bgr), ...] 当前段的帧
        self.cur_t0 = None
        self.cur_t1 = None
        self.latest = None           # 最近一帧（给快照用）
        self.latest_t = None
        self.n_frames = 0
        self.pending = []            # 待合成事件
        self.n_events = 0
        self.warned_codec = False

        sensor_qos = QoSProfile(depth=1,
                                reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Image, self.image_topic, self._on_image, sensor_qos)
        self.create_subscription(String, '/recorder/event', self._on_event, 10)
        self.create_subscription(Empty, '/recorder/snapshot', self._on_snapshot, 10)

        self.pub_state = self.create_publisher(String, str(p('state_topic').value), 5)

        self.create_timer(0.5, self._tick)

        self.get_logger().info(
            f'LightDog recorder ready: {self.image_topic} -> {self.clips_dir} '
            f'segment={self.segment_sec:.0f}s retain={self.retain_sec:.0f}s '
            f'quota={self.max_disk_mb:.0f}MB codec={self.codec} '
            f'recording={self.recording} next_seq={self.seq}')

    # ---------- 索引 ----------

    def _load_index(self):
        """从 clips.jsonl 恢复索引，并从现存文件推出下一个 seq。

        节点重启是常态（改参数、崩溃、上电），seq 从 0 重来会覆盖旧文件，
        所以起步 seq 取「索引里的最大值」和「目录里实际的最大值」的较大者。
        """
        if os.path.exists(self.index_path):
            try:
                with open(self.index_path, 'r', encoding='utf-8') as f:
                    for line in f:
                        line = line.strip()
                        if not line:
                            continue
                        try:
                            rec = json.loads(line)
                        except Exception:
                            continue        # 半行（写到一半断电）直接丢
                        if not os.path.exists(os.path.join(self.clips_dir, rec.get('file', ''))):
                            continue
                        self.clips.append(rec)
                        self.seq = max(self.seq, int(rec.get('seq', -1)) + 1)
            except Exception as e:
                self.get_logger().warn(f'读取 {self.index_path} 失败：{e}')

        try:
            for name in os.listdir(self.clips_dir):
                if not (name.startswith('seg_') and name.endswith('.mp4')):
                    continue
                try:
                    s = int(name[4:10])
                except Exception:
                    continue
                self.seq = max(self.seq, s + 1)
        except Exception:
            pass

        if self.clips:
            self.get_logger().info(
                f'已恢复 {len(self.clips)} 个历史分段，下一个 seq={self.seq}')

    def _append_index(self, rec):
        self.clips.append(rec)
        try:
            with open(self.index_path, 'a', encoding='utf-8') as f:
                f.write(json.dumps(rec, ensure_ascii=False) + '\n')
        except Exception as e:
            self.get_logger().warn(f'写索引失败：{e}')

    # ---------- 帧回调 ----------

    def _stamp_sec(self, msg) -> float:
        """优先用 header.stamp（与相机同步），stamp 为 0 时退回节点时钟。"""
        s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if s <= 0.0:
            return self.get_clock().now().nanoseconds * 1e-9
        return s

    def _on_image(self, msg: Image):
        img = image_to_bgr(msg)
        if img is None:
            return
        t = self._stamp_sec(msg)
        self.latest = img
        self.latest_t = t
        self.n_frames += 1

        if not self.recording:
            return

        with self._lock:
            if self.cur_t0 is None:
                self.cur_t0 = t
            self.cur.append((t, img))
            self.cur_t1 = t
            ready = (t - self.cur_t0) >= self.segment_sec

        if ready:
            self._flush_segment()

    # ---------- 分段写盘 ----------

    def _flush_segment(self):
        """把当前累积的帧切成一段交给写盘线程。空段直接跳过。"""
        with self._lock:
            if not self.cur:
                self.cur_t0 = None
                return
            frames = self.cur
            t0, t1 = self.cur_t0, self.cur_t1
            self.cur = []
            self.cur_t0 = None
            self.cur_t1 = None
            seq = self.seq
            self.seq += 1

        self._flush_q.put({
            'seq': seq, 'frames': frames, 't0': t0, 't1': t1,
            'wall': time.time(), 'codec': self.codec,
        })

    def _flush_worker(self):
        while True:
            job = self._flush_q.get()
            if job is None:
                self._flush_q.task_done()
                return
            try:
                self._write_segment(job)
            except Exception as e:
                self.get_logger().error(f'写分段 {job["seq"]} 失败：{e}')
            finally:
                self._flush_q.task_done()

    def _write_segment(self, job):
        frames = job['frames']
        seq, t0, t1 = job['seq'], job['t0'], job['t1']
        dur = max(1e-3, t1 - t0 + max(1e-3, (t1 - t0) / max(1, len(frames) - 1)))
        n = len(frames)

        # 帧率按实测写：软渲染 1.5Hz / 真机 30Hz 都能回放出正确速度。
        # 上下限只是防御（一帧的段会让 fps 变成无穷大）。
        fps = n / dur if dur > 0 else 1.0
        fps = max(0.5, min(60.0, fps))

        h = even(int(frames[0][1].shape[0]))
        w = even(int(frames[0][1].shape[1]))
        if h < 2 or w < 2:
            return

        name = 'seg_%06d.mp4' % seq
        path = os.path.join(self.clips_dir, name)
        # 先写 .part.mp4 再 rename：中途断电不会留下"看起来完整、其实写了一半"的文件。
        # 注意扩展名必须是 .mp4 —— cv2.VideoWriter 靠**后缀**选 muxer，
        # 叫 xxx.mp4.part 时它会找不到格式，直接打不开编码器（踩过）。
        tmp = os.path.join(self.clips_dir, 'seg_%06d.part.mp4' % seq)

        writer, used = self._open_writer(tmp, fps, (w, h))
        if writer is None:
            return
        for _, f in frames:
            writer.write(self._fit(f, w, h))
        writer.release()

        os.replace(tmp, path)
        size = os.path.getsize(path)
        rec = {'file': name, 'seq': seq,
               't0': round(t0, 3), 't1': round(t1, 3),
               'wall': round(job['wall'], 3),
               'frames': n, 'fps': round(fps, 3), 'bytes': size,
               'codec': used, 'session': self.session}
        self._append_index(rec)
        self.get_logger().debug(
            f'{name}: {n} 帧 / {dur:.1f}s / {fps:.2f}fps / {size/1024:.0f}KB ({used})')

    def _open_writer(self, path, fps, size):
        """优先 H.264；不可用则降级 mp4v（并只警告一次）。"""
        for tag in (self.codec, 'mp4v'):
            if not tag:
                continue
            try:
                w = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*tag), float(fps), size)
                if w.isOpened():
                    return w, tag
                w.release()
            except Exception:
                pass
        if not self.warned_codec:
            self.warned_codec = True
            self.get_logger().error(
                f'编码器 {self.codec}/mp4v 都打不开，录像不可用')
        return None, None

    @staticmethod
    def _fit(frame, w, h):
        if frame.shape[0] == h and frame.shape[1] == w:
            return frame
        return frame[:h, :w]

    # ---------- 事件 ----------

    def _now_ros(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_event(self, msg: String):
        try:
            req = json.loads(msg.data) if msg.data.strip() else {}
        except Exception as e:
            self.get_logger().warn(f'/recorder/event 不是合法 JSON（{e}）：{msg.data!r}')
            return

        etype = str(req.get('type', 'event'))
        label = str(req.get('label', etype))
        pre = float(req.get('pre', self.pre_sec))
        post = float(req.get('post', self.post_sec))
        note = str(req.get('note', ''))

        t_ev = self._now_ros()
        # 先把当前段落盘：这样 pre 窗口的尾部不会卡在内存里等段满
        self._flush_segment()

        ctx = {
            'type': etype, 'label': label, 'note': note,
            't_ev': t_ev, 'pre': pre, 'post': post,
            't_from': t_ev - pre, 't_to': t_ev + post,
            'wall': time.time(),
        }
        self.pending.append(ctx)
        self.get_logger().info(
            f'事件 {etype}（{label}）已受理：{t_ev:.1f}s，窗口 -{pre:.0f}s/+{post:.0f}s')

    def _on_snapshot(self, _msg: Empty):
        if self.latest is None:
            self.get_logger().warn('快照失败：还没有收到图像')
            return
        name = '%s.jpg' % time.strftime('%Y%m%d_%H%M%S')
        path = os.path.join(self.snaps_dir, name)
        ok, buf = cv2.imencode('.jpg', self.latest,
                               [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        if ok:
            with open(path, 'wb') as f:
                f.write(buf.tobytes())
            self.get_logger().info(f'快照 {path}（{len(buf)}B）')
        else:
            self.get_logger().warn('快照编码失败')

    def _tick(self):
        now = self._now_ros()

        # 事件后置时间到 -> 合成
        due = [c for c in self.pending if now >= c['t_to']]
        if due:
            self.pending = [c for c in self.pending if now < c['t_to']]
            self._flush_segment()      # 后置窗口的帧也要落盘
            for c in due:
                threading.Thread(target=self._assemble_event, args=(c,),
                                 daemon=True).start()

        self._gc()
        self._publish_state(now)

    # ---------- 事件合成 ----------

    def _assemble_event(self, ctx):
        """把所有相关分段读回来，裁出窗口内的帧，重新编码成一个事件视频。

        不在事件当下就合成，是因为后置窗口的帧还没产生；等到 t_to 之后再动手，
        用的是「已落盘的完整分段」，不需要额外在内存里留一份长缓冲。
        """
        # 等写盘队列排空（后置窗口刚 flush 的那段可能还在编码）
        self._flush_q.join()

        with self._lock:
            clips = list(self.clips)
        # 只取本会话的段：ROS 时间在重启后会回零，跨会话比较 t0/t1 是没有意义的
        # （见 __init__ 里 session 的注释）。
        hits = [c for c in clips
                if c.get('session') == self.session
                and c['t1'] >= ctx['t_from'] and c['t0'] <= ctx['t_to']]
        hits.sort(key=lambda c: c['seq'])

        frames = []
        for c in hits:
            frames.extend(self._read_clip_frames(c, ctx['t_from'], ctx['t_to']))
            if len(frames) >= self.max_event_frames:
                break
        if len(frames) > self.max_event_frames:
            frames = frames[:self.max_event_frames]

        # 事件 id 用墙钟时间戳，人能一眼看出「几点几分」；同秒多条时加序号
        base = time.strftime('%Y%m%d_%H%M%S', time.localtime(ctx['wall']))
        eid = 'evt_%s_%s' % (base, ctx['type'])
        k = 1
        while os.path.exists(os.path.join(self.events_dir, eid)):
            eid = 'evt_%s_%s_%d' % (base, ctx['type'], k)
            k += 1
        self.n_events += 1
        out_dir = os.path.join(self.events_dir, eid)
        os.makedirs(out_dir, exist_ok=True)

        meta = {
            'id': eid, 'type': ctx['type'], 'label': ctx['label'],
            'note': ctx['note'], 'wall': ctx['wall'],
            't_ev': round(ctx['t_ev'], 3),
            't_from': round(ctx['t_from'], 3), 't_to': round(ctx['t_to'], 3),
            'clips': [c['file'] for c in hits],
            'frames': len(frames), 'has_video': False,
        }

        if not frames:
            self.get_logger().warn(
                f'{eid}: 窗口内没有可用帧（分段可能已被环形清理）')
            meta['error'] = 'no_frames'
            self._write_meta(out_dir, meta)
            return

        # 事件视频的 fps：窗口内总帧数 / 窗口实际跨度的帧引用数
        span = max(1e-3, frames[-1][0] - frames[0][0])
        fps = len(frames) / span if span > 0.5 else 1.0
        fps = max(0.5, min(60.0, fps))

        h = even(int(frames[0][1].shape[0]))
        w = even(int(frames[0][1].shape[1]))
        path = os.path.join(out_dir, 'video.mp4')
        tmp = os.path.join(out_dir, 'video.part.mp4')   # 扩展名见 _write_segment 的注释
        writer, used = self._open_writer(tmp, fps, (w, h))
        if writer is None:
            meta['error'] = 'encoder_unavailable'
            self._write_meta(out_dir, meta)
            return
        for _, f in frames:
            writer.write(self._fit(f, w, h))
        writer.release()
        os.replace(tmp, path)

        # 缩略图取最接近事发时刻的那一帧
        thumb = min(frames, key=lambda it: abs(it[0] - ctx['t_ev']))[1]
        ok, buf = cv2.imencode('.jpg', thumb,
                               [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        if ok:
            with open(os.path.join(out_dir, 'thumb.jpg'), 'wb') as f:
                f.write(buf.tobytes())

        meta.update({'has_video': True, 'fps': round(fps, 3),
                     'bytes': os.path.getsize(path), 'codec': used})
        self._write_meta(out_dir, meta)
        self.get_logger().info(
            f'{eid}: {len(frames)} 帧 / {fps:.2f}fps / '
            f'{os.path.getsize(path)/1024:.0f}KB，来源 {len(hits)} 段'
            f'（{ctx["label"]}）')

    def _read_clip_frames(self, clip, t_from, t_to):
        """读一个分段，按线性插值给每帧估时间戳并裁窗口。

        mp4 里没有逐帧时间戳（至少 cv2 读不到），但一段内的帧是等间隔写进去的，
        用 (t1-t0)/N 插值足够定位「事发前 N 秒」这个精度。
        """
        path = self._clip_path(clip)
        if not os.path.exists(path):
            return []
        cap = cv2.VideoCapture(path)
        if not cap.isOpened():
            return []
        n = int(clip.get('frames') or 0)
        t0, t1 = clip['t0'], clip['t1']
        dt = (t1 - t0) / max(1, n - 1) if n > 1 else 0.0
        out = []
        i = 0
        while True:
            ok, img = cap.read()
            if not ok:
                break
            t = t0 + i * dt if dt > 0 else t0
            if t_from <= t <= t_to:
                out.append((t, img))
            i += 1
            if t > t_to:
                break
        cap.release()
        return out

    def _write_meta(self, out_dir, meta):
        tmp = os.path.join(out_dir, 'meta.json.part')
        try:
            with open(tmp, 'w', encoding='utf-8') as f:
                json.dump(meta, f, ensure_ascii=False, indent=2)
            os.replace(tmp, os.path.join(out_dir, 'meta.json'))
        except Exception as e:
            self.get_logger().warn(f'写 {out_dir}/meta.json 失败：{e}')

    # ---------- 清理 ----------

    def _gc(self):
        """两层清理：按时间保留（retain_sec）和按磁盘配额（max_disk_mb）。

        时间维度用**墙钟**，不用 ROS 时间：ROS 时间在重启后会回零，用它算
        "这段多久以前录的"会得到负数，结果重启前的录像永远不被清理（实测踩到）。
        墙钟是单调向前的，跨重启也成立。真机上两者等价。
        """
        if not self.clips:
            return
        now = time.time()
        removed = 0

        # 1) 时间维度
        keep = []
        for c in self.clips:
            old = (now - c.get('wall', now)) > self.retain_sec
            if old and len(self.clips) - removed > self.min_segments:
                self._drop_clip(c)
                removed += 1
            else:
                keep.append(c)
        self.clips = keep

        # 2) 容量维度
        total = sum(c['bytes'] for c in self.clips)
        while total > self.max_disk_mb * 1024 * 1024 and len(self.clips) > 1:
            c = self.clips.pop(0)
            total -= c['bytes']
            self._drop_clip(c)
            removed += 1

        if removed:
            self._rewrite_index()

    def _drop_clip(self, clip):
        p = self._clip_path(clip)
        try:
            os.remove(p)
        except FileNotFoundError:
            pass
        except Exception as e:
            self.get_logger().warn(f'删除 {p} 失败：{e}')

    def _clip_path(self, clip):
        return os.path.join(self.clips_dir, clip['file'])

    def _rewrite_index(self):
        """索引是 append-only 的，清理后重写成一个小文件，避免无限增长。"""
        tmp = self.index_path + '.part'
        try:
            with open(tmp, 'w', encoding='utf-8') as f:
                for c in self.clips:
                    f.write(json.dumps(c, ensure_ascii=False) + '\n')
            os.replace(tmp, self.index_path)
        except Exception as e:
            self.get_logger().warn(f'重写索引失败：{e}')

    # ---------- 状态 ----------

    def _publish_state(self, now):
        with self._lock:
            cur_n = len(self.cur)
            cur_t0 = self.cur_t0
        clips = list(self.clips)
        total = sum(c['bytes'] for c in clips)
        last_fps = clips[-1]['fps'] if clips else 0.0
        age = (now - self.latest_t) if self.latest_t is not None else -1.0

        st = {
            'recording': self.recording,
            'segments': len(clips),
            'total_bytes': total,
            'total_mb': round(total / 1048576.0, 2),
            'last_fps': last_fps,
            'current_frames': cur_n,
            'current_span': round(now - cur_t0, 2) if cur_t0 is not None else 0.0,
            'frames_seen': self.n_frames,
            'events': self.n_events,
            'pending_events': len(self.pending),
            'frame_age': round(age, 2),
            'latest_file': clips[-1]['file'] if clips else '',
            'latest_t': clips[-1]['t1'] if clips else 0.0,
        }
        msg = String()
        msg.data = json.dumps(st, ensure_ascii=False)
        self.pub_state.publish(msg)

    def destroy_node(self):
        try:
            self._flush_segment()
            self._flush_q.put(None)
            self._flush_thread.join(timeout=5.0)
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Recorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
