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
import shutil
import subprocess
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


def safe_token(text, default='event', max_len=24) -> str:
    """把外部输入收敛成可以安全拼进文件名的片段。

    `type` 来自 HTTP 请求体（`POST /api/v1/events`）或任意 ROS 话题发布者，
    会被直接拼成 `evt_<时间>_<type>` 目录名。不净化的话 `type="../../x"`
    能把 `os.makedirs` 的目标指到 events/ 之外，`type="a/b"` 会在中间多建一级
    目录 —— 后者在真机上表现为「事件在列表里消失但磁盘在涨」。
    只留 ASCII 字母数字与 `-_`，其余全部丢掉。
    """
    s = ''.join(ch for ch in str(text)
                if ch.isascii() and (ch.isalnum() or ch in ('-', '_')))
    s = s.strip('-_')[:max_len]
    return s or default


def clamp_sec(value, low=0.0, high=300.0, default=10.0) -> float:
    """把事件窗口参数夹到合理范围；坏值退回默认值。

    不夹的话 `pre=1e9` 会让窗口覆盖整个环形缓冲（以及已经被清掉的历史），
    合成线程要扫一遍所有分段才失败。
    """
    try:
        v = float(value)
    except (TypeError, ValueError):
        return default
    if not (v == v):            # NaN
        return default
    return max(low, min(high, v))


def faststart_remux(path, log=None):
    """把 moov 原子挪到文件头（需要 ffmpeg）。成功返回 True。

    只做 `-c copy`：不重编码，画质零损失，几毫秒到几十毫秒。
    没有 ffmpeg、超时、或者输出异常一律返回 False —— 原文件保持不动，
    因为「起播慢一点」远好过「为了优化把录像搞坏」。
    """
    exe = shutil.which('ffmpeg')
    if not exe:
        if log is not None:
            log.warn('faststart 已开启但镜像里没有 ffmpeg，跳过重封装')
        return False
    tmp = path + '.fs.mp4'
    ok = False
    try:
        r = subprocess.run(
            [exe, '-hide_banner', '-loglevel', 'error', '-y',
             '-i', path, '-c', 'copy', '-movflags', '+faststart', tmp],
            capture_output=True, timeout=60.0)
        if r.returncode == 0 and os.path.getsize(tmp) > 1024:
            os.replace(tmp, path)
            ok = True
        elif log is not None:
            err = (r.stderr or b'').decode('utf-8', 'replace').strip()
            log.warn(f'faststart 重封装失败（保留原文件）：{err[:160]}')
    except (OSError, subprocess.SubprocessError) as e:
        if log is not None:
            log.warn(f'faststart 重封装异常（保留原文件）：{e}')
    finally:
        try:
            if os.path.exists(tmp):
                os.unlink(tmp)
        except OSError:
            pass
    return ok


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
        # 单个事件视频最多几帧。真机 30fps 时 300 帧 = 10s；
        # 原来给的 900 帧意味着事件合成时一次把 900 张 640x480x3 解进 list，
        # 约 830MB，直接在 ARM 板上 OOM。
        # 事件视频的帧数上限。**必须 >= (pre+post) × 相机帧率**，否则
        # _assemble_event 是从窗口头部往后写的，撞上限后丢的是窗口尾巴 ——
        # 对跌倒告警来说正好丢掉「倒下之后」那段，留下的是人还站着的前半段。
        # P5 的窗口是 25+10=35s，真机 30Hz 相机要 1050 帧，取 1200 留余量。
        # 流式写盘只占一帧内存，所以放宽上限不会再回到当初 OOM 的老路。
        self.declare_parameter('max_event_frames', 1200)
        # 事件目录的保留策略。环形缓冲有配额，events/ 之前一没有 —— 一次跌倒
        # 35s 视频在真机 30fps 下约 2~3MB，一天几次，一个月就是几百 MB。
        # 卡写满后录像/事件/告警全部失败（ENOSPC），而且往往是半夜才发生。
        self.declare_parameter('event_retain_days', 30.0)
        self.declare_parameter('max_event_mb', 512.0)
        self.declare_parameter('min_events', 20)      # 至少留这么多条，供最近的取证
        self.declare_parameter('max_snapshots', 200)  # 快照目录同样要封顶
        self.declare_parameter('codec', 'avc1')
        # 事件视频的 moov 在文件尾（cv2 的写入顺序），iOS/Safari 起播要多一次
        # 往返。打开这个开关会在合成后用 ffmpeg 做一次 `-c copy` 重封装，
        # 把 moov 挪到文件头。**需要镜像里有 ffmpeg**；没有就静默跳过。
        # 默认关：OpenCV 用 avc1 标签写 MPEG-4 Part 2 是常见怪癖，重封装后
        # 仍要看播放器怎么解释，真机开之前先用手机实播一遍。
        self.declare_parameter('faststart', False)
        self.declare_parameter('jpeg_quality', 80)
        # 环形缓冲的中间编码质量。这里**不是**最终画质（写盘时会重编一次），
        # 只是为了把常驻内存从 900KB/帧 压到 40KB/帧。给 92 留出二次编码余量。
        self.declare_parameter('buffer_jpeg_quality', 92)
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
        self.event_retain_days = float(p('event_retain_days').value)
        self.max_event_mb = float(p('max_event_mb').value)
        self.min_events = int(p('min_events').value)
        self.max_snapshots = int(p('max_snapshots').value)
        self.codec = str(p('codec').value)
        self.faststart = bool(p('faststart').value)
        self.jpeg_quality = int(p('jpeg_quality').value)
        self.buffer_quality = int(p('buffer_jpeg_quality').value)
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

        self._lock = threading.Lock()
        # clips / clips.jsonl 的专属锁。刻意与 self._lock（保护 self.cur 帧缓冲）
        # 分开：self._lock 在图像回调里 30Hz 抢占，绝不能被「追加索引 + 写文件 /
        # 清理时逐个 os.remove」这种慢 I/O 长时间占住，否则回调排队丢帧。
        # 三个线程共享 self.clips：写盘线程 append、定时器线程 _gc 增删、
        # 事件合成线程与状态发布读取——全部走这把锁互斥（P-001）。
        # 必须在下面启动写盘线程之前建好：worker 一被唤醒就可能调 _append_index。
        self._clips_lock = threading.Lock()

        # 写入串行化：图像回调只负责攒帧，编码在单独线程里做，
        # 否则真机 30Hz 下编码会把回调堵死、直接丢帧。
        #
        # 队列**必须有界**：写一段 = 解码几百帧 + 重新编码，SD 卡上比采集慢
        # 是常态。无界队列会让内存按段线性上涨（一段 JPEG 约 12MB），几十分钟
        # 后 OOM —— server 的 FrameHub 早就用「单槽信箱，跟不上就丢最旧」解决过
        # 同一个问题，这里同理。6 段约一分钟缓冲，够吸收抖动；
        # 真跟不上就丢最老的段（环形录像本来就更在意「最近」）。
        self._flush_q = queue.Queue(maxsize=6)
        self._dropped_segments = 0
        self._flush_thread = threading.Thread(target=self._flush_worker, daemon=True)
        self._flush_thread.start()

        # [(ros_t, jpeg_bytes), ...] 当前段的帧。
        #
        # 刻意存**编码后的 JPEG** 而不是原始 BGR：640x480x3 = 900KB/帧，
        # 真机 30fps × 10s = 270MB，写盘线程一堆积就翻倍 —— 而仿真相机只有
        # 1.5Hz，这个数字完全看不出来。JPEG 约 40KB/帧，同样一段约 12MB。
        self.cur = []
        self.cur_t0 = None
        self.cur_t1 = None
        self.latest = None           # 最近一帧（给快照用）
        self.latest_t = None
        self.n_frames = 0
        self.pending = []            # 待合成事件
        # 每个待合成事件都会起一个线程，且线程内要 join 写盘队列、开
        # VideoCapture / VideoWriter。有人持续往 /recorder/event 灌消息时，
        # 无界起线程会把 fd / 内存打满。并发上限 2，超出的留到下一拍。
        self.MAX_PENDING = 32
        self._assemble_sem = threading.BoundedSemaphore(2)
        self.n_events = 0
        self.warned_codec = False
        self._last_dir_gc = 0.0      # events/ snapshots/ 清理的节流时刻（墙钟）

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
                        # 旧版本写下的索引可能缺字段（bytes 是后加的）。
                        # 不补齐的话 _gc 里 sum(c['bytes']) 直接 KeyError，
                        # 清理线程一崩，卡就一直写满到 ENOSPC。
                        rec = self._normalize_clip(rec)
                        if rec is None:
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

    def _normalize_clip(self, rec):
        """补齐旧索引缺的字段；结构不可用（没有 t0/t1/file）返回 None。

        `bytes` 缺失时用文件实际大小补，不写 0：写 0 会让磁盘配额统计永久偏小，
        配额就永远触发不了，卡慢慢写满。
        """
        if not isinstance(rec, dict) or not rec.get('file'):
            return None
        if self._clip_path(rec) is None:
            return None
        for k in ('t0', 't1'):
            if k not in rec:
                return None
        try:
            rec['t0'] = float(rec['t0'])
            rec['t1'] = float(rec['t1'])
        except (TypeError, ValueError):
            return None
        if 'bytes' not in rec or not isinstance(rec.get('bytes'), int):
            try:
                rec['bytes'] = os.path.getsize(self._clip_path(rec))
            except OSError:
                rec['bytes'] = 0
        rec.setdefault('frames', 0)
        rec.setdefault('fps', 0.0)
        rec.setdefault('session', -1)
        return rec

    def _append_index(self, rec):
        # 写盘线程调用。self.clips 与 clips.jsonl 一起在这把锁下更新：
        # 与定时器线程的 _gc（会原地增删 self.clips 并重写 clips.jsonl）互斥，
        # 否则「append 到一半被 _rewrite_index 覆盖」会让索引丢行或错位。
        with self._clips_lock:
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
        self.latest = img            # 单帧，给快照用，不参与环形缓冲
        self.latest_t = t
        self.n_frames += 1

        if not self.recording:
            return

        # 收到就编码：代价是一次 imencode（640x480 约几毫秒），换来的是
        # 环形缓冲的常驻内存降一个数量级。写盘线程再解码一次是值得的。
        ok, buf = cv2.imencode(
            '.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), self.buffer_quality])
        if not ok:
            return
        jpeg = buf.tobytes()

        with self._lock:
            if self.cur_t0 is None:
                self.cur_t0 = t
            self.cur.append((t, jpeg))
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

        # put_nowait 而不是 put：这个函数跑在图像回调 / 定时器里，排队满了
        # 绝不能把 ROS 回调堵住（一堵，后面的帧全丢，`frame_age` 直接涨）。
        job = {'seq': seq, 'frames': frames, 't0': t0, 't1': t1,
               'wall': time.time(), 'codec': self.codec}
        try:
            self._flush_q.put_nowait(job)
            return
        except queue.Full:
            pass
        # 队列满 = 写盘跟不上。丢**最老**的一段（不是最新的），
        # 保证手机看到的永远是最近的画面。
        try:
            old = self._flush_q.get_nowait()
            self._flush_q.task_done()
            self._dropped_segments += 1
            self.get_logger().warn(
                f'写盘跟不上，丢弃最老的分段 seg_{old["seq"]:06d}'
                f'（累计 {self._dropped_segments} 段）')
        except queue.Empty:
            pass
        try:
            self._flush_q.put_nowait(job)
        except queue.Full:
            # 抢不过写线程：认下这次丢弃，别在这里等
            self._dropped_segments += 1

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

        first = self._decode(frames[0][1])
        if first is None:
            return
        h = even(int(first.shape[0]))
        w = even(int(first.shape[1]))
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
            # 编码器打不开时 cv2 可能已经建了个 0 字节的空壳，顺手清掉，
            # 否则 clips/ 里会慢慢攒一堆没人认领的 .part.mp4。
            try:
                os.remove(tmp)
            except OSError:
                pass
            return
        written = 0
        failed = False
        try:
            for _, buf in frames:
                f = self._decode(buf)
                if f is None:
                    continue
                writer.write(self._fit(f, w, h))
                written += 1
        except Exception:
            # 写失败（磁盘满、编码器崩）：必须**先释放句柄再删半个文件**。
            # 不释放会漏一个 fd，Windows 上还会让随后的删除失败（文件被占用）；
            # 不删的话 clips/ 里会永远躺着一个 `_gc` 看不见（它只按索引清）、
            # 手机也放不了的 .part.mp4。
            failed = True
            raise
        finally:
            writer.release()
            if failed:
                try:
                    os.remove(tmp)
                except OSError:
                    pass
        if written == 0:
            try:
                os.remove(tmp)
            except OSError:
                pass
            return

        os.replace(tmp, path)
        size = os.path.getsize(path)
        rec = {'file': name, 'seq': seq,
               't0': round(t0, 3), 't1': round(t1, 3),
               'wall': round(job['wall'], 3),
               'frames': written, 'fps': round(fps, 3), 'bytes': size,
               'codec': used, 'session': self.session}
        self._append_index(rec)
        self.get_logger().debug(
            f'{name}: {written} 帧 / {dur:.1f}s / {fps:.2f}fps / {size/1024:.0f}KB ({used})')

    @staticmethod
    def _decode(buf):
        """JPEG bytes -> BGR；坏帧返回 None（不因为一帧报废整段）。"""
        try:
            a = np.frombuffer(buf, dtype=np.uint8)
            return cv2.imdecode(a, cv2.IMREAD_COLOR)
        except Exception:
            return None

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

        # type 会拼进目录名，必须先净化（见 safe_token 的注释）；
        # pre/post 夹到 0~300s，否则一个手滑的 pre=1e9 会扫遍整个环形缓冲。
        etype = safe_token(req.get('type', 'event'))
        label = str(req.get('label', etype))[:80]
        pre = clamp_sec(req.get('pre', self.pre_sec), default=self.pre_sec)
        post = clamp_sec(req.get('post', self.post_sec), default=self.post_sec)
        note = str(req.get('note', ''))[:200]

        t_ev = self._now_ros()
        # 先把当前段落盘：这样 pre 窗口的尾部不会卡在内存里等段满
        self._flush_segment()

        ctx = {
            'type': etype, 'label': label, 'note': note,
            't_ev': t_ev, 'pre': pre, 'post': post,
            't_from': t_ev - pre, 't_to': t_ev + post,
            'wall': time.time(),
        }
        if len(self.pending) >= self.MAX_PENDING:
            self.get_logger().warn(
                f'待合成事件已达上限 {self.MAX_PENDING}，丢弃本次事件（{etype}）')
            return
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
            rest = []
            for c in due:
                if self._assemble_sem.acquire(blocking=False):
                    threading.Thread(target=self._assemble_worker, args=(c,),
                                     daemon=True).start()
                else:
                    # 并发满了就留到下一拍，不丢事件也不额外起线程
                    rest.append(c)
            self.pending.extend(rest)

        self._gc()
        wall = time.time()
        if (wall - self._last_dir_gc) >= 60.0:
            self._last_dir_gc = wall
            self._gc_events(wall)
            self._gc_snapshots()
        self._publish_state(now)

    # ---------- 事件合成 ----------

    def _assemble_worker(self, ctx):
        """线程壳：无论合成成功还是抛异常，都必须把并发名额还回去。"""
        try:
            self._assemble_event(ctx)
        except Exception as e:                              # noqa: BLE001
            self.get_logger().error(f'事件合成线程异常：{type(e).__name__}: {e}')
        finally:
            self._assemble_sem.release()

    def _assemble_event(self, ctx):
        """把所有相关分段读回来，裁出窗口内的帧，重新编码成一个事件视频。

        不在事件当下就合成，是因为后置窗口的帧还没产生；等到 t_to 之后再动手，
        用的是「已落盘的完整分段」，不需要额外在内存里留一份长缓冲。

        **流式写**：读一帧写一帧，峰值只多占一帧（900KB）+ 缩略图那一帧。
        早先的写法是先把窗口内所有帧解进 list 再编码，300 帧就是 270MB，
        在 ARM 板上直接 OOM。fps 因此必须提前算出来 —— 用段落元数据估，
        不需要真的读完。
        """
        # 等写盘队列排空（后置窗口刚 flush 的那段可能还在编码）
        self._flush_q.join()

        with self._clips_lock:
            clips = list(self.clips)
        # 只取本会话的段：ROS 时间在重启后会回零，跨会话比较 t0/t1 是没有意义的
        # （见 __init__ 里 session 的注释）。
        hits = [c for c in clips
                if c.get('session') == self.session
                and c.get('t1', 0.0) >= ctx['t_from'] and c.get('t0', 0.0) <= ctx['t_to']]
        hits.sort(key=lambda c: c.get('seq', 0))

        # 事件 id 用墙钟时间戳，人能一眼看出「几点几分」；同秒多条时加序号。
        # 用「不带 exist_ok 的 makedirs + 捕获 FileExistsError 递增重试」原子占位：
        # 本函数跑在每个事件的组装线程上，若先 exists 判断再创建，同秒同类事件
        # （如手动事件 + 跌倒告警）会双双通过检查、落进同一目录互相覆盖
        # video.mp4/meta.json，丢掉一段取证视频。让「目录创建成功」本身充当占位
        # 才是原子的；每次重试 eid 都变，不会在同一名上死循环。
        base = time.strftime('%Y%m%d_%H%M%S', time.localtime(ctx['wall']))
        k = 0
        while True:
            if k == 0:
                eid = 'evt_%s_%s' % (base, ctx['type'])
            else:
                eid = 'evt_%s_%s_%d' % (base, ctx['type'], k)
            out_dir = os.path.join(self.events_dir, eid)
            try:
                os.makedirs(out_dir)          # 已存在会抛 FileExistsError
                break
            except FileExistsError:
                k += 1
        self.n_events += 1

        meta = {
            'id': eid, 'type': ctx['type'], 'label': ctx['label'],
            'note': ctx['note'], 'wall': ctx['wall'],
            't_ev': round(ctx['t_ev'], 3),
            't_from': round(ctx['t_from'], 3), 't_to': round(ctx['t_to'], 3),
            # 显式记下**实际生效**的窗口：请求里可能覆盖默认 pre/post
            # （P5 告警就用 25/10 覆盖了 P4 的 10/10），手机端要按它显示
            # 「事发前 25 秒」；只给 t_from/t_to 的话前端还得自己减。
            'pre_sec': round(ctx['pre'], 2),
            'post_sec': round(ctx['post'], 2),
            'duration': round(ctx['t_to'] - ctx['t_from'], 2),
            'clips': [c['file'] for c in hits],
            'frames': 0, 'has_video': False,
        }

        if not hits:
            self.get_logger().warn(
                f'{eid}: 窗口内没有可用分段（可能已被环形清理）')
            meta['error'] = 'no_frames'
            self._write_meta(out_dir, meta)
            return

        fps = self._estimate_event_fps(hits, ctx)
        path = os.path.join(out_dir, 'video.mp4')
        tmp = os.path.join(out_dir, 'video.part.mp4')   # 扩展名见 _write_segment 的注释

        writer = None
        used = None
        written = 0
        thumb_d = float('inf')
        thumb = None
        for c in hits:
            for t, img in self._iter_clip_frames(c, ctx['t_from'], ctx['t_to']):
                if writer is None:
                    h = even(int(img.shape[0]))
                    w = even(int(img.shape[1]))
                    writer, used = self._open_writer(tmp, fps, (w, h))
                    if writer is None:
                        meta['error'] = 'encoder_unavailable'
                        self._write_meta(out_dir, meta)
                        return
                d = abs(t - ctx['t_ev'])
                if d < thumb_d:
                    thumb_d, thumb = d, img.copy()
                writer.write(self._fit(img, w, h))
                written += 1
                if written >= self.max_event_frames:
                    break
            if written >= self.max_event_frames:
                break

        if writer is None:
            self.get_logger().warn(f'{eid}: 窗口内没有可用帧')
            meta['error'] = 'no_frames'
            self._write_meta(out_dir, meta)
            return

        writer.release()
        os.replace(tmp, path)

        if self.faststart:
            if faststart_remux(path, self.get_logger()):
                self.get_logger().info(f'{eid}: 已重封装为 faststart（moov 前置）')

        # 缩略图取最接近事发时刻的那一帧
        if thumb is not None:
            ok, buf = cv2.imencode('.jpg', thumb,
                                   [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
            if ok:
                with open(os.path.join(out_dir, 'thumb.jpg'), 'wb') as f:
                    f.write(buf.tobytes())

        meta.update({'has_video': True, 'fps': round(fps, 3), 'frames': written,
                     'bytes': os.path.getsize(path), 'codec': used})
        self._write_meta(out_dir, meta)
        self.get_logger().info(
            f'{eid}: {written} 帧 / {fps:.2f}fps / '
            f'{os.path.getsize(path)/1024:.0f}KB，来源 {len(hits)} 段'
            f'（{ctx["label"]}）')

    def _estimate_event_fps(self, hits, ctx):
        """不读帧、只按段落元数据估算窗口内的 fps。

        每个段落的帧是等间隔写入的，所以 (t1-t0)/(frames-1) 就是帧间隔；
        把每段与窗口的重叠时长折算成帧数，再除以窗口跨度，就是播放帧率。
        这样不必先把帧读进内存就能给 VideoWriter 一个正确的 fps。
        """
        est = 0
        lo = None
        hi = None
        for c in hits:
            f0 = max(float(c.get('t0', 0.0)), ctx['t_from'])
            f1 = min(float(c.get('t1', 0.0)), ctx['t_to'])
            if f1 < f0:
                continue
            lo = f0 if lo is None else min(lo, f0)
            hi = f1 if hi is None else max(hi, f1)
            n = int(c.get('frames') or 0)
            if n <= 1:
                est += 1
                continue
            dt = (float(c['t1']) - float(c['t0'])) / (n - 1)
            est += int((f1 - f0) / dt) + 1 if dt > 1e-6 else n
        if lo is None or hi is None:
            return 10.0
        # 注意：这里**不能**按 max_event_frames 截断 est。截断只会让视频变短
        # （丢掉窗口尾部的帧），不应该改变播放速度；拿截断后的帧数除以整窗
        # 时长会把 10fps 算成 7fps，回放就是慢动作。
        est = max(1, est)
        span = max(1e-3, hi - lo)
        fps = est / span if span > 0.5 else 1.0
        return max(0.5, min(60.0, fps))

    def _iter_clip_frames(self, clip, t_from, t_to):
        """逐帧读一个分段，按线性插值给每帧估时间戳并裁窗口（生成器）。

        mp4 里没有逐帧时间戳（至少 cv2 读不到），但一段内的帧是等间隔写进去的，
        用 (t1-t0)/N 插值足够定位「事发前 N 秒」这个精度。

        做成生成器是为了让 _assemble_event 能边读边写：调用方不攒帧，
        内存峰值就只有当前这一帧。
        """
        path = self._clip_path(clip)
        if path is None or not os.path.exists(path):
            return
        cap = cv2.VideoCapture(path)
        if not cap.isOpened():
            return
        try:
            n = int(clip.get('frames') or 0)
            t0, t1 = float(clip['t0']), float(clip['t1'])
            dt = (t1 - t0) / max(1, n - 1) if n > 1 else 0.0
            i = 0
            while True:
                ok, img = cap.read()
                if not ok:
                    return
                t = t0 + i * dt if dt > 0 else t0
                if t > t_to:
                    return
                if t >= t_from:
                    yield t, img
                i += 1
        finally:
            cap.release()

    def _read_clip_frames(self, clip, t_from, t_to):
        """兼容旧调用：把生成器收成 list（会把整段读进内存，事件合成不用它）。"""
        return list(self._iter_clip_frames(clip, t_from, t_to))

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
        with self._clips_lock:
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
            total = sum(c.get('bytes', 0) for c in self.clips)
            while total > self.max_disk_mb * 1024 * 1024 and len(self.clips) > 1:
                c = self.clips.pop(0)
                total -= c.get('bytes', 0)
                self._drop_clip(c)
                removed += 1

            if removed:
                self._rewrite_index()

    def _drop_clip(self, clip):
        p = self._clip_path(clip)
        if p is None:
            self.get_logger().warn(
                f'索引里的 file 不是纯文件名，拒绝删除：{clip.get("file")!r}')
            return
        try:
            os.remove(p)
        except FileNotFoundError:
            pass
        except Exception as e:
            self.get_logger().warn(f'删除 {p} 失败：{e}')

    def _clip_path(self, clip):
        """索引里的 file 必须是纯文件名：它会被 _drop_clip 传给 os.remove。

        clips.jsonl 与录像共用一个数据目录，一旦有别的写入者（或旧版索引）
        把 file 写成 `../../xxx`，清理逻辑就会删到目录外面去。
        """
        name = clip.get('file') if isinstance(clip, dict) else None
        if (not isinstance(name, str) or not name or name in ('.', '..')
                or '/' in name or '\\' in name or '\x00' in name
                or name != os.path.basename(name)):
            return None
        return os.path.join(self.clips_dir, name)

    def _rewrite_index(self):
        """索引是 append-only 的，清理后重写成一个小文件，避免无限增长。

        **调用方必须已持有 self._clips_lock**：这里读 self.clips 并原子替换
        clips.jsonl，锁不可重入（_gc 已在锁内调用本函数），故本函数自己不加锁。
        """
        tmp = self.index_path + '.part'
        try:
            with open(tmp, 'w', encoding='utf-8') as f:
                for c in self.clips:
                    f.write(json.dumps(c, ensure_ascii=False) + '\n')
            os.replace(tmp, self.index_path)
        except Exception as e:
            self.get_logger().warn(f'重写索引失败：{e}')

    def _gc_events(self, now):
        """事件视频保留策略：按天数 + 总容量，且至少留 min_events 条。

        之前只清 clips/，events/ 是只进不出的。真机上一次跌倒事件 35s 视频约
        2~3MB，一天几次、一个月就是几百 MB —— SD 卡写满以后录像、事件切片、
        告警落盘全部失败，而且通常发生在半夜。
        """
        try:
            names = os.listdir(self.events_dir)
        except OSError:
            return

        items = []
        for name in names:
            d = os.path.join(self.events_dir, name)
            # os.path.isdir 会跟随符号链接：不加这一条，指向外部目录的链接
            # 会被当成普通事件目录交给 _rm_tree 递归删光。
            if os.path.islink(d) or not os.path.isdir(d):
                continue
            # 正在合成的目录绝对不能删：video.part.mp4 还在写
            if os.path.exists(os.path.join(d, 'video.part.mp4')):
                continue
            try:
                mtime = os.path.getmtime(os.path.join(d, 'meta.json'))
            except OSError:
                try:
                    mtime = os.path.getmtime(d)
                except OSError:
                    continue
            size = 0
            try:
                with os.scandir(d) as it:
                    for e in it:
                        try:
                            if e.is_file(follow_symlinks=False):
                                size += e.stat(follow_symlinks=False).st_size
                        except OSError:
                            pass
            except OSError:
                pass
            items.append((name, mtime, size))

        if not items:
            return
        items.sort(key=lambda x: x[1])          # 最老的在前
        total = sum(x[2] for x in items)
        keep_from = max(0, len(items) - self.min_events)

        removed = 0
        for idx, (name, mtime, size) in enumerate(items):
            if idx >= keep_from:
                break                            # 进入「至少保留」区间
            too_old = (now - mtime) > self.event_retain_days * 86400.0
            too_big = total > self.max_event_mb * 1048576.0
            if not (too_old or too_big):
                continue
            if self._rm_tree(os.path.join(self.events_dir, name)):
                total -= size
                removed += 1

        if removed:
            self.get_logger().info(
                f'事件清理：删除 {removed} 条，剩余 {total/1048576.0:.1f}MB '
                f'（上限 {self.max_event_mb:.0f}MB / {self.event_retain_days:.0f} 天）')

    @classmethod
    def _rm_tree(cls, path):
        """删除一个目录及其内容；失败只返回 False，不抛（清理不能影响录像）。"""
        try:
            if os.path.islink(path):
                return False
            with os.scandir(path) as it:
                entries = list(it)
            for e in entries:
                try:
                    if e.is_dir(follow_symlinks=False):
                        cls._rm_tree(e.path)
                    else:
                        os.remove(e.path)
                except OSError:
                    pass
            os.rmdir(path)
            return True
        except OSError:
            return False

    def _gc_snapshots(self):
        """快照按文件名（时间戳）保留最新的 max_snapshots 张。"""
        if self.max_snapshots <= 0:
            return
        try:
            names = [n for n in os.listdir(self.snaps_dir)
                     if n.lower().endswith('.jpg')]
        except OSError:
            return
        if len(names) <= self.max_snapshots:
            return
        names.sort()
        for n in names[:len(names) - self.max_snapshots]:
            try:
                os.remove(os.path.join(self.snaps_dir, n))
            except OSError:
                pass

    # ---------- 状态 ----------

    def _publish_state(self, now):
        with self._lock:
            cur_n = len(self.cur)
            cur_t0 = self.cur_t0
        with self._clips_lock:
            clips = list(self.clips)
        total = sum(c.get('bytes', 0) for c in clips)
        last_fps = clips[-1].get('fps', 0.0) if clips else 0.0
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
            # 写盘跟不上丢掉的段数。>0 说明 SD 卡/CPU 已经扛不住当前帧率，
            # 手机端能直接看到（不然只表现为「录像莫名其妙缺了一段」）。
            'dropped_segments': self._dropped_segments,
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
        except Exception:
            pass
        # 队列有界，直接 put 在「写盘正好卡住」时会永久阻塞 —— 关节点反而卡死。
        # 带超时，抢不到就放弃这一轮等待（worker 是 daemon，进程照样能退）。
        try:
            self._flush_q.put(None, timeout=2.0)
        except queue.Full:
            pass
        try:
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
