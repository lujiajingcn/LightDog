#!/usr/bin/env python3
"""
LightDog 人体检测器（P2 / 场景 1）

## 这一版为什么不是 YOLO

产品的最终形态需要姿态识别（P5 跌倒检测），那时必须上真实模型。但 P2 要解决的
是「检测 -> 跟随 -> 光照」这条**控制闭环**能不能跑通，而且仿真里的「人」是我们
自己造的几何体，颜色可控。所以这一版用**颜色 + 深度**做检测，好处是：

- 零额外依赖：只用 numpy，不装 torch / ultralytics / opencv（镜像里都没有，
  当前网络下装 torch 要拉 ~2GB）
- 结果确定：仿真人上衣是高饱和橙，HSV 上和环境（灰白墙、绿地、黄车身、
  暖白灯条）分得干净，闭环验证不会被模型精度干扰

接口按可替换设计：外部只依赖 /person/present 与 /person/position 两个话题，
将来换 YOLOv8-pose 只需替换本文件的 _detect()，无关节点不用改。

## 输出（产品层接口）

    /person/present    std_msgs/Bool        画面里有人（按检测频率持续发布）
    /person/position   geometry_msgs/PointStamped
                       frame_id = base_link
                       x = 前向距离 (m)
                       y = 左右偏移 (m)，左为正
                       z = 目标高度 (m)，用于后续判断站/躺
    /person/debug_image sensor_msgs/Image   画了检测框的画面（R8G8B8）

## 输入

    /camera/color/image_raw          sensor_msgs/Image   640x480 @30Hz
    /camera/depth/image_rect_raw     sensor_msgs/Image   与彩色图同尺寸
    /camera/color/camera_info        sensor_msgs/CameraInfo（取 fx / cx）

## 相机安装

4wd 车型 `depth_sensor_pose` = xyz "-0.065 0 0.255" rpy "0 0 0"（见
linorobot2_description/urdf/4wd_properties.urdf.xacro）。rpy 为零意味着相机
光轴就是 base_link 的 +X，所以只需要补偿这段平移，不用 TF。

若改车型或改安装位，改 camera_offset 参数即可。
"""

import math
import sys
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Bool, Header
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped


# ---------- 图像/深度解码（不依赖 cv_bridge） ----------

def image_to_rgb(msg: Image):
    """sensor_msgs/Image -> (H, W, 3) uint8 RGB；无法识别返回 None。"""
    h, w = msg.height, msg.width
    enc = msg.encoding.lower()
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    if enc in ('rgb8', 'bgr8'):
        if buf.size < h * w * 3:
            return None
        a = buf[:h * w * 3].reshape(h, w, 3)
        return a[:, :, ::-1].copy() if enc == 'bgr8' else a.copy()
    if enc in ('rgba8', 'bgra8'):
        if buf.size < h * w * 4:
            return None
        a = buf[:h * w * 4].reshape(h, w, 4)[:, :, :3]
        return a[:, :, ::-1].copy() if enc == 'bgra8' else a.copy()
    return None


def image_to_depth(msg: Image):
    """sensor_msgs/Image -> (H, W) float32 米；无法识别返回 None。

    gz-sim 的 rgbd_camera 深度图实测是 32FC1（米）；16UC1 是毫米，按 1000 折算。
    """
    h, w = msg.height, msg.width
    enc = msg.encoding.lower()
    if enc == '32fc1':
        need = h * w * 4
        if len(msg.data) < need:
            return None
        return np.frombuffer(msg.data, dtype=np.float32)[:h * w].reshape(h, w).copy()
    if enc in ('16uc1', 'mono16'):
        need = h * w * 2
        if len(msg.data) < need:
            return None
        return (np.frombuffer(msg.data, dtype=np.uint16)[:h * w]
                .reshape(h, w).astype(np.float32) / 1000.0)
    return None


def rgb_to_hsv(rgb: np.ndarray):
    """(H, W, 3) uint8 RGB -> h(0~1), s(0~1), v(0~1)。纯 numpy，省掉 cv2 依赖。"""
    a = rgb.astype(np.float32) / 255.0
    r, g, b = a[:, :, 0], a[:, :, 1], a[:, :, 2]
    mx = a.max(axis=2)
    mn = a.min(axis=2)
    delta = mx - mn
    h = np.zeros_like(mx)
    nz = delta > 1e-6
    # 按最大通道分支计算色相
    rmax = nz & (mx == r)
    gmax = nz & (mx == g) & ~rmax
    bmax = nz & (mx == b) & ~rmax & ~gmax
    with np.errstate(invalid='ignore', divide='ignore'):
        h[rmax] = ((g[rmax] - b[rmax]) / delta[rmax]) % 6.0
        h[gmax] = (b[gmax] - r[gmax]) / delta[gmax] + 2.0
        h[bmax] = (r[bmax] - g[bmax]) / delta[bmax] + 4.0
    h = h / 6.0
    s = np.where(mx > 1e-6, delta / np.maximum(mx, 1e-6), 0.0)
    return h, s, mx


class PersonDetector(Node):

    def __init__(self):
        super().__init__('lightdog_person_detector')

        # ---- 输入 ----
        self.declare_parameter('rgb_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth/image_rect_raw')
        self.declare_parameter('info_topic', '/camera/color/camera_info')

        # ---- 相机安装偏移（base_link 下的 x,y,z），见文件头注释 ----
        self.declare_parameter('camera_offset', [-0.065, 0.0, 0.255])

        # ---- 颜色判据（HSV 归一化 0~1；橙色上衣实测 H≈0.052）----
        self.declare_parameter('h_min', 0.012)
        self.declare_parameter('h_max', 0.090)
        self.declare_parameter('s_min', 0.45)
        self.declare_parameter('v_min', 0.15)

        # ---- 空间过滤 ----
        self.declare_parameter('min_blob_px', 150)
        self.declare_parameter('col_gap', 6)          # 列方向允许的空隙像素
        self.declare_parameter('range_min', 0.30)
        self.declare_parameter('range_max', 8.00)
        # 形状约束：只用来滤掉极端横向的色块，不能卡太紧。
        # 相机装高只有 0.3745m（0.255 偏移 + 0.1195 base_link），人贴近到
        # 0.8m 时躯干在画面里只剩一条横带（140x56px，高/宽 = 0.4）。
        # 所以默认值必须跟 p2_params.yaml 一致（0.55 / 30）—— 这里的默认值
        # 曾经是 1.20 / 36，不带 params 启动就会复现 P2 §6.5 的「人贴近就丢检」。
        self.declare_parameter('aspect_min', 0.55)
        self.declare_parameter('min_height_px', 30)

        # ---- 输出 ----
        self.declare_parameter('publish_debug', True)
        self.declare_parameter('rate', 10.0)          # 处理频率，软件渲染下别给太高
        self.declare_parameter('lost_timeout', 1.0)   # 超过这么久没检测到就报没人
        # 帧新鲜度（**墙钟**秒）：回答的是「相机管线还在不在出帧」，与仿真
        # 跑多快无关，所以刻意不用 ROS 时钟。必须大于相机周期的若干倍：
        # 仿真相机实测 1.5Hz（0.67s/帧），真机 30Hz。1.5s 在仿真下容忍 2 帧丢失，
        # 在真机上 1.5s 还没新帧就说明管线已经死了。
        self.declare_parameter('frame_timeout', 1.5)
        # 深度图单独判：彩色在出帧、深度掉了，是最常见的半失效状态。
        # 这时不要拿旧深度配新彩色，退回「躯干高度/像素高度」粗估。
        self.declare_parameter('depth_timeout', 1.0)

        p = self.get_parameter
        self.cam_off = list(p('camera_offset').value)
        self.h_min, self.h_max = p('h_min').value, p('h_max').value
        self.s_min = float(p('s_min').value)
        self.v_min = float(p('v_min').value)
        self.min_blob = p('min_blob_px').value
        self.col_gap = p('col_gap').value
        self.rng_min, self.rng_max = p('range_min').value, p('range_max').value
        self.aspect_min = float(p('aspect_min').value)
        self.min_h = int(p('min_height_px').value)
        self.publish_debug = p('publish_debug').value
        self.frame_timeout = float(p('frame_timeout').value)
        self.depth_timeout = float(p('depth_timeout').value)
        self.lost_timeout = float(p('lost_timeout').value)

        self.fx = self.cx = None
        self.cy = None
        self.rgb = None
        self.depth = None
        self.last_seen = None

        # 帧序号 + 到达时刻：只对**新帧**做一次检测，并能在相机断流时报无人。
        # 没有这两个量时，_tick 会以 10Hz 对同一张旧图反复 _detect()，永远成功 ——
        # 相机冻帧/掉线后 /person/present 恒 true，跟随器会朝着画面里的「幽灵」
        # 一直开下去，灯也不灭。这是上真机前必须堵掉的坑。
        self.rgb_seq = 0
        self._det_seq = -1
        self.rgb_wall = 0.0
        self.depth_wall = 0.0

        self.present = False          # 上一次判定结果，供无新帧时重发
        self._warned_rgb_stall = False
        self._warned_no_input = False

        sensor_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.create_subscription(
            Image, p('rgb_topic').value, self._on_rgb, sensor_qos)
        self.create_subscription(
            Image, p('depth_topic').value, self._on_depth, sensor_qos)
        self.create_subscription(
            CameraInfo, p('info_topic').value, self._on_info, sensor_qos)

        self.pub_present = self.create_publisher(Bool, '/person/present', 10)
        self.pub_pos = self.create_publisher(PointStamped, '/person/position', 10)
        self.pub_dbg = self.create_publisher(Image, '/person/debug_image', 2)

        self.create_timer(1.0 / float(p('rate').value), self._tick)

        self.get_logger().info(
            'LightDog person detector ready: '
            f'H=[{self.h_min:.3f},{self.h_max:.3f}] S>{self.s_min} V>{self.v_min} '
            f'range=[{self.rng_min},{self.rng_max}]m '
            f'aspect_min={self.aspect_min} min_h={self.min_h}px '
            f'frame_timeout={self.frame_timeout:.1f}s')

    # ---------- 回调 ----------

    def _on_info(self, msg: CameraInfo):
        # 注意：CameraInfo.k 在 rclpy 里可能是 numpy 数组，
        # 写 `if not msg.k` 会抛「truth value of an array ... is ambiguous」。
        # 只用 len() 判空。
        if msg.k is None or len(msg.k) < 9:
            return
        fx = float(msg.k[0])
        if not math.isfinite(fx) or fx <= 0.0:
            return
        self.fx = fx
        self.cx = float(msg.k[2])
        self.cy = float(msg.k[5])

    def _on_rgb(self, msg: Image):
        self.rgb = msg
        self.rgb_seq += 1
        self.rgb_wall = time.monotonic()
        if self._warned_rgb_stall:
            self._warned_rgb_stall = False
            self.get_logger().info('相机图像恢复')

    def _on_depth(self, msg: Image):
        self.depth = msg
        self.depth_wall = time.monotonic()

    # ---------- 主循环 ----------

    def _person_expired(self, now_mono):
        """最后一次成功检出距今是否已超过 lost_timeout（墙钟秒）。"""
        if self.last_seen is None:
            return True
        return (now_mono - self.last_seen) > self.lost_timeout

    def _tick(self):
        now_mono = time.monotonic()

        # 没相机 / 没内参：这是「看不见」，不等于「没人」，但下游只有这一个
        # Bool 可看，所以必须明确发 false 并留一条日志。改之前这里直接
        # return —— 真机上话题名写错的现象是「跟随和自动灯都不工作，
        # 而节点日志一片安静」，只能靠猜。
        if self.rgb is None or self.fx is None or self.fx <= 1.0:
            if not self._warned_no_input:
                self._warned_no_input = True
                what = '彩色图' if self.rgb is None else 'camera_info（fx 无效）'
                self.get_logger().error(
                    f'尚未收到{what}，/person/present 固定发 false。'
                    f'检查 rgb_topic / info_topic 是否与实际相机驱动一致')
            self.present = False
            self.pub_present.publish(Bool(data=False))
            return
        self._warned_no_input = False

        if (now_mono - self.rgb_wall) > self.frame_timeout:
            # 相机断流/冻帧：绝不能拿最后一帧反复判「有人」。否则跟随器会对着
            # 画面里的幽灵一直走，而这正是最危险的一种失效（人早就走开了）。
            if not self._warned_rgb_stall:
                self._warned_rgb_stall = True
                self.get_logger().warn(
                    f'相机 {now_mono - self.rgb_wall:.1f}s 没有新帧，'
                    f'判定为无人并停止发布位置（检查相机/线缆/USB 带宽）')
            self.last_seen = None
            self.present = False
            self.pub_present.publish(Bool(data=False))
            return

        if self.rgb_seq == self._det_seq:
            # 没有新帧就不重复检测，但仍然按 rate 重发判定结果，
            # 让下游的超时逻辑（/person/present 是 Bool，没有时间戳）照常工作。
            # 重发值也要按 lost_timeout 老化：相机还在出帧、但这一帧检不到人时，
            # 若原样重发 true，连续丢检期间的 present 会一直粘在 true 上。
            present = self.present and not self._person_expired(now_mono)
            self.present = present
            self.pub_present.publish(Bool(data=present))
            return
        self._det_seq = self.rgb_seq

        det = self._detect()

        # 计龄一律用墙钟：这里问的是「多久没检出人了」，与仿真跑多快无关。
        # 用 ROS 时钟时若 use_sim_time 被误开而没有 /clock，时钟停走，
        # 这个超时永远不会到，灯就一直亮着。
        if det is not None:
            self.last_seen = now_mono
            self._publish_position(*det)
            present = True
        else:
            present = not self._person_expired(now_mono)

        self.present = present
        self.pub_present.publish(Bool(data=present))

    def _detect(self):
        """返回 (x_fwd, y_left, z_height, bbox) 或 None。bbox=(u0,v0,u1,v1)。"""
        rgb = image_to_rgb(self.rgb)
        if rgb is None:
            return None
        h, w, _ = rgb.shape

        hh, ss, vv = rgb_to_hsv(rgb)
        mask = ((hh >= self.h_min) & (hh <= self.h_max) &
                (ss >= self.s_min) & (vv >= self.v_min))
        if not mask.any():
            return None

        # 列投影找纵向条带（允许小空隙，人身上可能有阴影/遮挡断开）
        col = mask.sum(axis=0)
        cols = np.where(col > 0)[0]
        if cols.size == 0:
            return None

        segments = []
        start = cols[0]
        prev = cols[0]
        for c in cols[1:]:
            if c - prev <= self.col_gap:
                prev = c
            else:
                segments.append((start, prev))
                start = prev = c
        segments.append((start, prev))

        # 取像素数最多的条带
        best = None
        best_px = 0
        for u0, u1 in segments:
            sub = mask[:, u0:u1 + 1]
            px = int(sub.sum())
            if px > best_px:
                best_px, best = px, (u0, u1)
        if best is None or best_px < self.min_blob:
            return None

        u0, u1 = best
        rows = np.where(mask[:, u0:u1 + 1].any(axis=1))[0]
        if rows.size == 0:
            return None
        v0, v1 = int(rows[0]), int(rows[-1])

        # 形状约束：站立的人是竖长条。这一条把「球形头」「地上一块橙色杂物」
        # 这类误检挡在外面 —— 仿真里头部肤色渲染后 H/S 都落在橙色区间，
        # 只靠颜色分不开，必须用形状。
        bw = u1 - u0 + 1
        bh = v1 - v0 + 1
        if bh < self.min_h or bh < self.aspect_min * bw:
            if self.publish_debug:
                self._publish_debug(rgb, (u0, v0, u1, v1), False)
            return None

        # 距离：掩码像素上的深度中位数（避开背景深度把目标拉远）
        z = None
        dmsg = self.depth
        # 深度也要判新鲜度：彩色在出帧、深度掉了的时候，拿旧深度配新彩色
        # 会把人物的位置算到上一次的位置上（人走动时表现为「距离粘住」）。
        if (dmsg is not None
                and (time.monotonic() - self.depth_wall) <= self.depth_timeout):
            d = image_to_depth(dmsg)
            if d is not None and d.shape == (h, w):
                sub_mask = mask[v0:v1 + 1, u0:u1 + 1]
                sub_d = d[v0:v1 + 1, u0:u1 + 1]
                vals = sub_d[sub_mask]
                vals = vals[np.isfinite(vals) & (vals > 0.05)]
                if vals.size >= 20:
                    z = float(np.median(vals))

        if z is None:
            # 深度缺失时用「躯干高度/像素高度」粗估，仅用于不中断闭环
            px_h = max(1, v1 - v0 + 1)
            z = 0.62 * self.fx / float(px_h)

        if not (self.rng_min <= z <= self.rng_max):
            if self.publish_debug:
                self._publish_debug(rgb, (u0, v0, u1, v1), False)
            return None

        # 像素 -> 相机光学系 (X右, Y下, Z前) -> base_link (x前, y左, z上)
        u_c = 0.5 * (u0 + u1)
        v_c = 0.5 * (v0 + v1)
        x_cam = (u_c - self.cx) / self.fx * z
        y_cam = (v_c - self.cy) / self.fx * z

        x_base = z + self.cam_off[0]
        y_base = -x_cam + self.cam_off[1]
        z_base = -y_cam + self.cam_off[2]

        if self.publish_debug:
            self._publish_debug(rgb, (u0, v0, u1, v1), True)

        return (x_base, y_base, z_base, (u0, v0, u1, v1))

    # ---------- 发布 ----------

    def _publish_position(self, x, y, z, bbox):
        msg = PointStamped()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.point.x = float(x)
        msg.point.y = float(y)
        msg.point.z = float(z)
        self.pub_pos.publish(msg)
        self.get_logger().debug(
            f'person: fwd={x:.2f} left={y:+.2f} h={z:.2f} bbox={bbox}')

    def _publish_debug(self, rgb, bbox, found):
        u0, v0, u1, v1 = bbox
        out = rgb.copy()
        # 画框（纯 numpy，不引 cv2）
        color = np.array([0, 255, 0] if found else [255, 64, 64], dtype=np.uint8)
        t = 2
        h, w, _ = out.shape
        u0c, u1c = max(0, u0), min(w - 1, u1)
        v0c, v1c = max(0, v0), min(h - 1, v1)
        out[max(0, v0c - t):v0c + t, u0c:u1c] = color
        out[v1c - t:v1c + t, u0c:u1c] = color
        out[v0c:v1c, max(0, u0c - t):u0c + t] = color
        out[v0c:v1c, u1c - t:u1c + t] = color

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_link'
        msg.height, msg.width = h, w
        msg.encoding = 'rgb8'
        msg.is_bigendian = 0
        msg.step = w * 3
        msg.data = out.tobytes()
        self.pub_dbg.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PersonDetector()
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
