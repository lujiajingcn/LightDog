#!/usr/bin/env python3
"""
LightDog 人体检测器（P2 / 场景 1）

## 三个后端（`backend` 参数）

    color  仿真占位实现：认高饱和橙色上衣，只用 numpy。零依赖、结果确定，
           用来验证「检测 -> 跟随 -> 光照」这条控制闭环。
    yolo   ultralytics，调试方便（自动做预处理）。但 Pi 5 无 CUDA，PyTorch
           CPU 只有 1~2 FPS，撑不住 frame_timeout 要求的 ≥3 FPS。
    onnx   onnxruntime + 320 输入，真机用这个（Pi 5 上 6~10 FPS）。
           预处理/后处理在本文件里用纯 numpy 实现，见 letterbox / decode_yolo_onnx。

三者共用同一套「框 -> base_link 3D 点」的几何换算，所以换后端不会改动
`/person/position` 的语义（frame_id=base_link），下游节点一行都不用动。
缺依赖时**一律回退到 color 并打 error**，不会「启动成功但永远检测不到人」。

## 这一版为什么最早不是 YOLO

产品的最终形态需要姿态识别（P5 跌倒检测），那时必须上真实模型。但 P2 要解决的
是「检测 -> 跟随 -> 光照」这条**控制闭环**能不能跑通，而且仿真里的「人」是我们
自己造的几何体，颜色可控。所以第一版用**颜色 + 深度**做检测，好处是：

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


# ---------------------------------------------------------------- onnx 后端
#
# 为什么单独写一套预处理/后处理，而不是继续用 ultralytics：
#   Pi 5 没有 CUDA，ultralytics 走 PyTorch CPU 实测 1~2 FPS（docs/hardware_readiness.md
#   §8.5），撑不住 frame_timeout 要求的 ≥3 FPS。onnxruntime + 320 输入能到 6~10 FPS。
#   代价是模型的输入输出要自己接 —— letterbox、解码、NMS 全在这里。
#
# 这一段的函数都是**纯 numpy**、不碰 onnxruntime、不碰 ROS，所以可以在宿主机上
# 直接跑回归（lightdog/tests/test_onnx_detector.py）。这类几何换算最容易出的错是
# 「padding 补偿漏一项」，表现是坐标整体偏几十像素而**不报任何错** —— 只能靠
# 对着已知输入断言来挡。

ONNX_PAD = 114      # letterbox 填充灰度，与 ultralytics 默认值一致


def _resize_bilinear(img, out_w, out_h):
    """(H, W, C) uint8 双线性缩放到 out_w × out_h，返回 float32。

    不用 cv2.resize 是为了让这几段能在没有 cv2 的宿主机上测；
    也不用整数最近邻 —— 320 输入下最近邻丢掉的细节恰好是远距离行人的轮廓。
    """
    h, w = img.shape[:2]
    if out_w == w and out_h == h:
        return img.astype(np.float32)

    # 像素中心对齐（对齐 OpenCV 的 INTER_LINEAR 约定）：不这么做时，
    # 缩放 2 倍的图会整体偏半个像素，两三个 stage 累积下来就是可见的偏移。
    ys = (np.arange(out_h, dtype=np.float32) + 0.5) * (h / out_h) - 0.5
    xs = (np.arange(out_w, dtype=np.float32) + 0.5) * (w / out_w) - 0.5

    y0 = np.floor(ys).astype(np.int32)
    x0 = np.floor(xs).astype(np.int32)
    wy = (ys - y0).clip(0.0, 1.0)[:, None, None]
    wx = (xs - x0).clip(0.0, 1.0)[None, :, None]
    # 越界的邻居夹到边缘，同时把权重归零，避免把边上的像素重复加权
    y0c = y0.clip(0, h - 1)
    y1c = (y0 + 1).clip(0, h - 1)
    x0c = x0.clip(0, w - 1)
    x1c = (x0 + 1).clip(0, w - 1)

    src = img.astype(np.float32)
    top = src[y0c][:, x0c] * (1.0 - wx) + src[y0c][:, x1c] * wx
    bot = src[y1c][:, x0c] * (1.0 - wx) + src[y1c][:, x1c] * wx
    return top * (1.0 - wy) + bot * wy


def letterbox(rgb, size=320, pad=ONNX_PAD):
    """等比缩放 + 居中填充到 size×size。

    返回 (canvas, scale, dx, dy)：
      canvas —— (size, size, 3) uint8
      scale  —— 缩放比（原图 -> canvas）
      dx, dy —— 左上角填充量，**还原坐标时必须减掉**

    只做等比、不做拉伸：拉伸会把人的宽高比改掉，而下游 `_bbox_to_point`
    用框的像素高度反推躯干高度，宽高比错了距离就跟着错。
    """
    h, w = rgb.shape[:2]
    if h <= 0 or w <= 0:
        raise ValueError('letterbox 收到空图：%dx%d' % (w, h))
    scale = min(float(size) / float(w), float(size) / float(h))
    new_w = max(1, int(round(w * scale)))
    new_h = max(1, int(round(h * scale)))

    canvas = np.full((size, size, 3), pad, dtype=np.uint8)
    dx = (size - new_w) // 2
    dy = (size - new_h) // 2
    canvas[dy:dy + new_h, dx:dx + new_w] = _resize_bilinear(
        rgb, new_w, new_h).clip(0, 255).astype(np.uint8)
    return canvas, scale, dx, dy


def nms(boxes, scores, iou_thr=0.45):
    """单类别 NMS。boxes=(N,4) xyxy，scores=(N,)。返回保留的下标列表。

    朴素实现（不依赖 torchvision / cv2.dnn）：一个人的场景里候选框通常
    只有个位数，O(N²) 的代价可以忽略，换来的是这段逻辑在宿主机上可以断言。
    """
    boxes = np.asarray(boxes, dtype=np.float32).reshape(-1, 4)
    scores = np.asarray(scores, dtype=np.float32).reshape(-1)
    if boxes.shape[0] == 0:
        return []

    order = np.argsort(scores)[::-1]
    keep = []
    while order.size > 0:
        i = int(order[0])
        keep.append(i)
        if order.size == 1:
            break
        rest = order[1:]
        xx1 = np.maximum(boxes[i, 0], boxes[rest, 0])
        yy1 = np.maximum(boxes[i, 1], boxes[rest, 1])
        xx2 = np.minimum(boxes[i, 2], boxes[rest, 2])
        yy2 = np.minimum(boxes[i, 3], boxes[rest, 3])
        inter = np.maximum(0.0, xx2 - xx1) * np.maximum(0.0, yy2 - yy1)
        area_i = max(0.0, boxes[i, 2] - boxes[i, 0]) * max(0.0, boxes[i, 3] - boxes[i, 1])
        area_r = (np.maximum(0.0, boxes[rest, 2] - boxes[rest, 0]) *
                  np.maximum(0.0, boxes[rest, 3] - boxes[rest, 1]))
        union = area_i + area_r - inter
        iou = np.where(union > 1e-9, inter / np.maximum(union, 1e-9), 0.0)
        order = rest[iou <= iou_thr]
    return keep


def decode_yolo_onnx(out, scale, dx, dy, img_w, img_h,
                     conf_thr=0.35, iou_thr=0.45, person_class=0):
    """YOLOv8 的 onnx 输出 -> 原图坐标的框，按分数降序。

    out 接受两种布局：
      (1, 4+nc, N)  —— ultralytics 导出的默认布局（320 输入时 N=2100）
      (1, N, 4+nc)  —— 某些导出器会转置
    靠「哪个维度更小就当成 4+nc」判断：4+nc 至少是 5（单类别自训练模型），
    而 N 是 2100/8400 这种量级，不会混淆。

    还原公式：x_orig = (x_in_canvas - dx) / scale
    —— 漏掉 dx/dy 或除以 scale 的写法都能"跑起来"，只是框整体偏，
    偏多少取决于 padding 多大，是最难归因的一类错。
    """
    a = np.asarray(out, dtype=np.float32)
    if a.ndim == 3:
        a = a[0]
    elif a.ndim == 1:
        a = a.reshape(-1, 1)
    if a.ndim != 2:
        raise ValueError('decode_yolo_onnx 看不懂的输出形状：%s' % (np.shape(out),))

    rows, cols = a.shape
    # rows 小、cols 大 -> 已经是 (4+nc, N)
    if rows > cols:
        a = a.T
        rows, cols = cols, rows

    if rows < 5:
        raise ValueError('输出只有 %d 行，不可能是 4+nc（nc≥1）' % rows)
    num_cls = rows - 4
    if person_class >= 0 and person_class >= num_cls:
        raise ValueError(
            'onnx_person_class=%d 超出模型类别数 %d（模型不是 COCO 的 80 类？）'
            % (person_class, num_cls))

    cx, cy, bw, bh = a[0], a[1], a[2], a[3]
    if person_class >= 0:
        score = a[4 + person_class]
    else:
        score = a[4:].max(axis=0)

    m = score >= float(conf_thr)
    if not m.any():
        return []
    cx, cy, bw, bh, score = cx[m], cy[m], bw[m], bh[m], score[m]

    x1 = (cx - bw * 0.5 - float(dx)) / float(scale)
    y1 = (cy - bh * 0.5 - float(dy)) / float(scale)
    x2 = (cx + bw * 0.5 - float(dx)) / float(scale)
    y2 = (cy + bh * 0.5 - float(dy)) / float(scale)

    # 裁到原图内：padding 区域解出来的负坐标如果直接留着，
    # 下游取深度会索引越界（numpy 负索引是静默绕回，比报错更难查）。
    x1 = np.clip(x1, 0.0, img_w - 1.0)
    y1 = np.clip(y1, 0.0, img_h - 1.0)
    x2 = np.clip(x2, 0.0, img_w - 1.0)
    y2 = np.clip(y2, 0.0, img_h - 1.0)

    boxes = np.stack([x1, y1, x2, y2], axis=1)
    keep = nms(boxes, score, iou_thr)
    out_list = [(float(boxes[i, 0]), float(boxes[i, 1]),
                 float(boxes[i, 2]), float(boxes[i, 3]), float(score[i]))
                for i in keep]
    out_list.sort(key=lambda t: t[4], reverse=True)
    return out_list


def check_onnx_input(shape, want_size):
    """校验 onnx 的输入形状，返回错误说明；没问题返回 None。

    单独抽出来是为了能测：真模型不在仓库里，而这条校验恰恰是
    「坐标整体偏且不报错」的唯一防线，不能只靠肉眼看日志确认。

    shape 里可能是 int，也可能是字符串（动态维度，如 'batch' / 'height'）。
    动态维度按参数 want_size 走，静态维度必须严格等于它。
    """
    shape = list(shape)
    if len(shape) != 4:
        return f'onnx 输入维度是 {shape}，期望 4 维 (N,C,H,W)'
    c = shape[1]
    if isinstance(c, int) and c != 3:
        return f'onnx 输入通道是 {c}，期望 3（RGB/BGR）'
    for axis, label in ((2, '高'), (3, '宽')):
        v = shape[axis]
        if isinstance(v, int) and v != int(want_size):
            return (f'onnx 模型输入{label}度是 {v}，但 onnx_size={want_size}。'
                    f'两者不一致会让所有坐标整体偏，拒绝使用 —— '
                    f'要么把 onnx_size 改成 {v}，要么重新导出 '
                    f'imgsz={want_size} 的模型')
    return None


class PersonDetector(Node):

    def __init__(self):
        super().__init__('lightdog_person_detector')

        # ---- 输入 ----
        self.declare_parameter('rgb_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth/image_rect_raw')
        self.declare_parameter('info_topic', '/camera/color/camera_info')

        # ---- 相机安装偏移（base_link 下的 x,y,z），见文件头注释 ----
        self.declare_parameter('camera_offset', [-0.065, 0.0, 0.255])

        # ---- 检测后端 ----
        # color = 仿真占位实现（认橙色上衣）；yolo = ultralytics（调试方便，
        # 但 Pi 5 上 1~2 FPS，撑不住）；onnx = onnxruntime + 320 输入（真机用这个）。
        # 换后端**不改**输出契约：仍然是 /person/present + /person/position
        # （frame=base_link）。几何换算两边共用，见 _bbox_to_point。
        self.declare_parameter('backend', 'color')
        self.declare_parameter('yolo_model', 'yolov8n.pt')
        self.declare_parameter('yolo_conf', 0.35)
        # COCO 里 person = 0。设 -1 表示不限类别（换自训练模型时用）。
        self.declare_parameter('yolo_person_class', 0)

        # -- onnx 后端 --
        # 这些参数与 yolo_* 是两套：换后端时不用改另一套，也就不会出现
        # 「改了 yolo_conf 却没生效」这种看代码才发现的误会。
        self.declare_parameter('onnx_model', 'yolov8n_320.onnx')
        # 320 是 Pi 5 实测 6~10 FPS 的输入尺寸（hardware_readiness.md §8.5）。
        # 必须和模型的实际输入尺寸一致 —— 加载时会校验，不一致直接回退。
        self.declare_parameter('onnx_size', 320)
        self.declare_parameter('onnx_conf', 0.35)
        self.declare_parameter('onnx_iou', 0.45)
        self.declare_parameter('onnx_person_class', 0)
        # Pi 5 只有 4 核，而 10Hz 的 twist_mux 也在同一颗 CPU 上抢。
        # 留 2 核给控制环，比把推理跑满更划算（推理慢 30%，控制抖 0）。
        self.declare_parameter('onnx_threads', 2)

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
        self.backend = (str(p('backend').value).strip().lower() or 'color')
        self.yolo_conf = float(p('yolo_conf').value)
        self.yolo_person_class = int(p('yolo_person_class').value)
        self._yolo_model = None
        self._warned_yolo = False

        self.onnx_conf = float(p('onnx_conf').value)
        self.onnx_iou = float(p('onnx_iou').value)
        self.onnx_person_class = int(p('onnx_person_class').value)
        self.onnx_size = int(p('onnx_size').value)
        self._onnx = None
        self._onnx_input = None
        self._warned_onnx = False
        self._onnx_ms = []            # 最近的推理耗时（毫秒），用于报平均帧率
        self._onnx_slow_warned = False

        if self.backend == 'yolo':
            self._yolo_model = self._load_yolo(str(p('yolo_model').value))
            if self._yolo_model is None:
                # 回退而不是崩：仿真环境里没有 torch，假装成 yolo 只会让
                # 整个 P2 链路「启动成功但永远无人」。
                self.backend = 'color'
        elif self.backend == 'onnx':
            self._onnx, self._onnx_input = self._load_onnx(
                str(p('onnx_model').value), int(p('onnx_threads').value))
            if self._onnx is None:
                # 和 yolo 分支同一个理由：宁可退回「只认橙色」的显式降级，
                # 也不要「启动成功、日志正常、永远检测不到人」。
                self.backend = 'color'
        elif self.backend != 'color':
            self.get_logger().error(
                f'未知 backend={self.backend!r}（只认 color / yolo / onnx），'
                '按 color 运行')
            self.backend = 'color'
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
            f'backend={self.backend} '
            f'H=[{self.h_min:.3f},{self.h_max:.3f}] S>{self.s_min} V>{self.v_min} '
            f'range=[{self.rng_min},{self.rng_max}]m '
            f'aspect_min={self.aspect_min} min_h={self.min_h}px '
            f'frame_timeout={self.frame_timeout:.1f}s')

    # ---------- 后端加载 ----------

    def _load_yolo(self, path):
        """加载 YOLO 模型；任何失败都返回 None，由调用方回退到 color。

        这里刻意把 import 放在函数里：仿真镜像没装 ultralytics/torch，
        模块级 import 会让整个节点起不来，连「回退到颜色」的机会都没有。
        """
        try:
            from ultralytics import YOLO
        except ImportError as e:
            self.get_logger().error(
                f'backend=yolo 但无法 import ultralytics（{e}）。'
                f'已回退到 color —— 注意颜色判据认的是橙色上衣，'
                f'真机上必须 pip install ultralytics 并准备好模型')
            return None
        try:
            model = YOLO(path)
        except Exception as e:
            self.get_logger().error(
                f'加载 YOLO 模型 {path!r} 失败（{e}），已回退到 color')
            return None
        self.get_logger().info(
            f'检测后端 = YOLO({path}) conf={self.yolo_conf} '
            f'class={self.yolo_person_class}')
        return model

    def _load_onnx(self, path, threads):
        """加载 onnx 模型并**校验输入尺寸**；返回 (session, input_name) 或 (None, None)。

        为什么一定要校验尺寸：letterbox 是按 onnx_size 做的，模型若是 640 而参数
        写 320，推理不报错、框也能解出来，只是所有坐标都差一个缩放比 ——
        表现是「检测得到人，但距离永远不对」，查起来要绕很久。
        宁可启动时就报 error 并回退到颜色（颜色也不对，但那是显式降级）。
        """
        try:
            import onnxruntime as ort
        except ImportError as e:
            self.get_logger().error(
                f'backend=onnx 但无法 import onnxruntime（{e}）。'
                '已回退到 color —— 在 Pi 5 上装 onnxruntime：'
                'pip install onnxruntime（ARM64 有官方 wheel）')
            return None, None

        try:
            opts = ort.SessionOptions()
            # 推理线程数要留余量：10Hz 的 twist_mux 和本节点抢同一颗 CPU，
            # 把 4 核全给推理会让控制环抖动（见 hardware_readiness.md §8.7）。
            opts.intra_op_num_threads = max(1, int(threads))
            opts.inter_op_num_threads = 1
            # 图优化开满：320 输入下能白拿一截，不需要改模型
            opts.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
            sess = ort.InferenceSession(
                path, sess_options=opts, providers=['CPUExecutionProvider'])
        except Exception as e:
            self.get_logger().error(
                f'加载 onnx 模型 {path!r} 失败（{e}），已回退到 color')
            return None, None

        try:
            inp = sess.get_inputs()[0]
            shape = list(inp.shape)
        except Exception as e:
            self.get_logger().error(f'读 onnx 输入定义失败（{e}），已回退到 color')
            return None, None

        # 维度/通道/尺寸一起校验：静态尺寸必须与 onnx_size 一致，
        # 动态尺寸（字符串）按参数走。不一致就拒绝使用并回退。
        err = check_onnx_input(shape, self.onnx_size)
        if err is not None:
            self.get_logger().error(f'{err}，已回退到 color')
            return None, None

        # 预热：onnxruntime 首次 run 要做一次性准备（内存池、算子绑定），
        # Pi 5 上是几百毫秒。不预热的话这段开销落在 10Hz 定时器的**第一帧**，
        # 表现为「刚启动时跟随反应特别慢」，而且只在真机上出现、仿真里复现不了。
        # 顺便用一次全零输入把输出形状和类别数验掉 —— 比等到真有人站在
        # 镜头前才发现 onnx_person_class 越界要好。
        try:
            dummy = np.zeros(
                (1, 3, self.onnx_size, self.onnx_size), dtype=np.float32)
            raw = sess.run(None, {inp.name: dummy})[0]
            decode_yolo_onnx(raw, 1.0, 0, 0, self.onnx_size, self.onnx_size,
                             conf_thr=2.0, iou_thr=self.onnx_iou,
                             person_class=self.onnx_person_class)
        except Exception as e:
            self.get_logger().error(
                f'onnx 预热推理失败（{e}），模型不兼容，已回退到 color')
            return None, None

        self.get_logger().info(
            f'检测后端 = ONNX({path}) 输入={shape} size={self.onnx_size} '
            f'conf={self.onnx_conf} iou={self.onnx_iou} '
            f'class={self.onnx_person_class} threads={threads} '
            f'输出={np.shape(raw)}')
        return sess, inp.name

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
        """返回 (x_fwd, y_left, z_height, bbox) 或 None。bbox=(u0,v0,u1,v1)。

        后端只负责「人的外接框」；从框到 3D 点的几何换算（深度 / 内参 /
        相机到 base_link 的平移）两侧共用。换检测模型时不需要重做那一半，
        也就不会顺手把 frame 语义改错 —— 那是最难查的一类回归。
        """
        rgb = image_to_rgb(self.rgb)
        if rgb is None:
            return None
        if self.backend == 'yolo':
            bbox, mask = self._bbox_yolo(rgb), None
        elif self.backend == 'onnx':
            bbox, mask = self._bbox_onnx(rgb), None
        else:
            bbox, mask = self._bbox_color(rgb)
        if bbox is None:
            return None
        return self._bbox_to_point(rgb, bbox, mask)

    def _bbox_color(self, rgb):
        """颜色判据：返回 (bbox, mask) 或 (None, None)。

        这是**仿真占位实现**（认高饱和橙色上衣），真机请把 backend 设成 yolo。
        接口契约不在这里，而在「发布 base_link 下的人位置」这件事上。
        """
        hh, ss, vv = rgb_to_hsv(rgb)
        mask = ((hh >= self.h_min) & (hh <= self.h_max) &
                (ss >= self.s_min) & (vv >= self.v_min))
        if not mask.any():
            return None, None

        # 列投影找纵向条带（允许小空隙，人身上可能有阴影/遮挡断开）
        col = mask.sum(axis=0)
        cols = np.where(col > 0)[0]
        if cols.size == 0:
            return None, None

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
            return None, None

        u0, u1 = best
        rows = np.where(mask[:, u0:u1 + 1].any(axis=1))[0]
        if rows.size == 0:
            return None, None
        v0, v1 = int(rows[0]), int(rows[-1])

        # 形状约束：站立的人是竖长条。这一条把「球形头」「地上一块橙色杂物」
        # 这类误检挡在外面 —— 仿真里头部肤色渲染后 H/S 都落在橙色区间，
        # 只靠颜色分不开，必须用形状。
        bw = u1 - u0 + 1
        bh = v1 - v0 + 1
        if bh < self.min_h or bh < self.aspect_min * bw:
            if self.publish_debug:
                self._publish_debug(rgb, (u0, v0, u1, v1), False)
            return None, None

        return (u0, v0, u1, v1), mask

    def _bbox_yolo(self, rgb):
        """YOLO 后端：返回人的外接框或 None。

        依赖缺失时**不会**在上电后才发现：__init__ 里已经回退到 color 并打了
        error。这里再判一次是为了防御模型加载后被卸载/推理库半死的情况。
        多个框时取面积最大的（与颜色后端「取最大条带」的行为一致）。
        """
        if self._yolo_model is None:
            return None
        h, w = rgb.shape[:2]
        try:
            # ultralytics 吃 BGR；rgb[:, :, ::-1] 是视图，转成连续内存再进推理，
            # 否则某些版本会在内部拷贝时抛 not contiguous。
            frame = np.ascontiguousarray(rgb[:, :, ::-1])
            res = self._yolo_model.predict(
                frame, conf=self.yolo_conf, verbose=False)
        except Exception as e:                       # 推理库内部异常不该打挂节点
            if not self._warned_yolo:
                self._warned_yolo = True
                self.get_logger().error(f'YOLO 推理失败（{e}），本帧按无人处理')
            return None

        best, best_area = None, 0.0
        for r in (res or []):
            boxes = getattr(r, 'boxes', None)
            if boxes is None:
                continue
            for b in boxes:
                try:
                    cls = int(b.cls[0])
                    x1, y1, x2, y2 = [float(v) for v in b.xyxy[0]]
                except (AttributeError, IndexError, TypeError, ValueError):
                    continue
                if self.yolo_person_class >= 0 and cls != self.yolo_person_class:
                    continue
                area = max(0.0, x2 - x1) * max(0.0, y2 - y1)
                if area <= best_area:
                    continue
                best_area = area
                best = (int(max(0, min(w - 1, x1))), int(max(0, min(h - 1, y1))),
                        int(max(0, min(w - 1, x2))), int(max(0, min(h - 1, y2))))

        if best is None:
            return None
        u0, v0, u1, v1 = best
        # 形状/尺寸下限与颜色后端保持一致：贴脸（只剩一条横带）时别把
        # 「一截腿」当成完整的人。min_h 是像素高度下限，两个后端共用。
        if (u1 - u0 + 1) < 2 or (v1 - v0 + 1) < self.min_h:
            return None
        return best

    def _bbox_onnx(self, rgb):
        """onnxruntime 后端：返回人的外接框或 None。

        与 yolo 后端的分工差别：ultralytics 把预处理（letterbox、BGR、归一化）
        全包了，这里得自己做 —— 所以预处理和还原被抽成了模块级纯函数，
        单独有回归（test_onnx_detector.py）兜着漏乘 scale / 漏减 padding 这类错。
        """
        if self._onnx is None:
            return None
        h, w = rgb.shape[:2]
        t0 = time.monotonic()
        try:
            canvas, scale, dx, dy = letterbox(rgb, self.onnx_size)
            # ultralytics 训练时是 BGR，导出的 onnx 继承了这个约定。
            # 传 RGB 也能跑出框，但分数会低一截 —— 这是「能用但莫名其妙漏检」
            # 的典型来源，所以这里显式反转通道。
            blob = canvas[:, :, ::-1].astype(np.float32) / 255.0
            blob = np.ascontiguousarray(blob.transpose(2, 0, 1)[None])
            raw = self._onnx.run(None, {self._onnx_input: blob})[0]
            dets = decode_yolo_onnx(
                raw, scale, dx, dy, w, h,
                conf_thr=self.onnx_conf, iou_thr=self.onnx_iou,
                person_class=self.onnx_person_class)
        except Exception as e:                       # 推理库内部异常不该打挂节点
            if not self._warned_onnx:
                self._warned_onnx = True
                self.get_logger().error(f'onnx 推理失败（{e}），本帧按无人处理')
            return None
        self._note_onnx_latency((time.monotonic() - t0) * 1000.0)

        if not dets:
            return None
        # 多个框时取面积最大的，与另外两个后端的行为保持一致
        x1, y1, x2, y2, _ = max(
            dets, key=lambda t: max(0.0, t[2] - t[0]) * max(0.0, t[3] - t[1]))
        u0 = int(max(0, min(w - 1, round(x1))))
        v0 = int(max(0, min(h - 1, round(y1))))
        u1 = int(max(0, min(w - 1, round(x2))))
        v1 = int(max(0, min(h - 1, round(y2))))

        # 尺寸下限与另两个后端共用：贴脸（只剩一条横带）时别把「一截腿」
        # 当成完整的人。min_h 的来历见 __init__ 里的注释。
        if (u1 - u0 + 1) < 2 or (v1 - v0 + 1) < self.min_h:
            return None
        return (u0, v0, u1, v1)

    def _note_onnx_latency(self, ms):
        """记录单帧推理耗时，让「够不够快」变成一个能看见的数。

        判据来自 frame_timeout=1.5s：它要求至少 3 FPS（1.5s 内得有新帧，
        且留出余量），所以单帧预算 333ms；希望到 8 FPS 则是 125ms。
        不量这个的话，真机上「有点卡」只能靠感觉，而感觉会怪到相机头上。

        每 200 帧报一次平均（6 FPS 下约 33 秒一条），不刷屏；超过预算时
        立刻 warn 一次，因为那意味着跟随会开始一顿一顿。
        """
        self._onnx_ms.append(ms)
        if not self._onnx_slow_warned and ms > 333.0:
            self._onnx_slow_warned = True
            self.get_logger().warn(
                f'onnx 单帧推理 {ms:.0f}ms，超过 333ms 的预算'
                '（frame_timeout=1.5s 要求 ≥3FPS）。先看是不是 '
                'onnx_threads 被其他节点抢了 CPU，再考虑换 Hailo-8L')
        if len(self._onnx_ms) < 200:
            return
        avg = sum(self._onnx_ms) / float(len(self._onnx_ms))
        self._onnx_ms = []
        fps = 1000.0 / avg if avg > 1e-6 else 0.0
        self.get_logger().info(
            f'onnx 平均推理 {avg:.0f}ms（约 {fps:.1f} FPS）')

    def _bbox_to_point(self, rgb, bbox, mask):
        """外接框 + 深度 -> base_link 下的 3D 点。两个后端共用这一段。"""
        h, w, _ = rgb.shape
        u0, v0, u1, v1 = bbox

        # 距离：掩码像素上的深度中位数（避开背景深度把目标拉远）
        z = None
        dmsg = self.depth
        # 深度也要判新鲜度：彩色在出帧、深度掉了的时候，拿旧深度配新彩色
        # 会把人物的位置算到上一次的位置上（人走动时表现为「距离粘住」）。
        if (dmsg is not None
                and (time.monotonic() - self.depth_wall) <= self.depth_timeout):
            d = image_to_depth(dmsg)
            if d is not None and d.shape == (h, w):
                sub_d = d[v0:v1 + 1, u0:u1 + 1]
                if mask is not None:
                    # 颜色后端：只统计「人身上」的像素，避开框内的背景深度
                    vals = sub_d[mask[v0:v1 + 1, u0:u1 + 1]]
                else:
                    # 模型后端没有掩膜：用整框（框比人略大，中位数仍然稳）
                    vals = sub_d.reshape(-1)
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
