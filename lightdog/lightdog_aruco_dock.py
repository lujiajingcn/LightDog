#!/usr/bin/env python3
"""LightDog 充电底座 ArUco 视觉对准（P3）

## 它解决的问题

Nav2 `opennav_docking` 的 `SimpleChargingDock` 在
`use_external_detection_pose: true` 时，会订阅话题 `detected_dock_pose`，
用外部视觉给出的「桩相对机器人」位姿去精化停靠点。本节点就是那个外部视觉。

## 输出

    /detected_dock_pose   geometry_msgs/PoseStamped
                          frame_id = base_footprint
                          语义 = **机器人停靠时应该在的位姿**（opennav_docking 的
                          dock pose 语义），不是 marker 的位姿、也不是桩的位姿。
                          marker 安装变换 + 「桩->停靠位姿」的偏移都在本节点内部
                          用矩阵补偿掉，见 marker_pos / approach_offset。
    /dock/marker_visible  std_msgs/Bool   当前帧是否看到 marker（调试用）
    /dock/debug_image     sensor_msgs/Image 画了识别框的画面（可选）

为什么不把 marker 位姿直接发出去、再让 SimpleChargingDock 用
`external_detection_translation_x` 去补偏移：那个偏移只支持 x/y 平移和
roll/pitch/yaw，而我们的 marker 贴在前挡板内侧、法向朝外，需要的变换既有平移
又有 90°/90° 的轴向旋转，用那几个参数表达会绕成一团。在本节点里用矩阵一次算完，
SimpleChargingDock 那边的 external_detection_* 全部置零，语义最干净。

## 坐标系（改之前务必确认）

    图像（光学系）  x 右 / y 下 / z 前       —— OpenCV solvePnP 的约定
    camera_link     x 前 / y 左 / z 上       —— URDF（rpy 恒为 0，与 base_link 同向）
    两者差 R = Rz(-90°) · Rx(-90°)

    dock frame      +x 朝外（朝机器人来车方向）/ +y 左 / +z 上
    marker 在 dock 上的安装：
        位置 (-0.0775, 0, 0.155)   —— 后挡板前表面，中心高 0.155m
        朝向 rpy (90°, 0, 90°)     —— marker 的 +z 法向 → dock +x
                                     marker 的 +x（图像右）→ dock +y

这两个数直接对应 lightdog/models/dock/model.sdf 里的 marker_visual pose。
改桩的几何就必须同步改这里，否则解出来的 yaw 会整体差 90° 或 180°。

## 为什么不用 TF

和 lightdog_person_detector 一样：相机安装是静态的（rpy=0），用参数补偿比拉
tf2 少一个失败模式；P2 已经验证这条路可靠。真机上若换成可转动的云台，再改成
tf2 查询。
"""

import math
import sys
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Bool
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped

try:
    import cv2
except ImportError:  # pragma: no cover
    print('需要 opencv（镜像里的 python3-opencv 自带 contrib/aruco）', file=sys.stderr)
    raise


# ---------- 小工具 ----------

def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=np.float64)


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=np.float64)


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=np.float64)


def rpy_matrix(roll, pitch, yaw):
    """REP-103: R = Rz(yaw) · Ry(pitch) · Rx(roll)。"""
    return rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)


def make_tf(rotation, translation):
    t = np.eye(4, dtype=np.float64)
    t[:3, :3] = rotation
    t[:3, 3] = translation
    return t


def matrix_to_quat(m):
    """3x3 旋转矩阵 -> (x, y, z, w)，用 Shepperd 法，避免除法病态。"""
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    return x, y, z, w


def image_to_gray(msg: Image):
    """sensor_msgs/Image -> (H, W) uint8 灰度；无法识别返回 None。"""
    h, w = msg.height, msg.width
    enc = msg.encoding.lower()
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    if enc in ('rgb8', 'bgr8'):
        if buf.size < h * w * 3:
            return None
        a = buf[:h * w * 3].reshape(h, w, 3)
        if enc == 'rgb8':
            # 加权和近似，省一次通道重排
            return (0.299 * a[:, :, 0] + 0.587 * a[:, :, 1] + 0.114 * a[:, :, 2]).astype(np.uint8)
        return (0.299 * a[:, :, 2] + 0.587 * a[:, :, 1] + 0.114 * a[:, :, 0]).astype(np.uint8)
    if enc in ('rgba8', 'bgra8'):
        if buf.size < h * w * 4:
            return None
        a = buf[:h * w * 4].reshape(h, w, 4)
        if enc == 'rgba8':
            return (0.299 * a[:, :, 0] + 0.587 * a[:, :, 1] + 0.114 * a[:, :, 2]).astype(np.uint8)
        return (0.299 * a[:, :, 2] + 0.587 * a[:, :, 1] + 0.114 * a[:, :, 0]).astype(np.uint8)
    if enc == 'mono8':
        if buf.size < h * w:
            return None
        return buf[:h * w].reshape(h, w).copy()
    return None


def rgb_image(msg: Image):
    """sensor_msgs/Image -> (H, W, 3) uint8 RGB，仅调试画框用。"""
    h, w = msg.height, msg.width
    if h <= 0 or w <= 0:
        return None
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


class ArucoDockDetector(Node):

    def __init__(self):
        super().__init__('lightdog_aruco_dock')

        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('info_topic', '/camera/color/camera_info')
        self.declare_parameter('publish_debug', True)

        # marker 本体：黑框边长（米）。必须与 model.sdf 里贴图的实际尺寸一致：
        # 白板 0.14m 装 10 个 cell（6 个 marker cell + 每边 2 个静默 cell），
        # 所以黑框 = 0.14 * 6/10 = 0.084m。填错会让距离整体按比例缩放。
        self.declare_parameter('marker_length', 0.084)
        self.declare_parameter('marker_id', 0)
        self.declare_parameter('dictionary', 'DICT_4X4_50')

        # 相机安装（与 person_detector 一致：4wd 的 depth_sensor_pose）
        self.declare_parameter('camera_offset', [-0.065, 0.0, 0.255])
        # 相机安装位置是相对 base_link 给的，而我们发布在 base_footprint 下。
        # 4wd 的 base_link 离地 0.1195m（wheel_pos_z -0.04788 + wheel_radius 0.07162
        # 反向推算），漏掉这一项会让解出来的 dock z 恒偏 -0.1195。
        # 实测：补与不补，x/y 都不受影响（旋转链里 z 平移只进画面竖直方向）。
        self.declare_parameter('base_link_height', 0.1195)
        self.declare_parameter('optical_rpy_deg', [-90.0, 0.0, -90.0])

        # marker 在 dock 上的安装
        self.declare_parameter('marker_pos', [-0.0775, 0.0, 0.155])
        self.declare_parameter('marker_rpy_deg', [90.0, 0.0, 90.0])
        # 从桩原点沿桩 +x（朝外）到「机器人中心该停的位置」的距离。
        # opennav_docking 的 dock pose 语义是**机器人停靠位姿**，不是桩的位姿，
        # 所以这里要把桩位姿再前移这么多、并转 180°（机器人正对桩开进去）。
        # 0.30 来自桩的几何：后挡板前表面 0.080 + marker 板 0.004 + 车头前缘 0.2225
        # ≈ 0.3065，取 0.30 留 7mm 让物理碰撞先发生。
        self.declare_parameter('approach_offset', 0.30)

        self.declare_parameter('rate', 10.0)
        self.declare_parameter('max_range', 3.0)
        self.declare_parameter('invert_marker_yaw', False)

        p = self.get_parameter
        self.marker_length = float(p('marker_length').value)
        self.marker_id = int(p('marker_id').value)
        dict_name = str(p('dictionary').value)
        self.publish_debug = bool(p('publish_debug').value)
        self.rate = float(p('rate').value)
        self.max_range = float(p('max_range').value)

        cam_off = [float(v) for v in p('camera_offset').value]
        cam_off[2] += float(p('base_link_height').value)   # 相对 base_link -> base_footprint
        opt_rpy = [math.radians(float(v)) for v in p('optical_rpy_deg').value]
        mk_pos = [float(v) for v in p('marker_pos').value]
        mk_rpy = [math.radians(float(v)) for v in p('marker_rpy_deg').value]
        if bool(p('invert_marker_yaw').value):
            mk_rpy[2] += math.pi

        # T_base_camera_link：平移补偿，旋转恒等（URDF 里 rpy=0）
        t_base_cam = make_tf(np.eye(3), np.array(cam_off))
        # T_camera_link_optical：REP-103 光学系
        t_cam_opt = make_tf(rpy_matrix(*opt_rpy), np.zeros(3))
        self.t_base_opt = t_base_cam @ t_cam_opt
        # T_marker_dock：marker 安装变换的逆（marker -> dock）
        t_dock_marker = make_tf(rpy_matrix(*mk_rpy), np.array(mk_pos))
        self.t_marker_dock = np.linalg.inv(t_dock_marker)
        # T_dock_dockpose：桩位姿 -> 机器人停靠位姿（前移 approach、掉头 180°）
        approach = float(p('approach_offset').value)
        self.t_dock_dockpose = make_tf(rot_z(math.pi), np.array([approach, 0.0, 0.0]))

        # ArUco
        if not hasattr(cv2, 'aruco'):
            raise RuntimeError('这个 opencv 没编译 aruco 模块（需要 opencv-contrib）')
        self.dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))
        self.detector_params = cv2.aruco.DetectorParameters_create() \
            if hasattr(cv2.aruco, 'DetectorParameters_create') else cv2.aruco.DetectorParameters()
        # OpenCV >= 4.7 推荐 ArucoDetector 对象；老的 detectMarkers(gray, dict,
        # parameters=...) 在新版本里签名已经变了（apt 一升级就静默失效）。
        # 有 ArucoDetector 就用它，没有才退回老接口 —— 两条路返回值都是
        # (corners, ids, rejected)。
        self._aruco = None
        if hasattr(cv2.aruco, 'ArucoDetector'):
            try:
                self._aruco = cv2.aruco.ArucoDetector(self.dictionary, self.detector_params)
            except Exception:
                self._aruco = None
        if self._aruco is None:
            self.get_logger().warn(
                '当前 opencv 没有 ArucoDetector（<4.7），退回 detectMarkers 老接口')

        half = self.marker_length / 2.0
        self.obj_points = np.array([
            [-half, half, 0.0],
            [half, half, 0.0],
            [half, -half, 0.0],
            [-half, -half, 0.0],
        ], dtype=np.float64)

        self.camera_matrix = None
        self.dist = None
        self.n_seen = 0
        self.n_frames = 0

        # 最新一帧 + 处理状态。检测放在定时器里跑（rate Hz），不在图像回调里跑：
        # 回调里做 detectMarkers + solvePnP，30fps 下会吃满一个 ARM 核，
        # 而且 `rate` 参数形同虚设。对准只需要 10Hz。
        self.latest = None
        self.latest_wall = 0.0
        self._last_processed = None      # 上一帧的 header.stamp，避免重复检测
        self.visible = False

        sensor_qos = QoSProfile(depth=1,
                                reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST)

        self.create_subscription(Image, str(p('image_topic').value), self._on_image, sensor_qos)
        self.create_subscription(CameraInfo, str(p('info_topic').value), self.on_info, sensor_qos)

        self.pose_pub = self.create_publisher(PoseStamped, '/detected_dock_pose', 5)
        self.vis_pub = self.create_publisher(Bool, '/dock/marker_visible', 5)
        self.dbg_pub = self.create_publisher(Image, '/dock/debug_image', 2) if self.publish_debug else None

        self.create_timer(1.0 / max(1.0, self.rate), self._tick)

        self.get_logger().info(
            f'ArUco dock detector 就绪：{dict_name} id={self.marker_id} '
            f'边长 {self.marker_length:.3f}m，marker->dock rpy '
            f'{[round(math.degrees(v), 1) for v in mk_rpy]}')

    # ---------- 回调 ----------

    def on_info(self, msg: CameraInfo):
        if len(msg.k) < 9:
            return
        k = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        d = np.array(msg.d, dtype=np.float64) if len(msg.d) > 0 else np.zeros(5)
        self.camera_matrix = k
        self.dist = d

    def _on_image(self, msg: Image):
        """只收帧，不做检测（检测在 _tick 里按 rate 跑）。"""
        self.latest = msg
        self.latest_wall = time.monotonic()

    def _detect_markers(self, gray):
        if self._aruco is not None:
            return self._aruco.detectMarkers(gray)
        return cv2.aruco.detectMarkers(gray, self.dictionary,
                                       parameters=self.detector_params)

    def _tick(self):
        # 定时器回调里抛异常会直接把 rclpy 的 spin 打断（节点假死）。
        # 一帧坏数据不该让回充彻底失效，所以在这里兜一层。
        try:
            self._tick_impl()
        except Exception as e:
            self.get_logger().warn(f'ArUco 检测异常：{e}', throttle_duration_sec=10.0)

    def _tick_impl(self):
        msg = self.latest
        if msg is None or self.camera_matrix is None:
            return
        # 相机断流时不能让 marker_visible 一直停在 true：对准逻辑会以为
        # 桩还在视野里，于是原地微调、永远不进入扫描。
        if (time.monotonic() - self.latest_wall) > 1.0:
            if self.visible:
                self.visible = False
                self.vis_pub.publish(Bool(data=False))
            return

        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if stamp == self._last_processed:
            return                      # 没有新帧，别重复算
        self._last_processed = stamp

        gray = image_to_gray(msg)
        if gray is None:
            return

        self.n_frames += 1
        corners, ids, _ = self._detect_markers(gray)

        visible = False
        if ids is not None and len(ids) > 0:
            ids = ids.flatten()
            for idx, mid in enumerate(ids):
                if int(mid) != self.marker_id:
                    continue
                ok, rvec, tvec = cv2.solvePnP(
                    self.obj_points, corners[idx].reshape(4, 2).astype(np.float64),
                    self.camera_matrix, self.dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
                if not ok:
                    continue
                rmat, _ = cv2.Rodrigues(rvec)
                t_opt_marker = make_tf(rmat, np.array(tvec, dtype=np.float64).reshape(3))
                t_base_dock = self.t_base_opt @ t_opt_marker @ self.t_marker_dock
                # 转成 docking 期望的语义：机器人停靠时的位姿
                t_base_pose = t_base_dock @ self.t_dock_dockpose

                dist = float(np.linalg.norm(t_opt_marker[:3, 3]))
                if dist > self.max_range:
                    continue

                pose = PoseStamped()
                pose.header.stamp = msg.header.stamp
                pose.header.frame_id = 'base_footprint'
                pose.pose.position.x = float(t_base_pose[0, 3])
                pose.pose.position.y = float(t_base_pose[1, 3])
                pose.pose.position.z = float(t_base_pose[2, 3])
                qx, qy, qz, qw = matrix_to_quat(t_base_pose[:3, :3])
                pose.pose.orientation.x = qx
                pose.pose.orientation.y = qy
                pose.pose.orientation.z = qz
                pose.pose.orientation.w = qw
                self.pose_pub.publish(pose)
                visible = True
                self.n_seen += 1
                break

        self.vis_pub.publish(Bool(data=visible))
        self.visible = visible

        if self.dbg_pub is not None and self.n_frames % 3 == 0:
            rgb = rgb_image(msg)
            if rgb is not None:
                if ids is not None:
                    cv2.aruco.drawDetectedMarkers(rgb, corners, ids)
                out = Image()
                out.header = msg.header
                out.height, out.width = rgb.shape[0], rgb.shape[1]
                out.encoding = 'rgb8'
                out.is_bigendian = 0
                out.step = out.width * 3
                out.data = rgb.tobytes()
                self.dbg_pub.publish(out)

    def report(self):
        if self.n_frames:
            self.get_logger().info(
                f'识别率 {self.n_seen}/{self.n_frames} = {self.n_seen / self.n_frames:.1%}')


def main():
    rclpy.init()
    node = ArucoDockDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.report()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
