#!/usr/bin/env python3
"""
LightDog P5：跌倒检测（姿态判定）

    订阅 /camera/color/image_raw        sensor_msgs/Image
    订阅 /camera/depth/image_rect_raw   sensor_msgs/Image（depth_mode=off 时不订阅）
    订阅 /camera/color/camera_info      sensor_msgs/CameraInfo
    发布 /fall/state        std_msgs/String   standing | lying | absent | unknown
    发布 /fall/z_p95        std_msgs/Float32  人体点云离地高度的 95 分位（米）
    发布 /fall/lying        std_msgs/Bool
    发布 /fall/debug_image  sensor_msgs/Image

## 判据为什么是「离地高度」而不是「宽高比」

最直观的写法是看包围盒：站着是竖条、躺着是横条。但宽高比会被距离毁掉 ——
人走到 0.8m 时相机只能看到躯干中段，bbox 接近正方形；蹲下系鞋带也是横的。
靠 bbox 做跌倒检测，误报率高到没法用。

这里用的是一个物理量：**人体点云最高点离地多少米**。

相机装在 base_link 前 0.065m、高 0.255m，base_link 离地 0.1195m，所以光心离地
`0.3745m`，光轴水平朝前。已知每个像素的深度 `d`，就能把像素反投影回相机坐标系，
再换到「离地高度」：

    光学系（x 右 / y 下 / z 前）：  x = (u-cx)/fx*d ,  y = (v-cy)/fy*d ,  z = d
    base_link（x 前 / y 左 / z 上）： x_b = z ,  y_b = -x ,  z_b = -y
    离地高度：                       h = 0.3745 + z_b

站着的人：点在 0~1.6m 铺开（视野只能看到上半身，但下端一定扫到地面附近），
          `z_p95` 通常 > 0.75。
躺着的人：整体压在 0~0.35m，`z_p95` < 0.5。
蹲着的人：`z_p95` 介于两者之间 → 判 `unknown`，**不报警**。

用 95 分位而不是最大值：深度图边缘有飞点，`max` 会被一两个坏点顶到天上。

## 为什么不用点云话题

`/camera/depth/color/points` 是 `sensor_msgs/PointCloud2`，解析要自己啃
fields/offsets/data，而深度图 + camera_info 已经够算这个高度了，少一层不确定。

## 与 P2 检测器的关系

颜色判据（橙色上衣）整套沿用 `lightdog_person_detector.py`，但**去掉**
`aspect_min` 约束 —— P2 那个约束是「只认竖着的人」（为了跟随不跟错目标），
而 P5 恰恰要能看见躺着的人。两个节点订阅同一路图像，互不影响。
"""

import math
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import cv2
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, Float32, String


def rgb_to_hsv(rgb: np.ndarray):
    """(H, W, 3) uint8 RGB -> h(0~1), s(0~1), v(0~1)。纯 numpy，与 P2 检测器一致。"""
    a = rgb.astype(np.float32) / 255.0
    r, g, b = a[:, :, 0], a[:, :, 1], a[:, :, 2]
    mx = a.max(axis=2)
    mn = a.min(axis=2)
    delta = mx - mn
    h = np.zeros_like(mx)
    nz = delta > 1e-6
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


def image_to_rgb(msg: Image):
    h, w = msg.height, msg.width
    if h <= 0 or w <= 0:
        return None
    enc = msg.encoding.lower()
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    if enc in ('rgb8', 'bgr8'):
        if buf.size < h * w * 3:
            return None
        a = buf[:h * w * 3].reshape(h, w, 3)
        return a.copy() if enc == 'rgb8' else a[:, :, ::-1].copy()
    # 很多 USB / RealSense 驱动给的是 4 通道。与 P2 检测器保持同一套解码，
    # 否则同一路图像 P2 看得见、P5 却一条 /fall/state 都不发。
    if enc in ('rgba8', 'bgra8'):
        if buf.size < h * w * 4:
            return None
        a = buf[:h * w * 4].reshape(h, w, 4)[:, :, :3]
        return a.copy() if enc == 'rgba8' else a[:, :, ::-1].copy()
    return None


def image_to_depth(msg: Image):
    """深度图 -> (H, W) float32 米；16UC1 按毫米折算。"""
    h, w = msg.height, msg.width
    enc = msg.encoding.lower()
    if enc == '32fc1':
        if len(msg.data) < h * w * 4:
            return None
        return np.frombuffer(msg.data, dtype=np.float32)[:h * w].reshape(h, w).copy()
    if enc in ('16uc1', 'mono16'):
        if len(msg.data) < h * w * 2:
            return None
        return (np.frombuffer(msg.data, dtype=np.uint16)[:h * w]
                .reshape(h, w).astype(np.float32) / 1000.0)
    return None


def normalize_fall_mode(raw):
    """把 depth_mode 收敛成 on / off，返回 (mode, 错误说明或 None)。

    off 不是「跳过检测」，而是「明确宣告本节点不可用」：持续发 unknown，
    让告警状态机知道「我看不见」，而不是让 /fall/state 彻底沉默。

    YAML 1.1 会把裸写的 on/off 解析成 bool（off -> False），那样停用会
    静默失效。所以配置里必须给值加引号，这里也对 bool 给出明确诊断。
    """
    if isinstance(raw, bool):
        # 裸写 off -> False。这里必须按布尔值映射，不能一律返回 'on'：
        # 前者只是「配置写法待修」，后者会把运维明确要求的停用反转成启用，
        # 节点开始等一台根本不存在的深度相机。
        mode = 'on' if raw else 'off'
        return mode, (f'YAML 裸写 on/off 会被解析成布尔 {raw!r}，已按 {mode} 运行；'
                      "请给值加引号，例如 depth_mode: 'off'")
    mode = str(raw).strip().lower()
    if mode in ('on', 'off'):
        return mode, None
    return 'on', f'未知 depth_mode={raw!r}（只认 on / off），按 on 运行'


class FallDetector(Node):

    def __init__(self):
        super().__init__('lightdog_fall_detector')

        self.declare_parameter('rgb_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth/image_rect_raw')
        self.declare_parameter('info_topic', '/camera/color/camera_info')

        # 相机装车几何：base_link 到光心 0.255m，base_link 离地 0.1195m（4wd）
        self.declare_parameter('camera_offset', [-0.065, 0.0, 0.255])
        self.declare_parameter('base_link_height', 0.1195)

        # 人身上衣的 HSV（与 P2 检测器同一套实测值）
        self.declare_parameter('h_min', 0.012)
        self.declare_parameter('h_max', 0.090)
        self.declare_parameter('s_min', 0.45)
        self.declare_parameter('v_min', 0.15)
        self.declare_parameter('min_points', 200)     # 少于这么多像素当"无人"
        self.declare_parameter('range_min', 0.30)
        self.declare_parameter('range_max', 8.00)

        # 判据阈值（米，离地高度）。
        #
        # 这两个阈值之间是一片**空的**区间，不是拍脑袋划的：
        # 橙色躯干的下沿在 0.72m、上沿 1.47m，所以**只要能看见站着的躯干**，
        # z_p95 就必然 >= 0.72；而躺倒时躯干整根压在 0~0.34m。
        # 0.50 / 0.68 正好落在 (0.34, 0.72) 这个空档里，两边都有余量。
        self.declare_parameter('lying_top_max', 0.50)
        self.declare_parameter('stand_top_min', 0.68)

        self.declare_parameter('rate', 10.0)
        self.declare_parameter('absent_timeout', 1.5)   # 这么久看不到人 -> absent
        # 帧新鲜度（**墙钟**秒）。与 P2 检测器同一个理由：没有它，相机冻帧/断线后
        # _tick 会以 10Hz 对同一张旧图反复判定，永远得出同一个结论。
        # 对跌倒检测尤其危险：
        #   · 冻在「站着」-> 人真的倒下了也永远报 standing，告警永远不触发；
        #   · 冻在「躺着」-> 人早起来了还在 ALARM，半夜把全家叫醒。
        # 超时后改报 unknown（不是 absent）：告警状态机据此知道「看不见」，
        # 不会把它误当成「人站起来了」而撤销告警。
        self.declare_parameter('frame_timeout', 1.5)
        self.declare_parameter('depth_timeout', 1.0)
        self.declare_parameter('publish_debug', True)
        # on  ：深度相机在位，正常跑 z_p95（默认）
        # off ：普通彩色相机 / 已改由外部传感器（如毫米波雷达）接管 /fall/state。
        #       本节点不订阅深度，持续发 unknown，绝不静默装死。
        #       这个节点**没有单目降级路径** —— z_p95 本身就要逐像素深度。
        self.declare_parameter('depth_mode', 'on')

        p = self.get_parameter
        self.cam_off = list(p('camera_offset').value)
        self.base_h = float(p('base_link_height').value)
        self.h_min = float(p('h_min').value)
        self.h_max = float(p('h_max').value)
        self.s_min = float(p('s_min').value)
        self.v_min = float(p('v_min').value)
        self.min_points = int(p('min_points').value)
        self.rng_min = float(p('range_min').value)
        self.rng_max = float(p('range_max').value)
        self.lying_top_max = float(p('lying_top_max').value)
        self.stand_top_min = float(p('stand_top_min').value)
        self.absent_timeout = float(p('absent_timeout').value)
        self.frame_timeout = float(p('frame_timeout').value)
        self.depth_timeout = float(p('depth_timeout').value)
        self.publish_debug = bool(p('publish_debug').value)
        self.depth_mode, err = normalize_fall_mode(p('depth_mode').value)
        if err is not None:
            self.get_logger().error(err)
        self._warned_no_depth = False

        # 光心离地高度 = base_link 高度 + 相机在 base_link 上的 z 偏移
        self.cam_z_floor = self.base_h + float(self.cam_off[2])

        self.fx = self.cx = self.fy = self.cy = None
        self.rgb = None
        self.depth = None
        self.last_seen = None

        # 帧序号 + 到达时刻（墙钟）：只对**新帧**做一次检测，并能在相机断流时
        # 明确报 unknown。没有这两个量，_tick 会以 10Hz 反复解同一张旧图。
        self.rgb_seq = 0
        self._det_seq = -1
        self.rgb_wall = 0.0
        self.depth_wall = 0.0
        self._warned_stall = False
        self._warned_shape = False
        self._warned_depth = False
        self._warned_decode = False
        self._last_state = None       # 上一次的结论，无新帧时重发
        self._last_z = None
        self._last_spread = None

        sensor_qos = QoSProfile(depth=1,
                                reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Image, str(p('rgb_topic').value), self._on_rgb, sensor_qos)
        if self.depth_mode != 'off':
            self.create_subscription(
                Image, str(p('depth_topic').value), self._on_depth, sensor_qos)
        self.create_subscription(CameraInfo, str(p('info_topic').value), self._on_info, sensor_qos)

        self.pub_state = self.create_publisher(String, '/fall/state', 10)
        self.pub_z = self.create_publisher(Float32, '/fall/z_p95', 10)
        self.pub_lying = self.create_publisher(Bool, '/fall/lying', 10)
        self.pub_dbg = None
        if self.publish_debug:
            self.pub_dbg = self.create_publisher(Image, '/fall/debug_image', 2)

        self.create_timer(1.0 / max(1.0, float(p('rate').value)), self._tick)

        self.get_logger().info(
            f'LightDog fall detector ready: depth_mode={self.depth_mode} '
            f'camera_z={self.cam_z_floor:.3f}m '
            f'lying(z_p95<{self.lying_top_max:.2f}) stand(z_p95>{self.stand_top_min:.2f}) '
            f'min_pts={self.min_points}')
        if self.depth_mode == 'off':
            self.get_logger().error(
                '跌倒检测已停用（depth_mode=off）：普通相机算不出 z_p95。'
                '本节点将持续发布 /fall/state=unknown，不会误报 standing/lying。'
                '接外部传感器时直接让它发布 /fall/state 即可，无需改本节点')

    # ---------- 回调 ----------

    def _on_info(self, msg: CameraInfo):
        if msg.k is None or len(msg.k) < 9:
            return
        fx, fy = float(msg.k[0]), float(msg.k[4])
        if not (math.isfinite(fx) and fx > 0.0 and math.isfinite(fy) and fy > 0.0):
            return
        self.fx, self.fy = fx, fy
        self.cx, self.cy = float(msg.k[2]), float(msg.k[5])

    def _on_rgb(self, msg: Image):
        self.rgb = msg
        self.rgb_seq += 1
        self.rgb_wall = time.monotonic()

    def _on_depth(self, msg: Image):
        self.depth = msg
        self.depth_wall = time.monotonic()

    # ---------- 主循环 ----------

    def _tick(self):
        # depth_mode=off：普通相机算不了 z_p95。这里**必须持续发 unknown**，
        # 不能 return —— 原来是 `if self.depth is None: return`，结果是节点
        # 健康地跑着、systemd 显示 active、/fall/state 一条消息都不发，
        # 告警状态机连「传感器坏了」和「人好好的」都分不出来。
        if self.depth_mode == 'off':
            self._emit_unknown()
            return

        if self.depth is None and not self._warned_no_depth:
            self._warned_no_depth = True
            self.get_logger().error(
                'depth_mode=on 但一次深度图都没收到：普通相机？align_depth 没开？'
                'depth_topic 写错？跌倒检测改发 unknown。'
                '若无深度相机请设 depth_mode: off')
        if self.rgb is None or self.depth is None or self.fx is None:
            # 缺输入是「我看不见」，不是「没人」，必须发 unknown。
            self._emit_unknown()
            return

        now_mono = time.monotonic()

        # 1) 彩色断流：绝不能拿最后一帧反复判定（见 frame_timeout 的注释）
        if (now_mono - self.rgb_wall) > self.frame_timeout:
            if not self._warned_stall:
                self._warned_stall = True
                self.get_logger().error(
                    f'相机 {now_mono - self.rgb_wall:.1f}s 没有新帧，跌倒检测改为 '
                    f'unknown（检查相机/线缆/USB 带宽）。告警状态机会保持当前判断，'
                    f'不会把它当成「人已起身」而撤销告警。')
            self._emit_unknown()
            return
        if self._warned_stall:
            self._warned_stall = False
            self.get_logger().info('相机图像恢复，跌倒检测继续')

        # 2) 没有新帧：按 rate 重发上次结论。
        #    alert_manager 用 lying_gap（默认 1s）判"还在躺着"，不重发会被当成起身。
        if self.rgb_seq == self._det_seq:
            self._republish()
            return

        # 3) 深度断流：不要拿旧深度配新彩色。人走动时旧深度会把位置算在上一次
        #    的地方（"距离粘住"），算出来的离地高度跟着错。
        if (now_mono - self.depth_wall) > self.depth_timeout:
            if not self._warned_depth:
                self._warned_depth = True
                self.get_logger().error(
                    f'深度图 {now_mono - self.depth_wall:.1f}s 没有新帧，算不了离地高度，'
                    f'本帧判 unknown（不拿旧深度配新彩色）')
            self._emit_unknown()
            return
        if self._warned_depth:
            self._warned_depth = False
            self.get_logger().info('深度图恢复')

        self._det_seq = self.rgb_seq
        rgb = image_to_rgb(self.rgb)
        dep = image_to_depth(self.depth)
        if rgb is None or dep is None:
            # 解码失败和「没收到图」一样是「看不见」，必须发 unknown。
            # 原先这里直接 return：节点健康、systemd active，但 /fall/state
            # 一条都不发，告警状态机分不清「没人」和「传感器坏了」。
            if not self._warned_decode:
                self._warned_decode = True
                self.get_logger().error(
                    f'图像编码无法解码（rgb={getattr(self.rgb, "encoding", "?")} '
                    f'depth={getattr(self.depth, "encoding", "?")}）。'
                    '彩色只认 rgb8/bgr8/rgba8/bgra8，深度只认 32fc1/16uc1/mono16；'
                    '跌倒检测改发 unknown')
            self._emit_unknown()
            return
        if self._warned_decode:
            self._warned_decode = False
            self.get_logger().info('图像编码恢复可解码，跌倒检测继续')

        # 深度图和彩色图必须同尺寸。仿真里 gz 的 rgbd_camera 是对齐的；
        # 真机 RealSense 的 /camera/depth/image_rect_raw 默认 848x480，与彩色
        # 640x480 不同 —— 必须在驱动侧开 align_depth，或改订阅
        # /camera/aligned_depth_to_color/image_raw。这里明说，不静默失败。
        if rgb.shape[:2] != dep.shape[:2]:
            if not self._warned_shape:
                self._warned_shape = True
                self.get_logger().error(
                    f'彩色/深度尺寸不一致 {rgb.shape[:2]} vs {dep.shape[:2]}：'
                    f'真机需在 realsense2_camera 开 align_depth:=true，'
                    f'或把 depth_topic 改成 /camera/aligned_depth_to_color/image_raw')
            self._emit_unknown()
            return

        mask, n_px = self._person_mask(rgb, dep)
        H, W = dep.shape[:2]

        if n_px < self.min_points:
            self._emit('absent', None, None, mask, rgb)
            return

        vs, us = np.nonzero(mask)
        d = dep[vs, us]
        # 反投影：光学系 y 向下，取负号变"向上"，再加上光心离地高度
        z_floor = self.cam_z_floor - (vs.astype(np.float32) - self.cy) / self.fy * d

        z_p05 = float(np.percentile(z_floor, 5))
        z_p95 = float(np.percentile(z_floor, 95))
        spread = z_p95 - z_p05

        if z_p95 <= self.lying_top_max:
            # 只有「整片点都很低」才算躺倒。
            # 注意不能用 z_p05 或 z_min：站得近时画面上沿已经切到躯干，z_min 也在
            # 0.7 附近，但蹲下的人 z_p95 会落在 0.4~0.7 —— 那属于 unknown，
            # 不报警，交给后面的两级确认去问。
            state = 'lying'
        elif z_p95 >= self.stand_top_min:
            state = 'standing'
        else:
            state = 'unknown'

        self._emit(state, z_p95, spread, mask, rgb)

    def _person_mask(self, rgb, dep):
        h, s, v = rgb_to_hsv(rgb)
        m = (h >= self.h_min) & (h <= self.h_max) & (s >= self.s_min) & (v >= self.v_min)
        m &= (dep >= self.rng_min) & (dep <= self.rng_max)
        n = int(np.count_nonzero(m))
        return m, n

    def _emit_unknown(self):
        """相机/深度不可用：明确报 unknown，且**不**刷新 last_seen。

        为什么不报 absent：absent 的意思是「确定画面里没人」，而这里是
        「我不知道」。两者对告警状态机含义相反 —— absent 可能是人走了，
        不可用则意味着可能有人躺着但看不见。
        """
        self._publish_state('unknown', None)
        self._last_state = 'unknown'
        self._last_z = None
        self._last_spread = None

    def _republish(self):
        """没有新帧时重发上次结论（只发状态类话题，不重复编码调试图）。"""
        self._publish_state(self._last_state or 'unknown', self._last_z)

    def _publish_state(self, state, z_p95):
        msg = String()
        msg.data = state
        self.pub_state.publish(msg)

        if z_p95 is not None:
            m = Float32()
            m.data = float(z_p95)
            self.pub_z.publish(m)

        b = Bool()
        b.data = (state == 'lying')
        self.pub_lying.publish(b)

    def _emit(self, state, z_p95, spread, mask, rgb):
        now = self.get_clock().now()

        if state == 'absent':
            # 短暂丢帧不算"人不见了"：相机抖动、人侧身、颜色判据断一两帧都正常。
            # 在 absent_timeout 之内报 unknown，超时才报 absent —— 两者对告警
            # 状态机都是"非躺倒"，区别只在日志里能看出是"人走了"还是"看不见"。
            if self.last_seen is not None:
                age = (now - self.last_seen).nanoseconds * 1e-9
                if age <= self.absent_timeout:
                    state = 'unknown'
        else:
            self.last_seen = now

        self._publish_state(state, z_p95)
        self._last_state = state
        self._last_z = z_p95
        self._last_spread = spread

        if self.pub_dbg is not None and mask is not None:
            self._publish_debug(rgb, mask, state, z_p95, spread)

    def _publish_debug(self, rgb, mask, state, z_p95, spread):
        vis = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        vis[mask] = (0.35 * vis[mask] + 0.65 * np.array([0, 0, 255])).astype(np.uint8)
        color = {'lying': (0, 0, 255), 'standing': (0, 200, 0),
                 'unknown': (0, 180, 255)}.get(state, (160, 160, 160))
        cv2.putText(vis, state, (8, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
        if z_p95 is not None:
            cv2.putText(vis, f'z95={z_p95:.2f}m span={spread:.2f}m',
                        (8, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
        out = Image()
        out.header = self.rgb.header
        out.height, out.width = vis.shape[0], vis.shape[1]
        out.encoding = 'bgr8'
        out.is_bigendian = 0
        out.step = vis.shape[1] * 3
        out.data = vis.tobytes()
        self.pub_dbg.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = FallDetector()
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
