#!/usr/bin/env python3
"""下位机桥接：ROS 话题 ⟷ STM32 串口帧。

    /cmd_vel_muxed  ──┐                      ┌──> /battery_state
    /led/brightness ──┼─> lightdog_mcu_bridge ┼──> /odom
    /safety/estop   ──┘        (UART)        └──> /safety/estop（物理按钮）
                                   ↕
                              STM32 固件

协议定稿见 `lightdog_mcu_proto.py` / `docs/firmware_protocol.md` §六；
固件侧的参考实现（纯 C99，可在 PC 上 `make test`）在 `firmware/`。

## 这个节点最容易出的三个错，以及它是怎么处理的

1. **上位机停发速度 ≠ 停车。** `lightdog_follower` 让出控制权时会「先发零速、
   然后彻底停发」。如果桥接节点把「最后收到的速度」一直照着发，车会带着最后
   那条指令一直走 —— 这是最容易造成事故的一类 bug。所以有 `cmd_hold`（0.30s）：
   超过这个时间没新指令，就**主动发零速**。

2. **急停之后不能再发速度。** 软件急停一置位，本节点立刻把「待发速度」清成零，
   并把 ESTOP 帧发给固件；固件自己也会忽略后续速度。两边都做，是因为
   「只有一边做」时，另一边的一个 bug 就能让车动起来。

3. **串口断了不能静默。** 打不开串口时节点不崩（其余功能照常），但每秒最多
   报一次 error，并且**停发心跳** —— 让固件的 300ms 看门狗去断 PWM。
   反过来说：桥接节点死了，车也必须停，这是设计要求不是缺陷。

## 为什么不用 pyserial

`termios` 是标准库，树莓派上不需要额外装包，也不会因为 pyserial 版本差异
在「读超时是返回 b'' 还是抛异常」上踩坑。串口配置只要 8N1 无流控这一种，
用不到 pyserial 的抽象。
"""

import json
import math
import os
import sys
import termios
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, Float32, String

import lightdog_mcu_proto as proto
from lightdog_qos import SAFETY_QOS

_BAUDS = {}
for _name, _const in (('9600', 'B9600'), ('19200', 'B19200'),
                      ('38400', 'B38400'), ('57600', 'B57600'),
                      ('115200', 'B115200'), ('230400', 'B230400'),
                      ('460800', 'B460800'), ('921600', 'B921600')):
    if hasattr(termios, _const):
        _BAUDS[int(_name)] = getattr(termios, _const)


class SerialLink:
    """最小可用的 8N1 串口。非阻塞读，出错返回 None 让上层去重连。"""

    def __init__(self, port, baud=115200, log=None):
        self.port = port
        self.baud = int(baud)
        self.log = log or (lambda *a, **k: None)
        self.fd = None
        self._warned_full = False

    @property
    def open(self):
        return self.fd is not None

    def open_port(self):
        if self.fd is not None:
            return True
        speed = _BAUDS.get(self.baud)
        if speed is None:
            self.log(f'不支持的波特率 {self.baud}')
            return False
        try:
            fd = os.open(self.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        except OSError:
            # 不在这里打日志：调用方（_ensure_link）会带上下文报一次，
            # 而这里每 2 秒重试一次，重一条就是每 2 秒一条 error 刷屏。
            return False
        try:
            iflag, oflag, cflag, lflag, _, _, cc = termios.tcgetattr(fd)
            iflag &= ~(termios.IGNBRK | termios.BRKINT | termios.PARMRK
                       | termios.ISTRIP | termios.INLCR | termios.IGNCR
                       | termios.ICRNL | termios.IXON | termios.IXOFF)
            oflag &= ~termios.OPOST
            lflag &= ~(termios.ECHO | termios.ECHONL | termios.ICANON
                       | termios.ISIG | termios.IEXTEN)
            cflag &= ~(termios.CSIZE | termios.PARENB | termios.CSTOPB)
            cflag |= termios.CS8 | termios.CREAD | termios.CLOCAL
            cc[termios.VMIN] = 0
            cc[termios.VTIME] = 1          # 100ms 读超时
            termios.tcsetattr(fd, termios.TCSANOW,
                              [iflag, oflag, cflag, lflag, speed, speed, cc])
            termios.tcflush(fd, termios.TCIOFLUSH)
        except termios.error as e:
            # 非法波特率/不是 tty 都会走到这里。**必须关掉 fd**，
            # 否则每 2 秒重试一次就会泄漏一个文件描述符，几小时后 EBADF 满屏。
            self.log(f'配置串口失败（{self.port} @ {self.baud}）：{e}')
            os.close(fd)
            return False
        self.fd = fd
        return True

    def close(self):
        if self.fd is not None:
            try:
                os.close(self.fd)
            except OSError:
                pass
        self.fd = None

    def write(self, data):
        """返回 True=整帧写完 / False=链路坏 / None=暂时写不出去。

        fd 是 O_NONBLOCK：os.write 可能只写一半，也可能抛 EAGAIN。
        不检查返回值就把「写了 3 个字节」当成功，固件会收到残缺帧；
        把 EAGAIN 当链路故障去重连，则每 2 秒白白清一次缓冲。
        """
        if self.fd is None:
            return False
        view = memoryview(data)
        while view:
            try:
                n = os.write(self.fd, view)
            except BlockingIOError:
                # 输出缓冲满，串口还活着。帧可能已经写出去一截，绝不能整帧
                # 重发（那会让固件看到两个 SYNC）；丢掉剩余字节，让解析器在
                # 下一帧 SYNC 处重新同步，同时只报一次日志。
                if not self._warned_full:
                    self._warned_full = True
                    self.log('串口输出缓冲满：本拍丢弃剩余字节，不重连')
                return None
            except OSError as e:
                self.log(f'串口写失败：{e}')
                self.close()
                return False
            if n <= 0:
                return None
            view = view[n:]
        self._warned_full = False
        return True

    def read(self, max_bytes=256):
        """返回 bytes；链路坏了返回 None（和「这一拍没有数据」区分开）。"""
        if self.fd is None:
            return None
        try:
            return os.read(self.fd, max_bytes)
        except BlockingIOError:
            return b''
        except OSError as e:
            self.log(f'串口读失败：{e}')
            self.close()
            return None


class McuBridge(Node):

    def __init__(self):
        super().__init__('lightdog_mcu_bridge')

        self.declare_parameter('port', '/dev/ttyAMA0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('heartbeat_hz', 10.0)
        self.declare_parameter('cmd_hold', 0.30)      # 多久没新速度就发零速
        self.declare_parameter('max_linear', 0.25)    # 桥接层再钳一次
        self.declare_parameter('max_angular', 0.70)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_muxed')
        self.declare_parameter('led_topic', '/led/brightness')
        self.declare_parameter('estop_topic', '/safety/estop')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('battery_topic', '/battery_state')
        self.declare_parameter('fault_topic', '/mcu/fault')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('reconnect_sec', 2.0)
        # 物理按钮上报 true 之后，是否允许 MCU 自己报 false（一般不要：
        # 闭锁只能由人显式解除，见 firmware_protocol.md §3.2）
        self.declare_parameter('estop_auto_release', False)

        p = self.get_parameter
        self.port = str(p('port').value)
        self.baud = int(p('baud').value)
        self.cmd_hold = float(p('cmd_hold').value)
        self.max_linear = float(p('max_linear').value)
        self.max_angular = float(p('max_angular').value)
        self.odom_frame = str(p('odom_frame_id').value)
        self.base_frame = str(p('base_frame_id').value)
        self.reconnect_sec = float(p('reconnect_sec').value)
        self.estop_auto_release = bool(p('estop_auto_release').value)

        self.link = SerialLink(self.port, self.baud, log=self.get_logger().error)
        self.parser = proto.Parser()

        self.estop = False            # 软件急停（来自 /safety/estop）
        self.mcu_estop = False        # 固件上报的物理按钮状态
        self.estop_published = False  # 已经因物理按钮发过 true？
        self.cmd_lin = 0.0
        self.cmd_ang = 0.0
        self.cmd_wall = 0.0
        self.led = None
        self.led_sent = None
        self.last_open_try = -1e9
        self.fault_mask = None
        self.link_ok = False
        self.unknown_types = 0
        self.frames = 0

        self.create_subscription(Twist, str(p('cmd_vel_topic').value),
                                 self._on_cmd, 10)
        self.create_subscription(Float32, str(p('led_topic').value),
                                 self._on_led, 10)
        self.create_subscription(Bool, str(p('estop_topic').value),
                                 self._on_estop, SAFETY_QOS)

        self.pub_odom = self.create_publisher(Odometry, str(p('odom_topic').value), 10)
        self.pub_batt = self.create_publisher(BatteryState,
                                              str(p('battery_topic').value), 10)
        self.pub_estop = self.create_publisher(Bool, str(p('estop_topic').value),
                                               SAFETY_QOS)
        self.pub_fault = self.create_publisher(String, str(p('fault_topic').value), 10)

        hz = max(1.0, float(p('heartbeat_hz').value))
        self.create_timer(1.0 / hz, self._tick)
        self.get_logger().info(
            f'MCU 桥接就绪：{self.port} @ {self.baud}，心跳 {hz:.0f}Hz，'
            f'cmd_hold={self.cmd_hold:.2f}s')

    # ---------- 回调 ----------

    def _on_cmd(self, msg: Twist):
        if self.estop:
            # 急停期间收到的速度一律丢掉。只靠 twist_mux 挡是不够的：
            # 「解急停的瞬间把几秒前的旧指令发出去」是最典型的惊车路径。
            self.cmd_lin = 0.0
            self.cmd_ang = 0.0
            return
        lin = self._clamp(msg.linear.x, -self.max_linear, self.max_linear)
        ang = self._clamp(msg.angular.z, -self.max_angular, self.max_angular)
        self.cmd_lin = lin
        self.cmd_ang = ang
        self.cmd_wall = time.monotonic()

    def _on_led(self, msg: Float32):
        v = float(msg.data)
        if v != v:                    # NaN -> 0，并明确报一次
            self.get_logger().error('LED 亮度收到 NaN，按 0 处理')
            v = 0.0
        self.led = self._clamp(v, 0.0, 1.0)

    def _on_estop(self, msg: Bool):
        want = bool(msg.data)
        if want == self.estop:
            return
        self.estop = want
        if want:
            # 急停一置位，本节点手里的「待发速度」立刻作废
            self.cmd_lin = 0.0
            self.cmd_ang = 0.0
            self.cmd_wall = 0.0
        self._send(proto.encode_estop(want))
        self.get_logger().warn(f'急停 {"置位" if want else "解除"}，已下发固件')

    # ---------- 周期 ----------

    def _tick(self):
        self._ensure_link()
        if not self.link.open:
            return
        data = self.link.read()
        if data is None:
            self._link_lost('串口读失败')
            return
        if data:
            for frame in self.parser.feed(data):
                self._handle_frame(frame)

        now = time.monotonic()
        # 心跳和速度分开发：心跳必须能在「没有速度指令」时单独发出去，
        # 否则固件无法区分「待命」和「上位机死了」（firmware_protocol.md §1.2）
        self._send(proto.encode_heartbeat())
        lin, ang = self._cmd_to_send(now)
        self._send(proto.encode_cmd_vel(lin, ang))
        if self.led is not None and self.led != self.led_sent:
            if self._send(proto.encode_led(self.led)):
                self.led_sent = self.led

    def _cmd_to_send(self, now):
        """该发多快。超过 cmd_hold 没新指令就归零 —— 见文件头第 1 条。"""
        if self.estop:
            return 0.0, 0.0
        if now - self.cmd_wall > self.cmd_hold:
            self.cmd_lin = 0.0
            self.cmd_ang = 0.0
        return self.cmd_lin, self.cmd_ang

    def _ensure_link(self):
        if self.link.open:
            self.link_ok = True
            return
        now = time.monotonic()
        if now - self.last_open_try < self.reconnect_sec:
            return
        self.last_open_try = now
        if self.link.open_port():
            self.get_logger().info(f'串口已打开：{self.port} @ {self.baud}')
            self.parser = proto.Parser()
            self.link_ok = True
        else:
            self.get_logger().error(
                f'串口 {self.port} 打不开 —— 车不会动（固件看门狗也收不到心跳）。'
                f'检查接线、dtoverlay 与权限（dialout 组）',
                throttle_duration_sec=10.0)
            self.link_ok = False

    def _link_lost(self, why):
        if self.link_ok:
            self.get_logger().error(f'下位机链路断开（{why}）—— 固件将在 300ms 内断 PWM')
        self.link_ok = False
        self.link.close()

    def _send(self, frame):
        r = self.link.write(frame)
        if r is True:
            return True
        if r is None:
            # 缓冲满：这一拍没发出去，但链路没坏，不要触发重连。
            return False
        self._link_lost('串口写失败')
        return False

    # ---------- 上行帧 ----------

    def _handle_frame(self, frame):
        self.frames += 1
        t = frame.type
        if t == proto.T_BATTERY:
            self._publish_battery(frame.payload)
        elif t == proto.T_ODOM:
            self._publish_odom(frame.payload)
        elif t == proto.T_ESTOP_STATE:
            self._publish_estop_state(frame.payload)
        elif t == proto.T_FAULT:
            self._publish_fault(frame.payload)
        else:
            # 固件升级后新增了类型但上位机没更新：计数 + 报一次，
            # 不要静默丢弃（否则「固件升级没生效」永远查不出来）
            self.unknown_types += 1
            self.get_logger().warn(
                f'未知帧类型 0x{t:02x}（共 {self.unknown_types} 个）—— '
                f'固件比上位机新？', throttle_duration_sec=10.0)

    def _publish_battery(self, payload):
        vals = proto.decode_battery(payload)
        if vals is None:
            return
        pct, volt, amp = vals
        m = BatteryState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.present = True
        # NaN 原样传下去：dock_manager 会忽略 NaN，而不是当成「没电了」。
        # 反过来在这里填 0 会立刻触发低电回充。
        m.percentage = self._pct_to_msg(pct)
        m.voltage = volt
        m.current = amp
        self.pub_batt.publish(m)

    def _publish_odom(self, payload):
        vals = proto.decode_odom(payload)
        if vals is None:
            return
        x, y, yaw, v, w = vals
        z, qw = self._yaw_to_quat(yaw)
        m = Odometry()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.odom_frame
        m.child_frame_id = self.base_frame
        m.pose.pose.position.x = x
        m.pose.pose.position.y = y
        m.pose.pose.orientation.z = z
        m.pose.pose.orientation.w = qw
        m.twist.twist.linear.x = v
        m.twist.twist.angular.z = w
        self.pub_odom.publish(m)

    def _publish_estop_state(self, payload):
        on = proto.decode_estop_state(payload)
        if on is None or on == self.mcu_estop:
            return
        self.mcu_estop = on
        if on:
            self.get_logger().error('固件报告物理急停按钮按下 —— 车已断 PWM')
            # 用 SAFETY_QOS 上报：闭锁是状态不是事件，重启后的节点要能继承
            self.pub_estop.publish(Bool(data=True))
            self.estop_published = True
        elif self.estop_auto_release and self.estop_published:
            self.get_logger().warn('固件报告按钮已松开，自动解除闭锁')
            self.pub_estop.publish(Bool(data=False))
            self.estop_published = False
        else:
            self.get_logger().warn(
                '固件报告按钮已松开，但**闭锁保持**（要显式 lightdog-estop clear）')

    def _publish_fault(self, payload):
        mask = proto.decode_fault(payload)
        if mask is None or mask == self.fault_mask:
            return
        self.fault_mask = mask
        text = proto.fault_text(mask)
        m = String()
        m.data = json.dumps({'fault': mask, 'text': text,
                             'at_wall': round(time.time(), 3)},
                            ensure_ascii=False)
        self.pub_fault.publish(m)
        if mask:
            self.get_logger().error(f'固件故障：{text}')
        else:
            self.get_logger().info('固件故障已清除')

    # ---------- 工具 ----------

    @staticmethod
    def _clamp(v, lo, hi):
        # NaN 一律按 0 处理：NaN 进比较永远为假，会原样透出去变成「电机收到 NaN」。
        if v != v:
            return 0.0
        return lo if v < lo else (hi if v > hi else v)

    @staticmethod
    def _pct_to_msg(pct):
        """电量百分比：NaN 原样传（表示「不知道」），其余钳到 0..1。"""
        if pct != pct:
            return float('nan')
        return 0.0 if pct < 0.0 else (1.0 if pct > 1.0 else float(pct))

    @staticmethod
    def _yaw_to_quat(yaw):
        """偏航角 -> 四元数 (z, w)。只要平面运动，x/y 恒为 0。"""
        half = 0.5 * float(yaw)
        return math.sin(half), math.cos(half)

    def destroy_node(self):
        # 只关串口，**不发 ESTOP**：那是闭锁，会让下次上电必须人工解除。
        # 心跳一停，固件 300ms 后自己断 PWM —— 这才是「停机」的正确表达。
        self.link.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = McuBridge()
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
