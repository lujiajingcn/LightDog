#!/usr/bin/env python3
"""RASPBOT V2 实车驱动桥接 —— 以「车上真有什么」为准，不再假设 STM32。

    /cmd_vel_muxed  ──┐                          ┌──> /odom（开环 / 商家里程计）
    /led/brightness ──┼─> lightdog_raspbot_driver ┼──> /battery_state（无遥测时发 NaN）
    /safety/estop   ──┘                          └──> /raspbot/diag（后端、闭锁、协议状态）
       /raspbot/button ─┘        (Pi 直驱)       └──> /safety/estop（按键 -> 软件闭锁）
                                   ↓
                     官方 Raspbot_Lib 的 I2C 寄存器（Ctrl_Muto）

## 为什么要有这个节点

`docs/hardware_arrival.md` 记录的实车是：**Pi 5 + 机器人驱动板（板载 MCU +
出厂固件），通过 I2C 用 Python 驱动库 `Raspbot_Lib` 控制，不是我们自写的 STM32
CRC16 串口协议；四个麦克纳姆轮 + TT 减速电机，官方 I2C API 全清单里没有任何
编码器 / 里程计 / IMU / 电压读取函数；只有一个点动「按键」，没有自锁蘑菇头急停**。
所以 `lightdog_mcu_bridge.py` 依赖的 CRC16 帧、IWDG、编码器里程计、物理按钮闭锁
在这台车上全都没有对应的硬件。

本节点保留上层完全相同的 ROS 契约（速度进，odom / 电量 / 急停出），只把
「下位机」这一层换成实车能用的东西：

1. **全向运动学**：`/cmd_vel_muxed` 是 Twist，`twist_mux` 本来就把 `linear.y`
   透传下来（`lightdog_twist_mux.py:517`）。这里做 X 型麦轮逆解，四个轮速
   按最大值**等比缩放**（截断某一个轮子会悄悄改变整车运动方向）。
2. **后端可插拔**。官方教程已给出确切的 I2C 驱动 API（`Ctrl_Muto` 等），
   但电机编号 / 轮序映射在官方两处教程里互相矛盾，必须上机实测：
   - `vendor_i2c`（默认）：`from Raspbot_Lib import Raspbot`，麦轮逆解后逐个调
     `Ctrl_Muto(motor_id, pwm)`（-255..255 开环 PWM）。**`motor_id_map` 没实测
     填好时强制降级 `self_test`，一个字节都不发** —— 猜错轮序会让车乱跑。
   - `vendor_topic`：把 Twist 转发给 ROS 底盘节点（默认 `/cmd_vel`）。注意官方
     RASPBOT V2 出厂驱动是 Python/Jupyter API，**不是 ROS 底盘节点**，这个后端
     只有在你自己起了订阅者时才有用；无订阅者时 diag 会告警。
   - `motor_array`：发布 `Float32MultiArray` 四轮线速度。
   - `direct_serial`：按模板发串口 —— **与本车不符**（本车是 I2C），仅为通用方案
     保留；`protocol_confirmed=false` 时同样强制降级 `self_test`。
   - `self_test`：只跑逻辑不发包。
3. **里程计**：默认 `openloop`，按实际下发速度积分，并在 diag 里明说"开环、会漂"。
   这不是「还没确认有没有编码器」：官方 `Raspbot_Lib` 的完整 API 清单里只有
   `Ctrl_*` 写函数和 `read_data_array`（仅用于循迹 / 超声波 / 红外寄存器），
   **没有编码器、轮速、里程计、IMU 或电压的读取接口**。若日后外接了能发
   `/odom` 的节点，把 `odom_source` 改成 `vendor` 即可。
4. **急停**：车上没有自锁按钮，所以 `/raspbot/button` 的点动信号由**软件补闭锁**
   （松手不自动恢复），并明确标注「断电不保持、电气回路没有硬断 PWM」，
   这与原固件的物理闭锁不是等价安全等级，见 `docs/hardware_arrival.md` §C2/C3。
5. **没有 IWDG 了**：靠 10Hz 定时器持续下发 + `cmd_hold` 超时归零 + 退出前发零速
   来替代。这比固件看门狗弱，是真机的已知风险，不藏起来。

## 电池为什么发 NaN

`dock_manager` 会忽略 NaN 电量，但**把 0 当成"没电了"立刻触发低电回充**
（见 `lightdog_mcu_bridge.py:370`）。实车驱动板还没确认是否上报电压/电流，
所以在拿到遥测前发 NaN 表示"不知道"，绝不填 0。
"""

import importlib
import json
import math
import sys
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, Float32, Float32MultiArray, String

from lightdog_qos import SAFETY_QOS

#: 轮序固定：左前、右前、左后、右后。改接线必须同步改这里和模板里的 {m1..m4}。
WHEEL_NAMES = ('FL', 'FR', 'RL', 'RR')

BACKENDS = ('vendor_i2c', 'vendor_topic', 'motor_array', 'direct_serial',
             'self_test')
ODOM_SOURCES = ('openloop', 'vendor', 'none')


def clamp_speed(v, lo, hi):
    """NaN 一律按 0：NaN 的比较永远为假，漏过去就是「电机收到 NaN」。"""
    if v != v:
        return 0.0
    return lo if v < lo else (hi if v > hi else v)


def mecanum_wheel_speeds(vx, vy, wz, half_x, half_y):
    """X 型麦克纳姆轮逆解，返回四个**轮缘线速度 m/s**（FL, FR, RL, RR）。

    标准 45° 滚子模型：`k = half_x + half_y`（车中心到轮的纵向+横向距离）。
    half_x / half_y 是实车实测值；用仿真 4wd 的 `wheel_base` 代替会整体错。
    """
    k = float(half_x) + float(half_y)
    return (vx - vy - k * wz,
            vx + vy + k * wz,
            vx + vy - k * wz,
            vx - vy + k * wz)


def normalize_wheel_speeds(speeds, limit):
    """单轮超限时**等比缩放**四个轮子，而不是各截各的。

    只把 FL 截断、其余不动，等于偷偷改了 vx/vy/wz 的比例 —— 车会朝不是
    指令要求的方向走，而且上层完全看不出来。limit<=0 时原样返回。
    """
    if limit <= 0.0:
        return tuple(speeds)
    peak = max((abs(float(s)) for s in speeds), default=0.0)
    if peak <= limit or peak == 0.0:
        return tuple(speeds)
    scale = limit / peak
    return tuple(float(s) * scale for s in speeds)


def integrate_openloop(x, y, yaw, vx, vy, wz, dt):
    """按车体速度积分出 (x, y, yaw)。dt 由调用方钳过，避免卡死后大跳。

    用中点航向 `yaw + 0.5*wz*dt` 把车体系速度旋到世界系，比直接用末端
    yaw 在一拍转弯时少一半误差。这是开环，没有编码器就一定会漂。
    """
    if dt <= 0.0:
        return x, y, yaw
    mid = yaw + 0.5 * wz * dt
    c, s = math.cos(mid), math.sin(mid)
    return (x + (vx * c - vy * s) * dt,
            y + (vx * s + vy * c) * dt,
            yaw + wz * dt)


def cmd_to_send(vx, vy, wz, estop, now, wall, hold):
    """该发什么速度。急停或超过 cmd_hold 没新指令都归零 —— 见文件头第 5 条。

    这是从 `lightdog_mcu_bridge._cmd_to_send` 搬过来的同一条安全规则，
    只是多了横向分量。停发速度 ≠ 停车，这条不能省。
    """
    if estop or (now - wall) > hold:
        return 0.0, 0.0, 0.0
    return vx, vy, wz


def parse_motor_id_map(raw):
    """解析 '0,1,2,3' 形式的电机编号映射，按 WHEEL_NAMES 顺序（FL,FR,RL,RR）。

    官方两处教程冲突：`RASPBOT-V2/12197` 的测试代码注释用 0-3（L1,L2,R1,R2），
    `Pi5-Board/12077` 却写 1-4（M1..M4），且都没给「编号 -> 轮子位置」的图。
    所以这里**不设默认值**：空串 / 非四个不重复整数 / 越界一律返回 None，
    调用方必须据此降级 self_test，绝不猜一个能用的值出来。
    """
    if raw is None:
        return None
    text = str(raw).strip()
    if not text:
        return None
    parts = [x.strip() for x in text.split(',')]
    if len(parts) != 4:
        return None
    ids = []
    for x in parts:
        try:
            v = int(x)
        except (TypeError, ValueError):
            return None
        if v < 0 or v > 8:
            return None
        ids.append(v)
    if len(set(ids)) != 4:
        return None
    return tuple(ids)


def wheel_speeds_to_pwm(speeds, max_wheel, pwm_scale):
    """四轮轮缘线速度 (m/s) -> `Ctrl_Muto` 的开环 PWM 整数 (-255..255)。

    `Ctrl_Muto` 只吃 PWM，没有速度闭环（官方 API 无编码器读数）。NaN 按 0；
    max_wheel / pwm_scale 非正时全返回 0，宁可不动也不发越界值。
    """
    if max_wheel <= 0.0 or pwm_scale <= 0.0:
        return (0, 0, 0, 0)
    lim = int(pwm_scale)
    out = []
    for s in speeds:
        v = 0.0 if s != s else max(-max_wheel, min(max_wheel, float(s)))
        pwm = int(round(v / max_wheel * pwm_scale))
        out.append(max(-lim, min(lim, pwm)))
    return tuple(out)


def _quat_from_yaw(yaw):
    half = 0.5 * float(yaw)
    return math.sin(half), math.cos(half)


class RaspbotDriver(Node):

    def __init__(self):
        super().__init__('lightdog_raspbot_driver')

        # ---- 后端（不猜商家协议，全部显式可配）----
        self.declare_parameter('driver_backend', 'vendor_topic')
        self.declare_parameter('vendor_cmd_topic', '/cmd_vel')
        self.declare_parameter('motor_topic', '/raspbot/wheel_speeds')
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('motor_frame_template',
                               'M {m1:.3f} {m2:.3f} {m3:.3f} {m4:.3f}\n')
        self.declare_parameter('protocol_confirmed', False)
        self.declare_parameter('reconnect_sec', 2.0)

        # ---- 官方 I2C 驱动（Raspbot_Lib，出厂镜像自带 ~/py_install）----
        # 电机编号 / 轮序官方没给一致的答案，必须上机四轮单动实测后填。
        self.declare_parameter('i2c_module', 'Raspbot_Lib')
        self.declare_parameter('i2c_class', 'Raspbot')
        self.declare_parameter('motor_id_map', '')
        self.declare_parameter('motor_pwm_scale', 255.0)

        # ---- 麦轮几何（实车待实测，值是按尺寸图的初值）----
        # 轴距/轮距取尺寸图 117.20 / 165.83 mm 的初值，代码里再各取一半。
        # 这四个数都是**估算**，上真机必须卡尺实测，见 docs/hardware_arrival.md §五。
        self.declare_parameter('wheel_radius', 0.0325)
        self.declare_parameter('wheel_base_x', 0.1172)
        self.declare_parameter('wheel_base_y', 0.1658)
        self.declare_parameter('max_wheel_mps', 0.35)
        self.declare_parameter('max_linear', 0.25)
        self.declare_parameter('max_lateral', 0.25)
        self.declare_parameter('max_angular', 0.70)

        # ---- 话题契约（与 mcu_bridge 保持一致，上层节点零改动）----
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_muxed')
        self.declare_parameter('led_topic', '/led/brightness')
        self.declare_parameter('vendor_led_topic', '/raspbot/led')
        self.declare_parameter('led_backend', 'none')
        self.declare_parameter('estop_topic', '/safety/estop')
        self.declare_parameter('button_topic', '/raspbot/button')
        self.declare_parameter('button_latching', True)
        self.declare_parameter('estop_auto_release', False)
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('vendor_odom_topic', '/odom_raw')
        self.declare_parameter('odom_source', 'openloop')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('battery_topic', '/battery_state')
        self.declare_parameter('battery_source', 'none')
        self.declare_parameter('vendor_battery_topic', '/raspbot/battery_ratio')
        self.declare_parameter('fault_topic', '/mcu/fault')
        self.declare_parameter('diag_topic', '/raspbot/diag')

        # ---- 时序 ----
        self.declare_parameter('heartbeat_hz', 10.0)
        self.declare_parameter('cmd_hold', 0.30)
        self.declare_parameter('battery_period', 5.0)
        self.declare_parameter('diag_period', 2.0)

        p = self.get_parameter
        self.backend = str(p('driver_backend').value)
        if self.backend not in BACKENDS:
            self.get_logger().error(f'未知 driver_backend={self.backend}，降级 self_test')
            self.backend = 'self_test'
        self.protocol_confirmed = bool(p('protocol_confirmed').value)
        self.protocol_blocked = False
        self.block_reason = ''
        self.i2c_module = str(p('i2c_module').value)
        self.i2c_class = str(p('i2c_class').value)
        self.motor_id_map = parse_motor_id_map(p('motor_id_map').value)
        self.motor_pwm_scale = float(p('motor_pwm_scale').value)
        if self.backend == 'direct_serial' and not self.protocol_confirmed:
            # 猜错串口协议的后果是车乱跑，宁可不动；本车其实是 I2C，见文件头。
            self.get_logger().error(
                'direct_serial 的商家协议还没确认（protocol_confirmed=false）——'
                '强制降级 self_test，一字节都不发；本车正确后端是 vendor_i2c')
            self.backend = 'self_test'
            self.protocol_blocked = True
            self.block_reason = '串口协议未确认（且本车是 I2C）'
        if self.backend == 'vendor_i2c' and self.motor_id_map is None:
            self.get_logger().error(
                'vendor_i2c 需要 motor_id_map：四个实测电机编号（FL,FR,RL,RR，逗号分隔）。'
                '官方两处教程编号不一致（0-3 / 1-4）、轮序也没给，猜错会让车乱跑 ——'
                '强制降级 self_test，一个字节都不发')
            self.backend = 'self_test'
            self.protocol_blocked = True
            self.block_reason = 'motor_id_map 未实测'

        self.wheel_radius = float(p('wheel_radius').value)
        self.half_x = 0.5 * float(p('wheel_base_x').value)
        self.half_y = 0.5 * float(p('wheel_base_y').value)
        self.max_wheel = float(p('max_wheel_mps').value)
        self.max_linear = float(p('max_linear').value)
        self.max_lateral = float(p('max_lateral').value)
        self.max_angular = float(p('max_angular').value)
        self.cmd_hold = float(p('cmd_hold').value)
        self.odom_frame = str(p('odom_frame_id').value)
        self.base_frame = str(p('base_frame_id').value)
        self.estop_auto_release = bool(p('estop_auto_release').value)
        self.button_latching = bool(p('button_latching').value)
        self.motor_template = str(p('motor_frame_template').value)
        self.battery_period = float(p('battery_period').value)
        self.diag_period = float(p('diag_period').value)
        self.led_backend = str(p('led_backend').value)

        self.odom_source = str(p('odom_source').value)
        if self.odom_source not in ODOM_SOURCES:
            self.get_logger().error(f'未知 odom_source={self.odom_source}，用 openloop')
            self.odom_source = 'openloop'
        self.odom_topic = str(p('odom_topic').value)
        self.vendor_odom_topic = str(p('vendor_odom_topic').value)
        if self.odom_source == 'vendor' and self.vendor_odom_topic == self.odom_topic:
            # 订阅自己的输出 = 正反馈，位姿会一直累加。
            self.get_logger().error('vendor_odom_topic 不能等于 odom_topic，改用 openloop')
            self.odom_source = 'openloop'
        self.battery_source = str(p('battery_source').value)

        # ---- 运行状态 ----
        self.vx = self.vy = self.wz = 0.0
        self.cmd_wall = 0.0
        self.estop = False
        self.button_pressed = False
        self.estop_published = False
        self.x = self.y = self.yaw = 0.0
        self.last_tick = time.monotonic()
        self.last_battery = 0.0
        self.last_diag = 0.0
        self.led = None
        self.led_sent = None
        self._warned_led = False
        self._warned_battery = False
        self._warned_template = False
        self.frames_sent = 0
        self.last_cmd = (0.0, 0.0, 0.0)
        self.bot = None                 # Raspbot_Lib.Raspbot 实例，仅 vendor_i2c 用

        # ---- ROS 接口 ----
        self.create_subscription(Twist, str(p('cmd_vel_topic').value),
                                 self._on_cmd, 10)
        self.create_subscription(Float32, str(p('led_topic').value),
                                 self._on_led, 10)
        self.create_subscription(Bool, str(p('estop_topic').value),
                                 self._on_estop, SAFETY_QOS)
        self.create_subscription(Bool, str(p('button_topic').value),
                                 self._on_button, SAFETY_QOS)
        if self.battery_source == 'vendor':
            self.create_subscription(Float32, str(p('vendor_battery_topic').value),
                                     self._on_vendor_battery, 10)
        if self.odom_source == 'vendor':
            self.create_subscription(Odometry, self.vendor_odom_topic,
                                     self._on_vendor_odom, 10)

        self.pub_odom = self.create_publisher(Odometry, self.odom_topic, 10)
        self.pub_batt = self.create_publisher(BatteryState,
                                              str(p('battery_topic').value), 10)
        self.pub_estop = self.create_publisher(Bool, str(p('estop_topic').value),
                                               SAFETY_QOS)
        self.pub_fault = self.create_publisher(String, str(p('fault_topic').value), 10)
        self.pub_diag = self.create_publisher(String, str(p('diag_topic').value), 10)
        self.pub_vendor = None
        self.pub_motors = None
        self.pub_led = None
        self.link = None
        self._init_backend(p)

        hz = max(1.0, float(p('heartbeat_hz').value))
        self.create_timer(1.0 / hz, self._tick)
        self.get_logger().info(
            f'RASPBOT V2 驱动就绪：backend={self.backend} odom={self.odom_source} '
            f'battery={self.battery_source} button_latching={self.button_latching} '
            f'hz={hz:.0f} cmd_hold={self.cmd_hold:.2f}s')
        if self.odom_source == 'openloop':
            self.get_logger().warn(
                '里程计是**开环积分**（官方 I2C API 无编码器/里程计读取接口），'
                '位姿会漂；'
                '只适合短距离跟随，不要拿它做全局导航')
        if self.button_latching:
            self.get_logger().warn(
                '急停是**软件闭锁**：点动按键松手不自动恢复，但电气回路没有硬断 PWM，'
                '安全等级低于自锁蘑菇头 + 固件断使能')

    def _init_backend(self, p):
        if self.backend == 'vendor_i2c':
            try:
                mod = importlib.import_module(self.i2c_module)
                self.bot = getattr(mod, self.i2c_class)()
                self.get_logger().info(
                    f'已加载官方 I2C 驱动 {self.i2c_module}.{self.i2c_class}；'
                    f'电机编号(FL,FR,RL,RR)={self.motor_id_map}')
            except Exception as e:
                self.bot = None
                self.backend = 'self_test'
                self.protocol_blocked = True
                self.block_reason = f'I2C 驱动库不可用：{e}'
                self.get_logger().error(
                    f'{self.i2c_module} 加载失败（{e}）—— 本车出厂镜像自带 '
                    '~/py_install，请确认已启用 I2C 且用户在 i2c 组；'
                    '强制降级 self_test')
        elif self.backend == 'vendor_topic':
            self.pub_vendor = self.create_publisher(
                Twist, str(p('vendor_cmd_topic').value), 10)
            self.get_logger().info(
                f'转发 Twist 到商家话题 {p("vendor_cmd_topic").value}'
                '（按商家驱动实际订阅名改 vendor_cmd_topic）')
        elif self.backend == 'motor_array':
            self.pub_motors = self.create_publisher(
                Float32MultiArray, str(p('motor_topic').value), 10)
        elif self.backend == 'direct_serial':
            # 延迟 import：lightdog_mcu_bridge 顶层 import termios，
            # Windows 宿主机做离线自检时不需要它。
            from lightdog_mcu_bridge import SerialLink
            self.link = SerialLink(str(p('port').value), int(p('baud').value),
                                   log=self.get_logger().error)
        if self.led_backend == 'vendor_topic':
            self.pub_led = self.create_publisher(
                Float32, str(p('vendor_led_topic').value), 10)

    # ---------- 回调 ----------

    def _on_cmd(self, msg: Twist):
        if self.estop:
            # 急停期间收到的速度一律丢掉，防止解急停瞬间把旧指令放出去。
            self.vx = self.vy = self.wz = 0.0
            return
        self.vx = clamp_speed(msg.linear.x, -self.max_linear, self.max_linear)
        self.vy = clamp_speed(msg.linear.y, -self.max_lateral, self.max_lateral)
        self.wz = clamp_speed(msg.angular.z, -self.max_angular, self.max_angular)
        self.cmd_wall = time.monotonic()

    def _on_led(self, msg: Float32):
        v = float(msg.data)
        if v != v:
            self.get_logger().error('LED 亮度收到 NaN，按 0 处理')
            v = 0.0
        self.led = clamp_speed(v, 0.0, 1.0)

    def _on_estop(self, msg: Bool):
        want = bool(msg.data)
        if want == self.estop:
            return
        self.estop = want
        if want:
            self.vx = self.vy = self.wz = 0.0
            self.cmd_wall = 0.0
        # 立刻停车，不等下一拍定时器 —— 急停不能有 100ms 的窗口。
        self._drv_send(0.0, 0.0, 0.0)
        self.get_logger().warn(f'软件急停 {"置位" if want else "解除"}，已下发零速')

    def _on_button(self, msg: Bool):
        on = bool(msg.data)
        self.button_pressed = on
        if on and not self.estop_published:
            self.get_logger().error('车上的点动按键按下 —— 软件闭锁置位')
            self.pub_estop.publish(Bool(data=True))
            self.estop_published = True
        elif not on and self.estop_published:
            if self.button_latching and not self.estop_auto_release:
                self.get_logger().warn(
                    '按键已松开，但**软件闭锁保持**（需显式 lightdog-estop clear）')
            elif self.estop_auto_release:
                self.pub_estop.publish(Bool(data=False))
                self.estop_published = False

    def _on_vendor_battery(self, msg: Float32):
        v = clamp_speed(float(msg.data), 0.0, 1.0)
        self._publish_battery(v, float('nan'), float('nan'))

    def _on_vendor_odom(self, msg: Odometry):
        if not msg.header.frame_id:
            msg.header.frame_id = self.odom_frame
        self.pub_odom.publish(msg)

    # ---------- 周期 ----------

    def _tick(self):
        now = time.monotonic()
        dt = min(0.5, max(0.0, now - self.last_tick))
        vx, vy, wz = cmd_to_send(self.vx, self.vy, self.wz, self.estop,
                                 now, self.cmd_wall, self.cmd_hold)
        if (vx, vy, wz) != (self.vx, self.vy, self.wz):
            # 超时归零后把缓存也清掉，避免下一拍又拿旧值出来比。
            self.vx, self.vy, self.wz = vx, vy, wz
        self._drv_send(vx, vy, wz)

        if self.odom_source == 'openloop':
            self.x, self.y, self.yaw = integrate_openloop(
                self.x, self.y, self.yaw, vx, vy, wz, dt)
            self._publish_odom(vx, vy, wz)

        if self.led is not None and self.led != self.led_sent:
            if self._send_led(self.led):
                self.led_sent = self.led

        if self.battery_source != 'vendor' and now - self.last_battery >= self.battery_period:
            self._publish_battery(float('nan'), float('nan'), float('nan'))
            self.last_battery = now
            if not self._warned_battery:
                self._warned_battery = True
                self.get_logger().warn(
                    '驱动板没确认电压/电流遥测，电量发 NaN（不知道）——'
                    '绝不能填 0，否则 dock_manager 会立刻判低电回充')

        if now - self.last_diag >= self.diag_period:
            self._publish_diag(now, vx, vy, wz)
            self.last_diag = now
        self.last_tick = now

    def _drv_send(self, vx, vy, wz):
        self.last_cmd = (vx, vy, wz)
        if self.backend == 'self_test':
            return
        if self.backend == 'vendor_topic':
            m = Twist()
            m.linear.x, m.linear.y, m.angular.z = vx, vy, wz
            self.pub_vendor.publish(m)
            self.frames_sent += 1
            return
        speeds = normalize_wheel_speeds(
            mecanum_wheel_speeds(vx, vy, wz, self.half_x, self.half_y),
            self.max_wheel)
        if self.backend == 'motor_array':
            m = Float32MultiArray()
            m.data = [float(s) for s in speeds]
            self.pub_motors.publish(m)
            self.frames_sent += 1
            return
        if self.backend == 'vendor_i2c':
            self._send_i2c(speeds)
            return
        if self.backend == 'direct_serial':
            self._send_serial(speeds)

    def _send_serial(self, speeds):
        if self.link is None:
            return
        if not self.link.open and not self._ensure_link():
            return
        try:
            payload = self.motor_template.format(
                m1=speeds[0], m2=speeds[1], m3=speeds[2], m4=speeds[3],
                r=self.wheel_radius).encode('ascii')
        except (KeyError, IndexError, ValueError, UnicodeError) as e:
            if not self._warned_template:
                self._warned_template = True
                self.get_logger().error(f'电机帧模板无效，停发：{e}')
            return
        if self.link.write(payload) is True:
            self.frames_sent += 1

    def _send_i2c(self, speeds):
        """把四轮速度写进官方驱动的 PWM 寄存器。失败时必须补一次全零。"""
        if self.bot is None or self.motor_id_map is None:
            return
        pwms = wheel_speeds_to_pwm(speeds, self.max_wheel, self.motor_pwm_scale)
        try:
            for mid, pwm in zip(self.motor_id_map, pwms):
                self.bot.Ctrl_Muto(mid, pwm)
            self.frames_sent += 1
        except Exception as e:
            self.get_logger().error(
                f'I2C 电机下发失败，立刻补发全零：{e}', throttle_duration_sec=5.0)
            try:
                for mid in self.motor_id_map:
                    self.bot.Ctrl_Muto(mid, 0)
            except Exception:
                pass

    def _ensure_link(self):
        if self.link is None or self.link.open:
            return self.link is not None and self.link.open
        if self.link.open_port():
            self.get_logger().info(f'商家驱动板串口已打开：{self.link.port}')
            return True
        self.get_logger().error(
            f'串口 {self.link.port} 打不开 —— 电机不会动', throttle_duration_sec=10.0)
        return False

    def _send_led(self, v):
        if self.led_backend == 'i2c':
            # 官方 API：Ctrl_WQ2812_brightness_ALL(R, G, B)，0-255。
            if self.bot is None:
                return False
            v8 = int(round(clamp_speed(v, 0.0, 1.0) * 255.0))
            try:
                self.bot.Ctrl_WQ2812_brightness_ALL(v8, v8, v8)
                return True
            except Exception as e:
                self.get_logger().error(f'灯条 I2C 下发失败：{e}',
                                        throttle_duration_sec=5.0)
                return False
        if self.led_backend == 'none':
            if not self._warned_led:
                self._warned_led = True
                self.get_logger().warn(
                    'RGB 灯条协议未确认，/led/brightness 只记录不下发'
                    '（拿到商家协议后把 led_backend 改成 vendor_topic）')
            return True
        if self.pub_led is not None:
            self.pub_led.publish(Float32(data=float(v)))
            return True
        return False

    def _publish_odom(self, vx, vy, wz):
        m = Odometry()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.odom_frame
        m.child_frame_id = self.base_frame
        m.pose.pose.position.x = self.x
        m.pose.pose.position.y = self.y
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = \
            _quat_from_yaw(self.yaw)
        m.twist.twist.linear.x = vx
        m.twist.twist.linear.y = vy
        m.twist.twist.angular.z = wz
        self.pub_odom.publish(m)

    def _publish_battery(self, pct, volt, amp):
        m = BatteryState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.present = True
        m.percentage = pct            # NaN 表示「不知道」，dock_manager 会忽略
        m.voltage = volt
        m.current = amp
        self.pub_batt.publish(m)

    def _publish_diag(self, now, vx, vy, wz):
        subs = None
        if self.backend == 'vendor_topic' and self.pub_vendor is not None:
            subs = self.pub_vendor.get_subscription_count()
            if subs == 0:
                self.get_logger().warn(
                    'vendor_topic 目标话题没有任何订阅者：官方 RASPBOT V2 出厂驱动 '
                    '是 Python I2C API，不是 ROS 底盘节点，车不会动。'
                    '请改用 driver_backend=vendor_i2c 并填好 motor_id_map',
                    throttle_duration_sec=10.0)
        data = {
            'at_wall': round(now, 3),
            'backend': self.backend,
            'protocol_confirmed': self.protocol_confirmed,
            'protocol_blocked': self.protocol_blocked,
            'block_reason': self.block_reason,
            'motor_id_map': list(self.motor_id_map) if self.motor_id_map else None,
            'vendor_subscribers': subs,
            'odom_source': self.odom_source,
            'battery_source': self.battery_source,
            'button_latching': self.button_latching,
            'estop': self.estop,
            'cmd': [round(vx, 4), round(vy, 4), round(wz, 4)],
            'wheel_names': list(WHEEL_NAMES),
            'wheels': [round(v, 4) for v in normalize_wheel_speeds(
                mecanum_wheel_speeds(vx, vy, wz, self.half_x, self.half_y),
                self.max_wheel)],
            'pose': [round(self.x, 4), round(self.y, 4), round(self.yaw, 4)],
            'frames_sent': self.frames_sent,
        }
        self.pub_diag.publish(String(data=json.dumps(data, ensure_ascii=False)))

    def destroy_node(self):
        # 没有固件看门狗，退出前必须自己发零速；但不置急停闭锁（那是给人的状态）。
        try:
            self._drv_send(0.0, 0.0, 0.0)
        except Exception:
            pass
        if self.link is not None:
            self.link.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RaspbotDriver()
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
