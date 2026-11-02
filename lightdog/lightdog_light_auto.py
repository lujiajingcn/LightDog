#!/usr/bin/env python3
"""
LightDog 光照联动（P2 / 场景 1）

场景：夜里人下床 -> 机器人看到人 -> 主灯亮起照亮去卫生间的路 -> 人离开后自动熄灭。

## 为什么不能直接「有人亮、无人灭」

人走两步、一转身，检测会短暂丢帧。如果立刻灭灯，房间就是一片闪烁，比不开灯更糟。
所以这里有三个时间参数：

    fade_delay   确认人离开多久后才开始变暗（默认 5s，容忍短暂丢检）
    fade_time    变暗/变亮各用多久（默认 2s，线性过渡）
    hold         无人时是否保留一点余光（默认全灭，符合「不影响睡眠」）
    present_timeout  多久收不到 /person/present 就认为检测器已经死了（默认 3s）

时间一律用**墙钟**。这些量问的都是「上一次看到人是多久前」，与仿真跑多快无关；
用 ROS 时钟的话，真机上 use_sim_time 若被误开成 true 而没有 /clock，时钟停走，
「人走了就关灯」这条永远不会触发 —— 灯整夜亮着。

`present_timeout` 存在的理由同上一条的另一面：检测器崩掉后 /person/present 不再
有人发，最后一条 true 会永远留在本节点里。没有这个判据时现象是「人回床睡了，
灯一直亮到早上」。

亮度直接下发到 /led/brightness（P1 已实现的接口），本节点不碰材质层。

## 接口

    订阅 /person/present   std_msgs/Bool
    发布 /led/brightness   std_msgs/Float32  0.0 ~ 1.0

`enabled=false` 时本节点完全不发布，把 /led/brightness 让给手动控制（p1.sh）。
"""

import math
import sys
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, Float32, String


class LightAuto(Node):

    def __init__(self):
        super().__init__('lightdog_light_auto')

        self.declare_parameter('on_brightness', 0.85)   # 有人时的亮度
        self.declare_parameter('idle_brightness', 0.0)  # 无人时的余光
        self.declare_parameter('fade_delay', 5.0)       # 人走后多久开始变暗
        self.declare_parameter('fade_time', 2.0)        # 过渡时长
        self.declare_parameter('step_hz', 20.0)         # 渐变刷新率
        # 多久收不到 /person/present 就当作「检测器已经死了」（墙钟秒）。
        # 检测器正常时以 rate Hz 持续发（真机 10Hz、仿真 10Hz），3s 是 30 个周期。
        self.declare_parameter('present_timeout', 3.0)
        # 语音/手机端的手动覆盖：auto | on | off。留空/auto = 回到自动逻辑。
        # 这里是**覆盖**而不是另开一条亮度话题，是为了避免两个发布者同时往
        # /led/brightness 灌值，最后谁生效取决于时序。
        self.declare_parameter('mode_topic', '/light/mode')
        self.declare_parameter('enabled', True)

        p = self.get_parameter
        self.on_level = float(p('on_brightness').value)
        self.idle_level = float(p('idle_brightness').value)
        self.fade_delay = float(p('fade_delay').value)
        self.fade_time = max(0.1, float(p('fade_time').value))
        self.present_timeout = float(p('present_timeout').value)
        self.enabled = bool(p('enabled').value)
        hz = max(1.0, float(p('step_hz').value))

        self.present = False
        self.last_seen = None        # 最后一次 present=True 的墙钟时刻
        self.msg_wall = 0.0          # 最后一次收到 /person/present（无论真假）
        self.last_t = None
        self._warned_stall = False
        self.level = self.idle_level          # 当前实际下发值
        self.target = self.idle_level
        self.override = None                  # None/auto | on | off

        self.create_subscription(Bool, '/person/present', self._on_present, 10)
        self.create_subscription(
            String, str(p('mode_topic').value), self._on_mode, 10)
        self.pub = self.create_publisher(Float32, '/led/brightness', 10)

        self.create_timer(1.0 / hz, self._tick)

        self.get_logger().info(
            f'LightDog light-auto ready: on={self.on_level:.2f} '
            f'idle={self.idle_level:.2f} delay={self.fade_delay:.1f}s '
            f'fade={self.fade_time:.1f}s enabled={self.enabled}')

    def _on_present(self, msg: Bool):
        self.present = bool(msg.data)
        self.msg_wall = time.monotonic()
        if self.present:
            self.last_seen = self.msg_wall

    def _on_mode(self, msg: String):
        m = str(msg.data).strip().lower()
        if m in ('auto', ''):
            self.override = None
            self.get_logger().info('灯光：回到自动（跟随人体检测）')
        elif m in ('on', 'off'):
            self.override = m
            self.get_logger().info(f'灯光：手动 {m.upper()}')
        else:
            self.get_logger().warn(f'未知灯光模式 {m!r}（只认 auto / on / off）')

    def _tick(self):
        if not self.enabled:
            return

        now = time.monotonic()
        dt = 0.0 if self.last_t is None else (now - self.last_t)
        self.last_t = now
        # 时间跳变保护（定时器被拖长/系统休眠时别让亮度瞬间跳到底）
        dt = min(max(dt, 0.0), 0.5)

        # --- 检测器是否还活着 ---
        # 收不到消息时保留 present 的旧值就等于「灯永远亮着」：检测器一崩，
        # 最后那条 true 再也不会被推翻。过期后按「无人」处理，交给下面
        # 的 fade_delay 正常走完淡出。
        stale = (self.msg_wall <= 0.0
                 or (now - self.msg_wall) > self.present_timeout)
        if stale:
            # 只在「曾经收到过、后来断了」时报。开机还没收到过是正常启动顺序，
            # 这条 warn 会变成每次上电必现的噪音。
            if self.msg_wall > 0.0 and not self._warned_stall:
                self._warned_stall = True
                self.get_logger().warn(
                    f'{now - self.msg_wall:.1f}s 没收到 /person/present，'
                    f'判定检测器已退出，按无人处理并淡出'
                    f'（检查 lightdog_person_detector）')
            present = False
        else:
            if self._warned_stall:
                self._warned_stall = False
                self.get_logger().info('/person/present 恢复')
            present = self.present

        # --- 目标亮度决策 ---
        if self.override == 'on':
            self.target = self.on_level
        elif self.override == 'off':
            self.target = 0.0
        elif present:
            self.target = self.on_level
        elif self.last_seen is None:
            self.target = self.idle_level
        else:
            age = now - self.last_seen
            self.target = self.idle_level if age >= self.fade_delay else self.on_level

        # --- 线性过渡 ---
        # 按时间积分，而不是「每 tick 固定一步」。原来那种写法假定 timer 真能跑满
        # step_hz，但实测软件渲染下 20Hz 的 timer 只跑到 ~4.5Hz，于是 fade_time=2s
        # 的渐变实际要 8~9s。改成按 dt 积分后，渐变时长与 tick 频率解耦。
        diff = self.target - self.level
        max_delta = dt / self.fade_time
        if abs(diff) <= max_delta:
            self.level = self.target
        else:
            self.level += math.copysign(max_delta, diff)

        self.pub.publish(Float32(data=float(self.level)))


def main(args=None):
    rclpy.init(args=args)
    node = LightAuto()
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
