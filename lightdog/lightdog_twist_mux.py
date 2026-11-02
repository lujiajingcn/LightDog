#!/usr/bin/env python3
"""
LightDog 速度仲裁器（/cmd_vel 多源仲裁）

## 为什么需要这个节点

P2 跑跟随的时候发现一个硬问题：`/cmd_vel` 上有 4 个发布者在同时灌指令。

    collision_monitor   Nav2 的碰撞保护，把 cmd_vel_smoothed 中转成 cmd_vel
    docking_server      回充控制器，自己直接发 cmd_vel
    command_timeout     Nav2 的保险丝，/cmd_vel 一超时就发零速
    ros_gz_bridge       双向桥接，gz 侧的速度也会回灌到 ROS

导航空闲时 Nav2 那几个会持续发零速，把 follower 的 -0.15 m/s 淹掉 ——
实测机器人 6 秒只挪了 5cm。P2 当时的权宜之计是在 `p2.sh start` 里把
Nav2 的运动节点整个 kill 掉，代价是**丢了碰撞保护**。车装上真的电机之前
必须把这个补回来，否则一次近距离障碍就是一次撞车。

## 架构

                     ┌── /cmd_vel_smoothed  (Nav2,  prio 10) ──┐
                     │                                          │
   上游速度源 ────────┼── /cmd_vel_follow    (跟随,  prio 20) ──┼──> lightdog_twist_mux
                     │                                          │        │
                     └── /cmd_vel_dock      (回充,  prio 30) ──┘        │ /cmd_vel_muxed
                                                                         │
                                                          collision_monitor（碰撞保护）
                                                                         │ /cmd_vel
                                                               ros_gz_bridge → gz → 底盘

仲裁放在 `collision_monitor` **之前**，这样不管哪个源赢了，最终都要过一遍
碰撞保护 —— 这是与「直接抢 /cmd_vel」最大的区别。

## 仲裁规则

1. 每个源独立计时：最后一次收到该源消息的时间 + `*_timeout` 之内算「活跃」。
2. 活跃源里 `*_priority` 最大的那个独占输出；同优先级按 dock > follow > nav。
3. 一个活跃源都没有 → 发零速（默认），避免底层保持上一条指令。

## 两个窗口：源活着 ≠ 指令还能用

只用一个超时窗口是有害的：上游进程猝死（被 kill、段错误、USB 掉线）以后，
最后一条 Twist 还留在缓存里，在剩下的窗口内会被原样转发给底盘。
0.25 m/s × 1.0s = 25cm —— 足够把茶几上的杯子撞下来。

所以每个源有**两个**窗口：

- `*_timeout`（源窗口，0.6~1.0s）：「这个源还活着吗」。决定谁赢得仲裁。
- `*_hold`（指令窗口，0.3~0.5s）：「这条指令还新鲜吗」。超过就**输出零速**，
  状态里标成 `follow(stale)`。

`*_hold` 必须显著大于上游的正常发布周期（跟随 10Hz → 0.1s/条），
又要小于 `*_timeout`。启动时会做一次钳制：`hold = min(hold, timeout)`。

但**配置值不是最终生效值**：两个窗口都会按上游实测发布周期自适应放大
（`CmdVelSource._window`，窗口 = max(配置值, k × 实测周期)，k 分别为 4 和 2）。
这是被仿真打出来的教训：跟随节点按 ROS 时钟 10Hz 发，实时因子 0.21x 时墙钟上
只有 4.8Hz、间隔最大 0.32s，固定 0.30s 的 hold 被正常发包反复触发，
仲裁在 `follow` / `follow(stale)` 之间每 0.3s 抖一次，机器人几乎不给速度。
真机 RTF≈1 时 k×周期小于配置值，行为与固定窗口完全一致。

这仍然只是软件兜底。真正的刹车必须由 MCU 固件做：300ms 收不到速度指令
就断 PWM。参见 `docs/firmware_protocol.md`。

超时用**墙钟**（`time.monotonic()`）而不是 ROS 时钟：这个窗口回答的是
「上游还在不在发包」，与仿真跑多快无关。本机软件渲染实时因子只有 0.22x，
用 sim time 会把 0.6s 的窗口拖成 2.7s 墙钟，上游已经死了还在转发旧指令。
反过来 Nav2 空闲时 20Hz 的发包在墙上也只有约 4Hz，0.6s 窗口照样容得下。

优先级高 ≠ 永久占用：跟随节点在人丢了以后仍在发零速，但 `*_timeout` 到点后
（节点被停掉的情况下）它会掉出活跃集，Nav2 自然接管。要让它在回充/告警期间
真正让出场，由 `lightdog_mission` 发 `/follow/enable=false`，跟随器会停发。

## 安全闸：谁都不能在急停时动车

仲裁之上还有两道硬闸（都在本节点里，因为它们必须作用在**唯一的**收口处）：

- `/safety/estop`（Bool，**闭锁**）：人工急停。语音喊「急停」、手机端点按钮、
  或者遥控器上的物理按钮（由固件上报）都往这里发 true。收到 true 后一直发零速，
  直到显式收到 false —— 不做任何自动解除。
- `/mission/hold_motion`（Bool，**跟随**）：调度层在跌倒告警确认/报警期间按住轮子。
  它不是闭锁的：调度层每 1Hz 刷新，一旦它不再发（进程崩了），本节点就按
  「没有这个闸」处理，不会把机器人永久冻住。

两者分开是为了避免「两个写者互相清对方的状态」：人工急停只有人能解，
告警按住只有调度层能解，各管一条话题。

## 心跳

`/safety/heartbeat` 每个仲裁周期发一次。固件侧该做的是：**300ms 收不到
`/cmd_vel` 就断 PWM**。心跳用来区分「上位机还活着、只是故意发零速」和
「上位机死了」—— 没有这个区分，一次 panic 就会让车带着最后一次速度冲出去。

## 为什么不用 twist_mux

ROS 生态里有现成的 `twist_mux`，但这个镜像里没装，而容器内 `apt-get update`
实测跑不完（会被超时打断），重启容器后装的包也会丢。自己写一个 200 行的
节点反而更可控，而且能顺手把当前仲裁状态发布出来（`/cmd_vel_mux_state`），
排查「为什么车不动」时一眼就能看出是谁压住了谁。

## 为什么常驻而不是跟着 P2 起

`collision_monitor` 的输入现在指向本节点的输出。本节点不在，`/cmd_vel_muxed`
就断流，Nav2 的自主导航**连一条速度都发不出去**（会被 source_timeout 兜成零）。
所以它和 Nav2 一样是基础设施，由 `sim/entrypoint.sh` 常驻拉起，P2 只是往
里面多接一个跟随源。

## 话题

    订阅  /cmd_vel_smoothed  geometry_msgs/Twist   Nav2（velocity_smoother 的输出）
    订阅  /cmd_vel_follow    geometry_msgs/Twist   跟随控制器
    订阅  /cmd_vel_dock      geometry_msgs/Twist   回充控制器
    订阅  /safety/estop      std_msgs/Bool         人工急停（闭锁）
    订阅  /mission/hold_motion std_msgs/Bool       调度层按住（告警期间）
    发布  /cmd_vel_muxed     geometry_msgs/Twist   仲裁结果（接 collision_monitor）
    发布  /cmd_vel_mux_state std_msgs/String       当前生效的源名 / estop / hold
    发布  /safety/heartbeat  std_msgs/Bool         给固件看门狗
"""

import os
import sys
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist

from lightdog_qos import SAFETY_QOS


def clamp_speed(value, limit):
    """按上限钳制；limit <= 0 表示不限制。NaN 一律归零（宁可不动）。"""
    try:
        v = float(value)
    except (TypeError, ValueError):
        return 0.0
    if v != v:                     # NaN
        return 0.0
    if limit is None or limit <= 0.0:
        return v
    return max(-limit, min(limit, v))


class CmdVelSource:
    """一个速度输入源：话题、优先级、源窗口（timeout）、指令窗口（hold）。

    两个窗口都是**墙钟**，但会按上游实测发布周期自适应放大（见 _window）。
    原因：上游按 ROS 时钟定时（跟随节点 10Hz），仿真实时因子 0.21x 时墙钟上只有
    4.8Hz、间隔最大 0.32s。固定 0.30s 的 hold 会被这种正常发包反复触发，
    表现为仲裁在 follow / follow(stale) 之间抖、车走走停停（实测车完全不动）。
    """

    __slots__ = ('name', 'topic', 'prio', 'timeout', 'hold',
                 'msg', 'stamp', 'count', 'period')

    def __init__(self, name, topic, prio, timeout, hold):
        self.name = name
        self.topic = topic
        self.prio = prio
        self.timeout = timeout
        # hold 不能超过 timeout，否则「源已经掉了」这条判断永远先命中，
        # hold 形同虚设（配置写反时不至于静默失效）。
        self.hold = max(0.05, min(float(hold), timeout))
        self.msg = None       # 最后一条消息
        self.stamp = None     # 收到时刻（time.monotonic）
        self.count = 0
        self.period = 0.0     # 实测发布间隔（墙钟 EWMA），0 表示还没测出来

    def touch(self, now, msg):
        """记录一条新指令，顺便估计上游真实发布周期。"""
        if self.stamp is not None:
            dt = now - self.stamp
            # 只采纳合理区间，避免暂停/恢复或重启后的第一个间隔把周期带偏
            if 1e-3 < dt < 5.0:
                self.period = dt if self.period <= 0.0 else (0.7 * self.period + 0.3 * dt)
        self.stamp = now
        self.msg = msg
        self.count += 1

    def _window(self, base, k):
        """配置窗口与「k 个实测周期」取大者。

        真机 RTF≈1 时 k*period 通常小于配置值，行为和原来完全一致；
        仿真慢速时自动放宽，但**仍然有限**：上游一死，最多 k 个周期就判过期。
        """
        if self.period <= 0.0:
            return base
        return max(base, k * self.period)

    def fresh_window(self):
        """指令新鲜窗口（实际生效值）。"""
        return self._window(self.hold, 2.0)

    def alive_window(self):
        """源存活窗口（实际生效值）。"""
        return self._window(self.timeout, 4.0)

    def is_active(self, now):
        """在源窗口内收到过消息就算活跃（决定谁能赢得仲裁）。"""
        if self.stamp is None:
            return False
        return (now - self.stamp) <= self.alive_window()

    def is_fresh(self, now):
        """最后一条指令还够新吗？不够新就只能发零速，不能照抄。"""
        if self.stamp is None:
            return False
        return (now - self.stamp) <= self.fresh_window()


class TwistMux(Node):

    def __init__(self):
        super().__init__('lightdog_twist_mux')

        self.declare_parameter('nav_topic', '/cmd_vel_smoothed')
        self.declare_parameter('nav_priority', 10)
        self.declare_parameter('nav_timeout', 0.6)
        self.declare_parameter('nav_hold', 0.30)

        self.declare_parameter('follow_topic', '/cmd_vel_follow')
        self.declare_parameter('follow_priority', 20)
        self.declare_parameter('follow_timeout', 0.6)
        self.declare_parameter('follow_hold', 0.30)

        self.declare_parameter('dock_topic', '/cmd_vel_dock')
        self.declare_parameter('dock_priority', 30)
        self.declare_parameter('dock_timeout', 1.0)
        # docking_server 的指令周期比跟随慢（状态切换时会停顿），给宽一点
        self.declare_parameter('dock_hold', 0.50)

        self.declare_parameter('out_topic', '/cmd_vel_muxed')
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('zero_on_idle', True)
        # 安全闸（见文件头）。两条话题分开是为了让「人工急停」和「告警按住」
        # 各有唯一写者，不会互相清状态。
        self.declare_parameter('estop_topic', '/safety/estop')
        self.declare_parameter('hold_topic', '/mission/hold_motion')
        self.declare_parameter('heartbeat_topic', '/safety/heartbeat')
        # 急停闭锁的落盘路径。空 = 不落盘（仿真默认：容器一重启所有状态本来就没了）。
        #
        # 真机必须设成 /var/lib/lightdog/estop —— 因为部署层给本节点配的是
        # `Restart=always`：进程崩了会自动拉起，而闭锁只存在内存里，
        # 重启后 self.estop=False，**一辆按着急停的车就又能动了**。
        # 这不是理论风险，正是「自动重启」和「急停不过期」这两条语义的正面冲突。
        self.declare_parameter('estop_state_path', '')
        # 平台物理上限，作用在**输出**上：任何上游（跟随 P 控制、Nav2、
        # docking_server）算错了，最终发给底层的速度都不会超过这里。
        # 0 表示不限制。真机上按电机实际能力设（本项目底盘建议 0.30 / 1.0）。
        self.declare_parameter('max_linear', 0.50)
        self.declare_parameter('max_angular', 1.50)

        p = self.get_parameter
        # 顺序即同优先级下的偏好：dock > follow > nav
        self.sources = [
            CmdVelSource('dock', str(p('dock_topic').value),
                         int(p('dock_priority').value),
                         float(p('dock_timeout').value),
                         float(p('dock_hold').value)),
            CmdVelSource('follow', str(p('follow_topic').value),
                         int(p('follow_priority').value),
                         float(p('follow_timeout').value),
                         float(p('follow_hold').value)),
            CmdVelSource('nav', str(p('nav_topic').value),
                         int(p('nav_priority').value),
                         float(p('nav_timeout').value),
                         float(p('nav_hold').value)),
        ]
        self.zero_on_idle = bool(p('zero_on_idle').value)
        self.max_lin = float(p('max_linear').value)
        self.max_ang = float(p('max_angular').value)
        self._warned_clamp = False
        rate = max(1.0, float(p('publish_rate').value))

        self.pub = self.create_publisher(Twist, str(p('out_topic').value), 10)
        self.pub_state = self.create_publisher(String, '/cmd_vel_mux_state', 10)
        self.pub_beat = self.create_publisher(
            Bool, str(p('heartbeat_topic').value), 10)

        self.estop = False            # 闭锁：只有显式的 false 能解
        self.hold = False             # 调度层按住：它停发就自动放开（见 _hold_stale）
        self._hold_stamp = 0.0
        self.hold_timeout = 3.0       # 必须 > 调度层 1Hz 的刷新周期

        # 从磁盘恢复闭锁。**必须早于任何订阅建立**，否则启动那一刻会先按
        # estop=False 放行几拍，车会动一下 —— 这正是「明明按了急停它却动」。
        self.estop_path = str(p('estop_state_path').value or '').strip()
        self._estop_republish = False
        if self.estop_path:
            self.estop = self._load_estop()
            if self.estop:
                self._estop_republish = True      # 见 _tick：等发现完成再回发锁存
                self.get_logger().error(
                    f'从 {self.estop_path} 恢复了急停闭锁 —— 本节点重启过，'
                    f'车仍然是锁住的。确认现场安全后再显式解除')

        # 急停状态的**长期持有者**。
        #
        # 为什么本节点要回发自己刚收到的值：`/safety/estop` 的写者有 CLI、
        # 语音、网页按钮，它们都是短命或有条件存活的；TRANSIENT_LOCAL 的锁存
        # 样本随**最后一个 writer 退出**而消失。于是出现过这种情况：
        # 用 CLI 按下急停 -> 只重启 mission -> 新 mission 收不到任何锁存值，
        # 从 estop=False 起步，而 mux 还老老实实锁着。
        # 本节点是 entrypoint 起的常驻进程，只要它活着，锁存就一直有主人，
        # 后来者一定能继承「已经急停」。
        self.pub_estop = self.create_publisher(
            Bool, str(p('estop_topic').value), SAFETY_QOS)

        self.create_subscription(
            Bool, str(p('estop_topic').value), self._on_estop, SAFETY_QOS)
        self.create_subscription(
            Bool, str(p('hold_topic').value), self._on_hold, 10)

        self.active_name = None       # 上一个周期的赢家，用来只打切换日志
        self.subs = []
        for src in self.sources:
            # 用默认参数绑定的 lambda 捕获当前 src，避免闭包陷阱
            self.subs.append(self.create_subscription(
                Twist, src.topic, lambda m, s=src: self._on_twist(m, s), 10))

        self.create_timer(1.0 / rate, self._tick)

        desc = ', '.join(
            f'{s.name}={s.topic}(prio {s.prio}, timeout {s.timeout:.1f}s, '
            f'hold {s.hold:.2f}s)'
            for s in sorted(self.sources, key=lambda x: -x.prio))
        self.get_logger().info(f'TwistMux ready: {desc} -> {p("out_topic").value}')

    # ---------- 回调 ----------

    def _on_twist(self, msg: Twist, src: CmdVelSource):
        src.touch(time.monotonic(), msg)

    def _on_estop(self, msg: Bool):
        v = bool(msg.data)
        if v == self.estop:
            return
        if v:
            self.get_logger().warn(
                '急停闭锁（/safety/estop=true）-> 输出零速，直到显式解除')
        else:
            self.get_logger().info('急停已解除')
        self.estop = v
        self._persist_estop(v)
        # 回发一份（同一话题、同一 QoS）。上面已经 `v == self.estop` 早退，
        # 所以自己收到自己的消息不会再发，不会形成循环。
        try:
            self.pub_estop.publish(Bool(data=v))
        except Exception:                      # 发布失败不该影响刹车本身
            pass

    # ---------- 闭锁落盘 ----------

    def _load_estop(self):
        """读回上次的闭锁状态。

        读不懂就当**锁着**：这个方向的错误是「车不动、日志让你去删文件」，
        反方向是「以为锁着其实能动」。安全件只接受前一种。
        """
        try:
            with open(self.estop_path, 'r', encoding='utf-8') as f:
                raw = f.read().strip().lower()
        except FileNotFoundError:
            return False
        except OSError as e:
            self.get_logger().error(
                f'读不了急停状态文件 {self.estop_path}（{e}），按「已急停」处理。'
                f'确认安全后删掉该文件，或用 estop=false 显式解除')
            return True
        if raw in ('0', 'false', 'no', 'off'):
            return False
        if raw in ('1', 'true', 'yes', 'on'):
            return True
        self.get_logger().error(
            f'急停状态文件 {self.estop_path} 内容无法识别（{raw[:20]!r}），'
            f'按「已急停」处理。删掉该文件即可清除')
        return True

    def _persist_estop(self, on):
        """把闭锁状态原子地落盘。

        写盘失败**不回滚**内存状态：刹车已经生效了，这时回头改成「没锁」才是
        真正的危险。只 warn，让运维知道下次重启会丢这个状态。
        """
        if not self.estop_path:
            return
        try:
            if on:
                d = os.path.dirname(self.estop_path)
                if d:
                    os.makedirs(d, exist_ok=True)
                tmp = self.estop_path + '.tmp'
                # 先写临时文件再 rename：掉电/被 kill 时不会留下半截内容被
                # _load_estop 误判成 True（那会让车永远起不来）。
                with open(tmp, 'w', encoding='utf-8') as f:
                    f.write('1')
                    f.flush()
                    os.fsync(f.fileno())
                os.replace(tmp, self.estop_path)
            else:
                try:
                    os.unlink(self.estop_path)
                except FileNotFoundError:
                    pass
        except OSError as e:
            self.get_logger().warn(
                f'急停状态落盘失败（{e}）：当前闭锁仍然有效，'
                f'但本节点一旦重启就会忘记它。检查 {self.estop_path} 的权限')

    def _on_hold(self, msg: Bool):
        v = bool(msg.data)
        if v != self.hold:
            self.get_logger().info(
                '调度层按住轮子（告警中）' if v else '调度层放开轮子')
        self.hold = v
        self._hold_stamp = time.monotonic()

    def _gate(self, now):
        """返回当前生效的安全闸名；没有闸返回 None。

        hold 会过期：调度层崩了不会留下一个永久冻结的机器人。而 estop 不过期，
        因为它是人按下去的 —— 只有人能取消。
        """
        if self.estop:
            return 'estop'
        if self.hold:
            if self._hold_stamp > 0.0 and \
                    (now - self._hold_stamp) > self.hold_timeout:
                self.hold = False
                self._hold_stamp = 0.0
                self.get_logger().error(
                    f'{self.hold_timeout:.1f}s 没收到 /mission/hold_motion，'
                    f'判定调度层已退出，解除按住（请检查 lightdog_mission）')
                return None
            return 'hold'
        return None

    # ---------- 仲裁 ----------

    def _pick(self):
        """选出当前该生效的源；没有活跃源返回 None。"""
        now = time.monotonic()
        best = None
        for src in self.sources:          # 已按优先级从高到低排列
            if src.is_active(now):
                if best is None or src.prio > best.prio:
                    best = src
        return best

    def _tick(self):
        now = time.monotonic()

        # 心跳：给固件看门狗区分「上位机活着、故意零速」和「上位机死了」。
        # 每拍都发（20Hz），MCU 侧 300ms 收不到就断 PWM。
        self.pub_beat.publish(Bool(data=True))

        # 从磁盘恢复的闭锁：__init__ 里发的那一次很可能还在 DDS 发现之前，
        # 锁存样本没有接收者。等第一拍 timer（约 50ms 后，发现基本完成）补发一次，
        # 让后起的 mission / server 能继承「已经急停」。
        if self._estop_republish:
            self._estop_republish = False
            try:
                self.pub_estop.publish(Bool(data=True))
            except Exception:
                self._estop_republish = True   # 下拍再试，别把锁丢掉

        gate = self._gate(now)
        if gate is not None:
            # 安全闸优先于一切源，且**必须**发零速（不受 zero_on_idle 影响）。
            if self.active_name != gate:
                self.get_logger().warn(f'安全闸 {gate} 生效 -> 输出零速')
                self.active_name = gate
            self.pub_state.publish(String(data=gate))
            self.pub.publish(Twist())
            return

        best = self._pick()
        name = best.name if best is not None else 'idle'

        # 源还在窗口内，但最后一条指令已经过期 —— 上游很可能猝死了。
        # 这时**不能**把旧 Twist 继续转发，只能发零速，并在状态里标 stale，
        # 让排查的人一眼看出「不是没人要动，是发指令的那个已经没了」。
        stale = best is not None and not best.is_fresh(now)
        if stale:
            name = f'{best.name}(stale)'

        if name != self.active_name:
            if best is None:
                self.get_logger().info('仲裁: 无活跃源 -> 输出零速')
            elif stale:
                self.get_logger().warn(
                    f'仲裁: {best.name} 已 {now - best.stamp:.2f}s 没更新指令'
                    f'（窗口 {best.fresh_window():.2f}s，配置 hold {best.hold:.2f}s）'
                    f'-> 输出零速')
            else:
                self.get_logger().info(
                    f'仲裁: 切到 {best.name}（{best.topic}, prio {best.prio}）')
            self.active_name = name

        # 状态周期发布（不是只在切换时发）：否则 `ros2 topic echo --once`
        # 得等到下次切换才有消息，排查"为什么车不动"时全是超时。
        self.pub_state.publish(String(data=name))

        if best is None or stale:
            if self.zero_on_idle:
                self.pub.publish(Twist())
            return

        # 输出前按平台上限钳制。这里刻意**新建**一条消息而不是改 best.msg：
        # 改源里的那条会污染 CmdVelSource 缓存（下一拍读到的就是被改过的值），
        # 也看不出「谁超速了」。
        src = best.msg
        out = Twist()
        out.linear.x = clamp_speed(src.linear.x, self.max_lin)
        out.linear.y = clamp_speed(src.linear.y, self.max_lin)
        out.linear.z = src.linear.z
        out.angular.x = src.angular.x
        out.angular.y = src.angular.y
        out.angular.z = clamp_speed(src.angular.z, self.max_ang)

        clamped = (out.linear.x != src.linear.x or out.angular.z != src.angular.z)
        if clamped and not self._warned_clamp:
            self._warned_clamp = True
            self.get_logger().warn(
                f'{best.name} 的指令超出平台上限，已钳制：'
                f'v {src.linear.x:+.2f}->{out.linear.x:+.2f} '
                f'w {src.angular.z:+.2f}->{out.angular.z:+.2f} '
                f'（max_linear={self.max_lin}, max_angular={self.max_ang}）')
        elif not clamped:
            self._warned_clamp = False

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = TwistMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.pub.publish(Twist())    # 退出前确保下游收到停止指令
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
