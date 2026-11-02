#!/usr/bin/env python3
"""
LightDog 人体跟随控制器（P2 / 场景 1）

## 为什么不用 Nav2

路线里专门记过一笔：**跟随 ≠ 导航**。Nav2 要的是一张地图 + 一个 map 坐标系里的
目标点，人一进屋、一关门、一消失在视野里，代价地图就不知道该往哪走；而且人来
的地方机器人未必过得去，把人的位置当导航目标会让机器人硬往墙上撞。

P2 的需求是「人往哪走，灯就照到哪」，本质是**视觉伺服**：保持固定距离跟着轮廓走，
人停下机器人就停下。所以这里用最直接的 P 控制，输出 /cmd_vel。
Nav2 留给 P3 的自动回充（那里有确定的静态目标点）。

## 输入 / 输出

    订阅 /person/position  geometry_msgs/PointStamped  frame=base_link
    订阅 /person/present   std_msgs/Bool
    发布 /cmd_vel_follow   geometry_msgs/Twist

输出不直接发 `/cmd_vel`：那个话题上有 Nav2 的 collision_monitor / docking_server /
command_timeout 一群发布者，跟随指令会被它们的空闲零速淹没（P2 实测：follower
一直发 -0.15，机器人 6 秒只挪 5cm）。现在统一交给 `lightdog_twist_mux` 仲裁，
再由 collision_monitor 兜碰撞保护。话题名可用 `cmd_vel_topic` 参数改。

## 控制律

    err_dist  = x - target_distance          正=太远
    err_theta = atan2(y, x)                  正=人在左

    v = clamp(k_linear * err_dist, -0.6*max_v, max_v)，|err_dist| <= tol 时归零
    w = clamp(k_angular * err_theta, -max_w, max_w)，   |err_theta| <= ang_tol 时归零

四个安全约束（真机上这几条比控制精度重要）：
1. 距离小于 stop_distance 直接后退，不因为「还差一点没到目标距离」而继续顶上去。
2. 人离开视野超过 person_timeout 秒立刻停，宁可停在原地也不凭记忆往前冲。
   「离开视野」有两条判据：`/person/present` 到期，**或者**坐标本身到期
   （检测器可能连着发 present=True 却不再更新 position）。
3. 人在侧后方（|theta| > side_turn_only）时只转不走，避免绕着人画大圈。
4. 坐标的 frame_id 必须是 expected_frame（默认 base_link）。检测器换实现
   （比如换 YOLO 后改成发 camera_link）时宁可停住，也不能把别的坐标系下的
   点当成 base_link 的 x/y 去算速度 —— 那是一脚油门直奔墙。
5. 调度层说「现在不该你说话」时，**先发一条零速、然后彻底停发**。
   停发不是省事，是让 twist_mux 的 follow 源在超时后掉出活跃集，Nav2 才接得上
   （P2 §7.8：跟随节点一直发零速会永久压住导航）。而停发前那条零速也不能省：
   否则 mux 会把缓存里最后一条非零指令继续转发最多 `follow_hold` 秒。
   这个门是**带超时**的 —— 调度层崩了，5 秒后回落到本节点自己的 `enabled`。
"""

import math
import sys
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped, Twist


class PersonFollower(Node):

    def __init__(self):
        super().__init__('lightdog_follower')

        self.declare_parameter('target_distance', 1.20)   # 保持距离
        self.declare_parameter('distance_tol', 0.15)      # 距离死区
        self.declare_parameter('stop_distance', 0.55)     # 低于此值后退
        self.declare_parameter('k_linear', 0.9)
        self.declare_parameter('max_linear', 0.25)
        self.declare_parameter('k_angular', 1.4)
        self.declare_parameter('max_angular', 0.70)
        self.declare_parameter('angle_tol', 0.06)         # 角度死区
        self.declare_parameter('side_turn_only', 0.9)     # 超过此角度只转不走
        self.declare_parameter('person_timeout', 1.5)
        # 坐标必须在这个坐标系下。/person/position 的语义（x 前 / y 左，米）
        # 只对 base_link 成立；换检测器实现时若不改本参数，就会把 camera_link
        # 下的点当 base_link 用，速度方向直接错掉。
        self.declare_parameter('expected_frame', 'base_link')
        # dock_manager 以 10Hz 发 /dock/inhibit。它要是崩了/被杀，最后那条
        # true 会永远留在跟随器里 —— 表现为「跟随彻底不工作，日志里什么都没有」。
        # 超过这个墙钟时间没再收到 inhibit 就视为回充已结束（并打一条 error）。
        self.declare_parameter('inhibit_timeout', 5.0)
        # 调度层的放行开关。它崩了以后必须能自己回到「按 enabled 工作」，
        # 否则一次 OOM 就把跟随永久关掉，而日志里什么都不会有。
        self.declare_parameter('enable_topic', '/follow/enable')
        self.declare_parameter('follow_enable_timeout', 5.0)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_follow')
        self.declare_parameter('enabled', True)

        p = self.get_parameter
        self.target = float(p('target_distance').value)
        self.tol = float(p('distance_tol').value)
        self.stop_d = float(p('stop_distance').value)
        self.k_v = float(p('k_linear').value)
        self.max_v = float(p('max_linear').value)
        self.k_w = float(p('k_angular').value)
        self.max_w = float(p('max_angular').value)
        self.ang_tol = float(p('angle_tol').value)
        self.side_only = float(p('side_turn_only').value)
        self.timeout = float(p('person_timeout').value)
        self.expected_frame = str(p('expected_frame').value).lstrip('/')
        self.inhibit_timeout = float(p('inhibit_timeout').value)
        self.enable_timeout = float(p('follow_enable_timeout').value)
        self.enabled = bool(p('enabled').value)

        self.person = None            # 最新一次人位置
        self.present = False
        # 两者都用**墙钟** time.monotonic()，不是 ROS 时钟：
        # 这两个量回答的都是「上一条消息多久以前到的」，与仿真/系统时钟无关。
        # 真机上若 use_sim_time 被误开成 true 而没有 /clock，ROS 时钟会停走，
        # 超时判据永远不触发 —— 表现为机器人对着早就离开的人一直开。
        self.last_present = None      # 最后一次 present=True 的到达时刻
        self.pos_wall = 0.0           # 最后一次 /person/position 的到达时刻
        # 回充期间让位。dock_manager 在 ALIGNING/DOCKING/DOCKED/UNDOCKING 时把
        # /dock/inhibit 置 true（DOCKED 也置 true，否则充电时人一经过就会被开走）。
        # 只靠优先级其实也能压住（dock 30 > follow 20），但跟随节点会一直发
        # 非零速度去"抢"，docking 一结束人还在原地，机器人立刻掉头追人 ——
        # 这不是低电回充该有的行为。所以显式认这个标志。
        self.inhibited = False
        self.inhibit_stamp = 0.0      # 最后一条 /dock/inhibit 的墙钟时刻
        self._warned_frame = ''       # 已报过的 frame_id（避免每拍刷日志）
        self._warned_pos = False
        # 调度层放行开关：None = 从没收到过（按自身 enabled 工作）
        self.enable_cmd = None
        self.enable_stamp = 0.0
        self._warned_enable = False
        self._silent = False          # 已经停发（只用来保证「零速只发一次」）

        self.create_subscription(PointStamped, '/person/position', self._on_pos, 10)
        self.create_subscription(Bool, '/person/present', self._on_present, 10)
        self.create_subscription(Bool, '/dock/inhibit', self._on_inhibit, 10)
        self.create_subscription(
            Bool, str(p('enable_topic').value), self._on_enable, 10)
        self.cmd_topic = str(p('cmd_vel_topic').value)
        self.pub = self.create_publisher(Twist, self.cmd_topic, 10)

        self.create_timer(0.1, self._tick)   # 10 Hz

        self.get_logger().info(
            f'LightDog follower ready: target={self.target:.2f}m '
            f'tol=±{self.tol:.2f}m stop<{self.stop_d:.2f}m '
            f'vmax={self.max_v:.2f}m/s wmax={self.max_w:.2f}rad/s '
            f'enabled={self.enabled}')

    # ---------- 回调 ----------

    def _on_pos(self, msg: PointStamped):
        self.person = msg
        self.pos_wall = time.monotonic()

    def _on_present(self, msg: Bool):
        self.present = bool(msg.data)
        if self.present:
            self.last_present = time.monotonic()

    def _on_inhibit(self, msg: Bool):
        if bool(msg.data) != self.inhibited:
            self.get_logger().info(
                '回充占用中，跟随让位' if msg.data else '回充结束，跟随恢复')
        self.inhibited = bool(msg.data)
        self.inhibit_stamp = time.monotonic()

    def _on_enable(self, msg: Bool):
        v = bool(msg.data)
        if v != self.enable_cmd:
            self.get_logger().info(
                '调度层放行：恢复跟随' if v else '调度层收回控制权：停止跟随')
        self.enable_cmd = v
        self.enable_stamp = time.monotonic()

    # ---------- 控制 ----------

    def _person_lost(self) -> bool:
        """人是否已经「不该再跟」了。

        两条判据缺一不可：
        * `/person/present` 到期 —— 常规丢检；
        * 坐标本身上一次到达已超时 —— 检测器存在「还在发 present=True，但
          连续几帧没检出目标、于是不再发 position」的窗口（person_detector
          的 lost_timeout 就是干这个的）。只看 present 会拿一份旧坐标继续走，
          表现为「人已经拐弯，机器人还朝原来的方向直冲」。

        两条都按墙钟计龄，理由见 __init__ 里的注释。
        """
        if self.last_present is None:
            return True
        now = time.monotonic()
        if (now - self.last_present) > self.timeout:
            return True
        if self.pos_wall <= 0.0:
            return True               # 还没收到过坐标，等同于没看见人
        if (now - self.pos_wall) > self.timeout:
            if not self._warned_pos:
                self._warned_pos = True
                self.get_logger().warn(
                    f'收到 present=True 但坐标已 {now - self.pos_wall:.1f}s 没更新，'
                    f'按丢失处理并停车（检查 /person/position 发布频率）')
            return True
        self._warned_pos = False
        return False

    def _check_frame(self) -> bool:
        """坐标必须是我期望的那个坐标系，否则拒绝使用。"""
        want = self.expected_frame
        if not want:
            return True
        if self.person is None:       # 没有坐标当然不能用（正常路径已被 _person_lost 挡掉）
            return False
        got = ''
        try:
            got = str(self.person.header.frame_id or '')
        except AttributeError:
            got = ''
        got = got.lstrip('/')
        if got == want:
            self._warned_frame = ''
            return True
        if self._warned_frame != got:
            self._warned_frame = got
            self.get_logger().error(
                f'/person/position 的 frame_id={got!r} 与 expected_frame={want!r} '
                f'不一致，拒绝使用该坐标并停车。'
                f'（若确实换了坐标系，请同时改本节点的 expected_frame，'
                f'或让检测器在发布前先变换到 {want}）')
        return False

    def _follow_gate(self) -> bool:
        """调度层是否允许本节点控制底盘。

        没收到过 `/follow/enable` 时按自己的 `enabled` 参数工作 —— 这样单独跑
        跟随（不带调度层）时行为与以前完全一致，不至于一升级就「车不动了」。
        """
        if not self.enabled:
            return False
        if self.enable_cmd is None:
            return True
        if self.enable_cmd:
            return True
        silent = time.monotonic() - self.enable_stamp
        if silent <= self.enable_timeout:
            return False
        # 调度层不再刷新了：宁可恢复跟随，也不能因为一个上游节点死掉就永久
        # 失去跟随能力。安全由 mux 的零速兜底，不靠这里「一直关着」。
        if not self._warned_enable:
            self._warned_enable = True
            self.get_logger().error(
                f'{silent:.1f}s 没收到 /follow/enable，判定调度层已退出，'
                f'回落到本节点的 enabled={self.enabled}（请检查 lightdog_mission）')
        self.enable_cmd = None
        self.enable_stamp = 0.0
        return True

    def _tick(self):
        cmd = Twist()

        if not self._follow_gate():
            # 先发一条零速，然后彻底停发。理由见文件头第 5 条：
            # 不停发 -> mux 的 follow 源永远活跃 -> Nav2 永远拿不到方向盘。
            if not self._silent:
                self.pub.publish(cmd)
                self._silent = True
            return
        self._silent = False
        self._warned_enable = False

        inhibited = self.inhibited
        if inhibited and self.inhibit_stamp > 0.0:
            silent = time.monotonic() - self.inhibit_stamp
            if silent > self.inhibit_timeout:
                # dock_manager 已经不再发 inhibit 了（崩了/被 kill），
                # 不能再让这条陈旧的 true 把跟随永久锁死。
                self.inhibited = False
                self.inhibit_stamp = 0.0
                inhibited = False
                self.get_logger().error(
                    f'{silent:.1f}s 没收到 /dock/inhibit，判定回充节点已退出，'
                    f'跟随恢复（若回充仍在进行，请检查 dock_manager）')

        if (not self.enabled or inhibited
                or self._person_lost() or not self._check_frame()):
            self.pub.publish(cmd)     # 持续发零速，防止底层保持上一条指令
            return

        x = float(self.person.point.x)      # 前向距离
        y = float(self.person.point.y)      # 左正
        if x <= 0.05:
            self.pub.publish(cmd)
            return

        err_theta = math.atan2(y, x)

        # --- 线速度 ---
        if x < self.stop_d:
            # 太近：按差距后退，限幅 0.6*max_v
            cmd.linear.x = -min(0.6 * self.max_v, self.k_v * (self.stop_d - x))
        elif abs(x - self.target) <= self.tol:
            cmd.linear.x = 0.0
        else:
            v = self.k_v * (x - self.target)
            cmd.linear.x = max(-0.6 * self.max_v, min(self.max_v, v))

        # --- 角速度 ---
        if abs(err_theta) <= self.ang_tol:
            cmd.angular.z = 0.0
        else:
            cmd.angular.z = max(-self.max_w, min(self.max_w, self.k_w * err_theta))

        # 人在侧后方：只转不走
        if abs(err_theta) > self.side_only:
            cmd.linear.x = 0.0

        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PersonFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.pub.publish(Twist())    # 退出前确保机器人停下
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
