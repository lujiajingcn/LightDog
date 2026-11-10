#!/usr/bin/env python3
"""LightDog 任务调度层（模式仲裁）

## 这个节点解决什么问题

P1~P5 每个阶段都只回答自己那一小段问题，于是留下了两个**架构级**缺口：

1. **跟随会压死导航**。`lightdog_follower` 只要在跑就 10Hz 发 `/cmd_vel_follow`，
   哪怕人早就走了也是发零速。而 twist_mux 里 follow(20) 的优先级高于 nav(10)，
   所以「跟随源一直活跃」= Nav2 永远拿不到方向盘。P2 文档 §7.8 记的就是这条：
   想在跟随和导航之间切换，只能手动杀掉跟随节点。
2. **没人负责「从远处回充」**。P3 的 `dock_manager` 边界是「已经在桩附近 →
   精确停靠」，`navigate_to_staging` 默认 false。真机上机器人白天在客厅、
   晚上低电，它不会自己走回去。

这两件事都不是某个阶段内部的 bug，而是**缺一个决定「现在该谁说话」的地方**。
本节点就是那一层，而且刻意只做决策、不产生速度：

    读  /person/present、/dock/state、/dock/battery_ratio、/alert/state
    写  /follow/enable      （跟随器据此决定发不发速度，见下）
        /mission/hold_motion（告警期间把轮子按住，比让各节点自己收敛可靠）
        /dock/command       （dock_manager 已有的命令入口）
        /safety/estop 只读  （人工急停一旦闭锁，本节点也必须让路）
        Nav2 navigate_to_pose（只在「回桩接近」和「巡游」时用）

## 模式与优先级

    ESTOP    急停闭锁（人按的 / 网页点的 / 语音喊的）—— 谁都别动
    ALERT_HOLD 跌倒告警确认中/已报警 —— 别动，等人起身（人可能就躺在轮子前）
    DOCKING  回充进行中（dock_manager 说了算）
    DOCKED   在桩上充电（同样别动，否则充电时人一经过就被开走）
    APPROACH 低电，导航到桩前 staging 点，到了再交给 dock_manager
    FOLLOW   有人 —— 跟随（优先级高于导航，这是场景 1 的产品定义）
    PATROL   无人、电量够、开了巡游 —— 按路点转圈（场景 3）
    IDLE     待机
    MANUAL   人工接管：本节点只放行跟随，不自动切模式、不发导航目标

优先级就是上面这个顺序，`decide()` 里逐条 if。**顺序本身是安全设计**：
告警压过回充，回充压过跟随，跟随压过巡游。

## 为什么用 /follow/enable 而不是直接杀节点

跟随器收到 `false` 会**先发一条零速、然后彻底停发**。停发这一下很关键：
twist_mux 的 follow 源在 `follow_timeout`(0.6s) 后掉出活跃集，nav 才真正接得上。
如果改成「继续发零速」，源就永远活跃，nav 又拿不到方向盘了 —— 这正是 P2 的坑。

停发之前必须发那一条零速，否则 mux 会把缓存里最后一条**非零**指令继续转发最多
`follow_hold`(0.3s)，车会自己往前溜一小段。

反过来，`/follow/enable` 是有超时的：本节点挂了以后，跟随器在
`follow_enable_timeout`(5s) 之后回落到自己 yaml 里的 `enabled` 参数。
不做这个回落，调度层一崩，跟随就永久失效，而且日志里什么都没有。

## 为什么低电阈值比 dock_manager 高

本节点 `approach_ratio`(0.35) > dock_manager `low_ratio`(0.25)，两者**各管一段**：

- 35%：本节点开始导航到桩前（这段时间机器人可能还在另一个房间）
- 25%：dock_manager 自己判低电、发起精确停靠（P3 已验证的路径，不改）

两者同时命中时 `dock_manager` 赢：它一进 ALIGNING/DOCKING，本节点立刻取消导航
目标并让出。**不抢所有权**，因为它那套状态机是经过验收的，而调度层只是
「把车送到门口」的角色。这样即使本节点崩了，P3 的低电保护依然在。

## 「到了就交给 dock_manager」具体怎么交

    导航到 staging（桩前 staging_distance 米、朝向桩）
      → NavigateToPose 成功
      → 发 /dock/command "dock"
      → dock_manager: ALIGNING（ArUco 视觉对准）→ DOCKING → DOCKED

对准不放在这里：四驱 skid steer 的 odom yaw 高估 35%，导航只能把车送到
「大概对着桩」的位置，最后那几度必须靠视觉。P3 已经有这段逻辑，复用。

## 巡游（场景 3：白天当移动监控）

`patrol_enabled` 默认 **false**。打开后按 `patrol_waypoints`（`"x,y,yaw; x,y,yaw"`，
map 坐标）循环导航，每个点停 `patrol_dwell` 秒。有人出现立刻中断巡游
去跟随，人走了从**当前点重新开始**（不记忆进度，简单可预测）。

## 话题

    订阅  /person/present        std_msgs/Bool
    订阅  /dock/state            std_msgs/String
    订阅  /dock/battery_ratio    std_msgs/Float32
    订阅  /alert/state           std_msgs/String   JSON
    订阅  /safety/estop          std_msgs/Bool     只读：人工急停
    订阅  /mission/command       std_msgs/String    见 _on_command
    发布  /mission/state         std_msgs/String    JSON，给手机端和排障
    发布  /mission/mode          std_msgs/String    纯模式名（方便 echo）
    发布  /follow/enable         std_msgs/Bool
    发布  /mission/hold_motion   std_msgs/Bool
    发布  /dock/command          std_msgs/String
    action client /navigate_to_pose  nav2_msgs/NavigateToPose
"""

import json
import math
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Bool, Float32, String

from lightdog_qos import SAFETY_QOS


# 告警状态里哪几个必须按住轮子。
# CONFIRMING=正在语音问询、ALARM=已报警、ACKED=家属已知晓 —— 这三个阶段人
# 大概率还躺在地上，机器人这时候挪动毫无价值，风险却是实打实的。
ALERT_HOLD_STATES = ('CONFIRMING', 'ALARM', 'ACKED')


def make_quaternion(yaw):
    """只绕 z 转的四元数（导航目标只用到 yaw）。"""
    return (0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw))


def parse_waypoints(items):
    """把 "x,y,yaw; x,y,yaw"（或字符串列表）解析成 [(x, y, yaw), ...]。

    为什么是一个**字符串**而不是 ROS 的字符串数组参数：空数组在 ROS 2 里推不出
    类型，声明成空列表之后再读会抛 ParameterUninitializedException
    （descriptor 里写的 type 会被 rclpy 忽略），节点直接起不来 —— 这是实测
    踩到的。一条分号分隔的字符串在 yaml、`ros2 param set`、命令行里都顺手，
    也避开了这个坑。

    坏数据整条丢弃并返回解析失败计数，让调用方决定要不要报错 ——
    绝不猜一个默认点出来（那会让机器人跑向一个没人指定的位置）。
    """
    if isinstance(items, str):
        chunks = [c for c in items.replace('\n', ';').split(';') if c.strip()]
    else:
        chunks = [c for c in (items or []) if str(c).strip()]

    out, bad = [], 0
    for item in chunks:
        try:
            parts = [p.strip() for p in str(item).split(',')]
            if len(parts) < 2:
                bad += 1
                continue
            x = float(parts[0])
            y = float(parts[1])
            yaw = float(parts[2]) if len(parts) > 2 else 0.0
            if not all(math.isfinite(v) for v in (x, y, yaw)):
                bad += 1
                continue
            out.append((x, y, yaw))
        except (TypeError, ValueError):
            bad += 1
    return out, bad


class Mission(Node):

    # 模式常量（测试和日志都直接用字符串，避免引入枚举序列化问题）
    ESTOP = 'ESTOP'
    ALERT = 'ALERT_HOLD'
    DOCKING = 'DOCKING'
    DOCKED = 'DOCKED'
    APPROACH = 'DOCK_APPROACH'
    FOLLOW = 'FOLLOW'
    PATROL = 'PATROL'
    IDLE = 'IDLE'
    MANUAL = 'MANUAL'

    def __init__(self):
        super().__init__('lightdog_mission')

        # ---- 时间窗口（全部是墙钟，见文件头）----
        self.declare_parameter('tick_hz', 2.0)
        self.declare_parameter('person_timeout', 2.0)      # present 超过这么久没更新 = 无人
        self.declare_parameter('dock_state_timeout', 5.0)  # dock 状态断流 = 回充进程已退出
        self.declare_parameter('alert_timeout', 5.0)       # 告警状态断流 = 告警节点已退出

        # ---- 模式开关 ----
        self.declare_parameter('auto_mode', True)          # false = 启动即 MANUAL
        self.declare_parameter('follow_enabled', True)     # 允许自动进入 FOLLOW
        self.declare_parameter('patrol_enabled', False)
        # 一处分号一个点，点内 "x,y,yaw"（yaw 可省）。见 parse_waypoints 的注释：
        # 用字符串而不是字符串数组，是因为空数组在 ROS 2 里无法推断类型。
        self.declare_parameter('patrol_waypoints', '')
        self.declare_parameter('patrol_dwell', 5.0)

        # ---- 回充接近 ----
        self.declare_parameter('dock_approach', True)
        self.declare_parameter('approach_ratio', 0.35)     # 必须 > dock_manager.low_ratio
        self.declare_parameter('clear_ratio', 0.45)        # 回到这个值以上才解除
        self.declare_parameter('approach_confirm', 5.0)    # 持续低电多久才动
        self.declare_parameter('approach_retry_cooldown', 60.0)
        self.declare_parameter('approach_timeout', 180.0)  # 单次导航超时
        # 桩在 map 系下的位姿。仿真：世界系 (-2.3,0.5) 减去出生点 (-1.8,0.5)。
        self.declare_parameter('dock_pose_x', -0.5)
        self.declare_parameter('dock_pose_y', 0.0)
        self.declare_parameter('dock_pose_yaw', 0.0)
        self.declare_parameter('staging_distance', 1.0)    # 停在桩前多远
        self.declare_parameter('map_frame', 'map')

        # ---- 话题 ----
        self.declare_parameter('follow_enable_topic', '/follow/enable')
        self.declare_parameter('hold_topic', '/mission/hold_motion')
        self.declare_parameter('nav_action', '/navigate_to_pose')

        p = self.get_parameter
        self.person_timeout = float(p('person_timeout').value)
        self.dock_timeout = float(p('dock_state_timeout').value)
        self.alert_timeout = float(p('alert_timeout').value)
        self.auto_mode = bool(p('auto_mode').value)
        self.follow_enabled = bool(p('follow_enabled').value)
        self.patrol_enabled = bool(p('patrol_enabled').value)
        self.patrol_dwell = float(p('patrol_dwell').value)
        self.dock_approach = bool(p('dock_approach').value)
        self.approach_ratio = float(p('approach_ratio').value)
        self.clear_ratio = max(self.approach_ratio + 0.01,
                               float(p('clear_ratio').value))
        self.approach_confirm = float(p('approach_confirm').value)
        self.approach_cooldown = float(p('approach_retry_cooldown').value)
        self.approach_timeout = float(p('approach_timeout').value)
        self.dock_x = float(p('dock_pose_x').value)
        self.dock_y = float(p('dock_pose_y').value)
        self.dock_yaw = float(p('dock_pose_yaw').value)
        self.staging_distance = float(p('staging_distance').value)
        self.map_frame = str(p('map_frame').value)
        self.nav_action = str(p('nav_action').value)

        # ---- 输入快照 ----
        self.person_present = False
        self.person_wall = 0.0
        self.dock_state = ''
        self.dock_wall = 0.0
        self.ratio = None
        self.ratio_wall = 0.0
        self.alert_state = ''
        self.alert_wall = 0.0
        self.estop = False

        # ---- 决策状态 ----
        self.manual = not self.auto_mode
        self.mode = ''
        self.mode_since = 0.0
        self.reason = '启动'
        self.approach_wanted = False
        self._low_since = None
        self._approach_cooldown_until = 0.0
        # 「手动要求回充」与「低电自动判定」必须分开记：共用 approach_wanted
        # 的话，用户满电时点「回充」会被 update_approach 当场上限迟滞清掉
        # （电量 >= clear_ratio -> 解除接近），表现是命令被无声吞掉。
        self._approach_manual = False

        # 巡游
        self.waypoints, bad = parse_waypoints(p('patrol_waypoints').value)
        self._wp_index = 0
        self._patrol_wait_until = 0.0
        if bad:
            self.get_logger().error(
                f'patrol_waypoints 有 {bad} 条无法解析，已丢弃（格式 "x,y,yaw"）')
        if self.patrol_enabled and not self.waypoints:
            self.get_logger().warn(
                'patrol_enabled=true 但 patrol_waypoints 为空，巡游不会生效')

        # ---- 跟随放行：只在变化时发，另外 1Hz 刷新一次 ----
        self._follow_sent = None
        self._follow_beat = 0.0
        self._hold_sent = None
        self._hold_beat = 0.0

        # ---- 导航目标 ----
        self._nav_state = None        # None / 'sending' / 'active'
        self._nav_tag = None
        self._nav_epoch = 0
        self._nav_handle = None
        self._nav_sent_at = 0.0
        self._nav_cooldown_until = 0.0
        self._nav_fail = 0
        self._nav_warned_unavailable = False

        # ---- 订阅 ----
        self.create_subscription(Bool, '/person/present', self._on_person, 10)
        self.create_subscription(String, '/dock/state', self._on_dock, 10)
        self.create_subscription(
            Float32, '/dock/battery_ratio', self._on_ratio, 10)
        self.create_subscription(String, '/alert/state', self._on_alert, 10)
        self.create_subscription(Bool, '/safety/estop', self._on_estop, SAFETY_QOS)
        self.create_subscription(
            String, '/mission/command', self._on_command, 10)

        # ---- 发布 ----
        self.pub_state = self.create_publisher(String, '/mission/state', 10)
        self.pub_mode = self.create_publisher(String, '/mission/mode', 10)
        self.pub_follow = self.create_publisher(
            Bool, str(p('follow_enable_topic').value), 10)
        self.pub_hold = self.create_publisher(
            Bool, str(p('hold_topic').value), 10)
        self.pub_dock = self.create_publisher(String, '/dock/command', 10)

        self.nav_client = ActionClient(self, NavigateToPose, self.nav_action)

        hz = max(0.5, float(p('tick_hz').value))
        self.create_timer(1.0 / hz, self.tick)

        self.get_logger().info(
            f'LightDog mission ready: auto={self.auto_mode} '
            f'follow={self.follow_enabled} patrol={self.patrol_enabled} '
            f'approach={self.dock_approach} '
            f'(低电 {self.approach_ratio:.0%} 接近 / 解除 {self.clear_ratio:.0%}) '
            f'dock_map=({self.dock_x:.2f},{self.dock_y:.2f},{self.dock_yaw:.2f}) '
            f'staging={self.staging_distance:.2f}m')

    # ---------- 回调 ----------

    def _on_person(self, msg: Bool):
        self.person_present = bool(msg.data)
        self.person_wall = time.monotonic()

    def _on_dock(self, msg: String):
        self.dock_state = str(msg.data).strip()
        self.dock_wall = time.monotonic()

    def _on_ratio(self, msg: Float32):
        try:
            v = float(msg.data)
        except (TypeError, ValueError):
            return
        if math.isfinite(v):
            self.ratio = max(0.0, min(1.0, v))
            self.ratio_wall = time.monotonic()

    def _on_alert(self, msg: String):
        st = ''
        try:
            st = str(json.loads(msg.data).get('state', '')).upper()
        except (ValueError, TypeError, AttributeError):
            st = msg.data.strip().upper()      # 兼容裸字符串（调试时手发）
        self.alert_state = st
        self.alert_wall = time.monotonic()

    def _on_estop(self, msg: Bool):
        v = bool(msg.data)
        if v != self.estop:
            self.get_logger().warn(
                '收到急停闭锁，调度层让路' if v else '急停已解除，调度层恢复')
        self.estop = v

    def _on_command(self, msg: String):
        cmd = str(msg.data).strip().lower()
        if not cmd:
            return
        self.get_logger().info(f'收到命令: {cmd}')
        if cmd in ('follow', 'follow_on', 'come'):
            self.manual = False
            self.follow_enabled = True
            self.reason = '命令：开始跟随'
        elif cmd in ('follow_off', 'stop_follow'):
            self.follow_enabled = False
            self.reason = '命令：停止跟随'
        elif cmd in ('auto', 'auto_on'):
            self.manual = False
            self.reason = '命令：恢复自动模式'
        elif cmd in ('manual', 'manual_on'):
            self.manual = True
            self.reason = '命令：人工接管'
        elif cmd == 'patrol_on':
            self.patrol_enabled = True
            self._wp_index = 0
            self.reason = '命令：开始巡游'
            if not self.waypoints:
                # 不 warn 的话现场看到的是「说了开始巡逻、车一动不动」，
                # 而 decide() 里那条 `and self.waypoints` 把原因藏得死死的。
                self.get_logger().warn(
                    'patrol_on：patrol_waypoints 为空，巡游不会真的动起来'
                    '（先在 config/system_params.yaml 里填 "x,y,yaw; ..."）')
        elif cmd == 'patrol_off':
            self.patrol_enabled = False
            self.reason = '命令：停止巡游'
        elif cmd in ('stop', 'halt', 'all_stop'):
            # 「停下」= 停掉所有会让车动起来的东西。不设 manual：MANUAL 仍然
            # 给跟随放行（见 _apply），人站在面前时车会继续跟过来。
            self.follow_enabled = False
            self.patrol_enabled = False
            self.approach_wanted = False
            self._approach_manual = False
            self.pub_dock.publish(String(data='cancel'))
            self._cancel_nav('命令全停')
            self.reason = '命令：全部停止'
        elif cmd in ('dock', 'go_dock', 'charge'):
            # 手动回充：不改 approach_wanted 的自动语义，先导航到桩前；
            # 没有 Nav2 或关掉了接近模式时，直接交给 dock_manager ——
            # 它已经有对齐/重试/冷却，重复一套没意义。
            self.manual = False
            self._approach_cooldown_until = 0.0
            if self.dock_approach and self.nav_client.server_is_ready():
                self.approach_wanted = True
                self._approach_manual = True
                self.reason = '命令：手动回充（先导航到桩前）'
            else:
                # 没有 Nav2 的时候不能让「手动回充」彻底失效：直接让
                # dock_manager 自己试（它在桩附近就能成，P3 已验证）。
                # dock_approach=false 同理 —— 那一支在 update_approach 里会把
                # approach_wanted 立刻清掉，命令会被**无声吞掉**（reason 还写着
                # 「先导航到桩前」），所以这里也必须走直接停靠。
                self.approach_wanted = False
                self._approach_manual = False
                self.pub_dock.publish(String(data='dock'))
                why = '已关闭接近模式' if not self.dock_approach else 'Nav2 不可用'
                self.reason = f'命令：手动回充（{why}，直接停靠）'
        elif cmd in ('cancel', 'cancel_dock'):
            self.approach_wanted = False
            self._approach_manual = False
            self.pub_dock.publish(String(data='cancel'))
            self._cancel_nav('命令取消')
            self.reason = '命令：取消回充'
        elif cmd == 'undock':
            self.approach_wanted = False
            self._approach_manual = False
            self.pub_dock.publish(String(data='undock'))
            self.reason = '命令：离桩'
        elif cmd == 'status':
            self.reason = '命令：查询状态（见 /mission/state）'
        else:
            self.get_logger().warn(f'未知命令: {cmd}')
            return
        # 立刻跑一拍，命令的反馈不用等下一个周期
        self.tick()

    # ---------- 输入是否还新鲜 ----------

    def _person_ready(self, now):
        """有人且 present 没断流。断流 = 检测器死了，不能当成「有人」跟着走。"""
        return (self.person_present
                and (now - self.person_wall) <= self.person_timeout)

    def _dock_fresh(self, now):
        return self.dock_wall > 0.0 and (now - self.dock_wall) <= self.dock_timeout

    def _alert_fresh(self, now):
        return self.alert_wall > 0.0 and (now - self.alert_wall) <= self.alert_timeout

    def _alert_holds(self, now):
        if not self._alert_fresh(now):
            return False
        return self.alert_state in ALERT_HOLD_STATES

    def _ratio_fresh(self, now):
        return self.ratio is not None and (now - self.ratio_wall) <= 5.0

    # ---------- 决策（纯函数，便于离线测试） ----------

    def decide(self, now):
        """按优先级选模式。不产生任何副作用。"""
        if self.estop:
            return self.ESTOP
        if self._alert_holds(now):
            return self.ALERT

        fresh = self._dock_fresh(now)
        # UNDOCKING 也必须算「回充进行中」：dock_manager 正在用它的 action 把车
        # 开出桩，这时候调度层若判成 PATROL/FOLLOW，就会往同一个 Nav2 发巡游目标。
        # 速度上 dock(prio 30) 压得住，但两个控制器抢方向盘、导航目标还会莫名
        # 成功/失败，状态里全是错的。
        if fresh and self.dock_state in ('ALIGNING', 'DOCKING', 'UNDOCKING'):
            return self.DOCKING
        if fresh and self.dock_state == 'DOCKED':
            return self.DOCKED

        if self.manual:
            return self.MANUAL

        if self.dock_approach and self.approach_wanted:
            return self.APPROACH

        if self.follow_enabled and self._person_ready(now):
            return self.FOLLOW

        if self.patrol_enabled and self.waypoints:
            return self.PATROL

        return self.IDLE

    def update_approach(self, now):
        """低电判据的迟滞。只改 approach_wanted，不决定模式。"""
        if not self.dock_approach:
            self.approach_wanted = False
            self._approach_manual = False
            return

        # 手动回充不受电量迟滞约束：用户说去充电就去充电，哪怕电量是 90%。
        # 它只由「到达桩前 / 导航失败 / 取消」三件事结束。
        if self._approach_manual:
            return

        if not self._ratio_fresh(now):
            return

        ratio = self.ratio
        if self.approach_wanted:
            if ratio >= self.clear_ratio:
                self.approach_wanted = False
                self._low_since = None
                self.get_logger().info(
                    f'电量已回到 {ratio:.0%}，解除回桩接近')
            return

        if ratio > self.approach_ratio:
            self._low_since = None
            return

        if not self._dock_fresh(now) or self.dock_state != 'IDLE':
            return                       # 回充已经在跑了，别插一脚
        if now < self._approach_cooldown_until:
            return

        if self._low_since is None:
            self._low_since = now
            self.get_logger().info(
                f'电量 {ratio:.0%} <= {self.approach_ratio:.0%}，确认中…')
            return

        if (now - self._low_since) >= self.approach_confirm:
            self._low_since = None
            self.approach_wanted = True
            self.get_logger().warn(
                f'电量持续 {self.approach_confirm:.0f}s 低于 {self.approach_ratio:.0%}'
                f'（当前 {ratio:.0%}），导航回桩')

    # ---------- 主循环 ----------

    def tick(self):
        now = time.monotonic()
        self.update_approach(now)
        mode = self.decide(now)

        if mode != self.mode:
            if self.mode:
                self.get_logger().info(f'模式 {self.mode} -> {mode}')
            self.mode = mode
            self.mode_since = now

        self._apply(mode, now)
        self._publish_state(now)

    def _set_reason(self, text):
        if text != self.reason:
            self.reason = text

    def _apply(self, mode, now):
        hold = mode in (self.ESTOP, self.ALERT)
        # 跟随放行：
        #   FOLLOW        —— 唯一「要它动」的模式
        #   MANUAL        —— 人工接管，跟随器按自己的 enabled 参数决定，
        #                    这里放行，避免接管后连跟随都用不了
        follow_ok = mode in (self.FOLLOW, self.MANUAL)

        if mode == self.ESTOP:
            self._set_reason('急停闭锁')
        elif mode == self.ALERT:
            self._set_reason(f'告警 {self.alert_state}，按住轮子')
        elif mode == self.DOCKING:
            self._set_reason(f'回充中（{self.dock_state}）')
        elif mode == self.DOCKED:
            self._set_reason('在桩上充电')
        elif mode == self.APPROACH:
            self._set_reason('低电，导航到桩前')
        elif mode == self.FOLLOW:
            self._set_reason('检测到人')
        elif mode == self.PATROL:
            self._set_reason(f'巡游 {self._wp_index + 1}/{len(self.waypoints)}')
        elif mode == self.MANUAL:
            self._set_reason('人工接管')
        elif mode == self.IDLE:
            # 不写这一支的话，IDLE 会**沿用上一轮的 reason**：车明明停着待命，
            # /mission/state 里却写着「检测到人」。看状态的人（和手机端）会被
            # 这个自相矛盾的快照带偏，排查时白走一圈。
            self._set_reason('待命（无事可做）')

        self._send_follow(follow_ok, now)
        self._send_hold(hold, now)

        if mode in (self.APPROACH, self.PATROL):
            want = 'approach' if mode == self.APPROACH else 'patrol'
            # 只在「跑着的是**另一个**目标」时取消。无条件取消会把
            # 当前这一拍的导航目标自己干掉，表现为机器人刚起步就停。
            if self._nav_tag is not None and self._nav_tag != want:
                self._cancel_nav(f'导航目标切换 {self._nav_tag} -> {want}')
            if mode == self.APPROACH:
                self._ensure_nav_approach(now)
            else:
                self._ensure_nav_patrol(now)
            return

        # 其它模式一律不持有导航目标。ALERT/ESTOP 尤其要取消 ——
        # 人可能正躺在这条路径上。
        self._cancel_nav(f'模式 {mode} 不需要导航')

    # ---------- 输出 ----------

    def _send_follow(self, want, now):
        # 边沿触发 + 1Hz 心跳。心跳是给跟随器判「调度层还在不在」用的，
        # 没有它，本节点一崩，跟随器就一直停在 disabled 上。
        changed = (self._follow_sent is None) or (want != self._follow_sent)
        if changed or (now - self._follow_beat) >= 1.0:
            self._follow_sent = want
            self._follow_beat = now
            self.pub_follow.publish(Bool(data=bool(want)))

    def _send_hold(self, want, now):
        changed = (self._hold_sent is None) or (want != self._hold_sent)
        if changed or (now - self._hold_beat) >= 1.0:
            self._hold_sent = want
            self._hold_beat = now
            self.pub_hold.publish(Bool(data=bool(want)))

    def staging_pose(self):
        """桩前 staging 点：沿桩的朝向退 staging_distance，然后掉头对着桩。"""
        x = self.dock_x + self.staging_distance * math.cos(self.dock_yaw)
        y = self.dock_y + self.staging_distance * math.sin(self.dock_yaw)
        yaw = math.atan2(self.dock_y - y, self.dock_x - x)
        return x, y, yaw

    # ---------- 导航 ----------

    def _nav_busy(self):
        return self._nav_state in ('sending', 'active')

    def _ensure_nav_approach(self, now):
        if self._nav_busy():
            if self._nav_tag == 'approach' and \
                    (now - self._nav_sent_at) > self.approach_timeout:
                self.get_logger().error(
                    f'回桩导航超过 {self.approach_timeout:.0f}s 未完成，放弃并冷却')
                self._cancel_nav('超时')
                self._nav_fail += 1
                self.approach_wanted = False
                self._approach_manual = False
                self._approach_cooldown_until = now + self.approach_cooldown
            return
        if now < self._nav_cooldown_until:
            return
        x, y, yaw = self.staging_pose()
        self._send_nav(x, y, yaw, 'approach', now)

    def _ensure_nav_patrol(self, now):
        if now < self._patrol_wait_until:
            return
        if self._nav_busy():
            return
        if self._wp_index >= len(self.waypoints):
            self._wp_index = 0
        x, y, yaw = self.waypoints[self._wp_index]
        self._send_nav(x, y, yaw, 'patrol', now)

    def _send_nav(self, x, y, yaw, tag, now):
        if not self.nav_client.wait_for_server(timeout_sec=0.1):
            if not self._nav_warned_unavailable:
                self._nav_warned_unavailable = True
                self.get_logger().error(
                    f'{self.nav_action} 不可用（Nav2 没起来？）。'
                    f'回充接近与巡游都不会生效，跟随不受影响')
            self._nav_cooldown_until = now + 5.0
            return
        self._nav_warned_unavailable = False

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.position.z = 0.0
        qx, qy, qz, qw = make_quaternion(yaw)
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        self._nav_epoch += 1
        epoch = self._nav_epoch
        self._nav_state = 'sending'
        self._nav_tag = tag
        self._nav_sent_at = now
        self.get_logger().info(
            f'导航目标 [{tag}] {self.map_frame}:({x:.2f},{y:.2f}) yaw={yaw:+.2f}')

        fut = self.nav_client.send_goal_async(goal)
        fut.add_done_callback(lambda f: self._on_nav_accepted(f, epoch, tag))

    def _on_nav_accepted(self, future, epoch, tag):
        try:
            handle = future.result()
        except Exception as e:            # 服务端在发送中途消失
            if epoch == self._nav_epoch:
                self._nav_failed(epoch, f'发送异常：{e}')
            return
        if epoch != self._nav_epoch:
            # goal 在「发出去」到「响应回来」之间被 _cancel_nav 取消了，
            # 那时还没有 handle 可 cancel。但 Nav2 已经接受并在执行这个目标：
            # 不显式取消，它会继续发 /cmd_vel_smoothed，车朝旧目标开走。
            # dock_manager 对同样的竞态就是这么处理的。
            if handle is not None and handle.accepted:
                self.get_logger().info(
                    f'过期的导航目标 [{tag}] 已被接受，立即取消')
                try:
                    handle.cancel_goal_async()
                except Exception:
                    pass
            return
        if handle is None or not handle.accepted:
            self._nav_failed(epoch, 'goal 被拒绝')
            return
        self._nav_state = 'active'
        self._nav_handle = handle
        handle.get_result_async().add_done_callback(
            lambda f: self._on_nav_result(f, epoch, tag))

    def _on_nav_result(self, future, epoch, tag):
        if epoch != self._nav_epoch:
            return                        # 已被取消/取代的目标，迟到的结果
        try:
            status = future.result().status
        except Exception as e:
            self._nav_failed(epoch, f'结果异常：{e}', tag)
            return
        self._nav_state = None
        self._nav_handle = None
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._nav_tag = None
            self._on_nav_success(tag)
        else:
            # 这里**必须把 tag 传下去**：先把 _nav_tag 清空再调 _nav_failed，
            # 它读到的就是 None，`if tag == 'approach'` 永远不成立 ——
            # 回充导航失败时既不清 approach_wanted、也不进冷却，
            # 下一拍立刻重发同一个到不了的目标，形成无声的失败循环。
            self._nav_failed(epoch, f'status={status}', tag)

    def _on_nav_success(self, tag):
        now = time.monotonic()
        self._nav_fail = 0
        if tag == 'approach':
            self.get_logger().info('已到桩前 staging，交给 dock_manager 精确停靠')
            self.pub_dock.publish(String(data='dock'))
            # 交给 dock_manager 以后就别再抢：等它进 ALIGNING，
            # 才会自动切到 DOCKING 模式。这里先把「想要回充」清掉，
            # 否则它一失败我们立刻又发一次导航，变成两个状态机打架。
            self.approach_wanted = False
            self._approach_manual = False
            self._approach_cooldown_until = now + self.approach_cooldown
        elif tag == 'patrol':
            self.get_logger().info(
                f'巡游到达 {self._wp_index + 1}/{len(self.waypoints)}，停 '
                f'{self.patrol_dwell:.0f}s')
            self._patrol_wait_until = now + self.patrol_dwell
            self._wp_index = (self._wp_index + 1) % max(1, len(self.waypoints))

    def _nav_failed(self, epoch, why, tag=None):
        if epoch != self._nav_epoch:
            return
        self._nav_state = None
        self._nav_handle = None
        if tag is None:
            tag = self._nav_tag
        self._nav_tag = None
        self._nav_fail += 1
        now = time.monotonic()
        self._nav_cooldown_until = now + max(5.0, self.approach_cooldown / 2.0)
        self.get_logger().error(f'导航失败 [{tag}]：{why}')
        if tag == 'approach':
            self.approach_wanted = False
            self._approach_manual = False
            self._approach_cooldown_until = now + self.approach_cooldown

    def _cancel_nav(self, why):
        if not self._nav_busy():
            return
        handle = self._nav_handle
        # 先把 epoch 推进，让迟到的 result 进不了 _on_nav_result
        self._nav_epoch += 1
        self._nav_state = None
        self._nav_handle = None
        self._nav_tag = None
        self.get_logger().info(f'取消导航目标（{why}）')
        if handle is not None:
            try:
                handle.cancel_goal_async()
            except Exception:
                pass

    # ---------- 状态 ----------

    def _publish_state(self, now):
        payload = {
            'mode': self.mode,
            'reason': self.reason,
            'since': round(self.mode_since, 3),
            'age': round(now - self.mode_since, 1) if self.mode_since else 0.0,
            'follow_enabled': bool(self._follow_sent),
            'hold': bool(self._hold_sent),
            'manual': bool(self.manual),
            'estop': bool(self.estop),
            'person': self._person_ready(now),
            'dock_state': self.dock_state,
            'battery': None if self.ratio is None else round(self.ratio, 3),
            'approach_wanted': bool(self.approach_wanted),
            'patrol': bool(self.patrol_enabled and self.waypoints),
            'nav': self._nav_tag,
        }
        m = String()
        m.data = json.dumps(payload, ensure_ascii=False)
        self.pub_state.publish(m)
        self.pub_mode.publish(String(data=self.mode))


def main(args=None):
    rclpy.init(args=args)
    node = Mission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            # 退出时把油门放掉、把「按住」撤销，别把状态留在别人身上
            node.pub_hold.publish(Bool(data=False))
            node.pub_follow.publish(Bool(data=False))
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
