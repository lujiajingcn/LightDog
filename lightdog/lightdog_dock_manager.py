#!/usr/bin/env python3
"""LightDog 低电自动回充状态机（P3）

## 职责边界

本节点**不产生任何速度指令**，只做三件事：
  1. 看电量，判断"该回去了"
  2. 调 Nav2 `docking_server` 的 `DockRobot` / `UndockRobot` action
  3. 把状态机状态以话题形式暴露出来（给手机端 / 调试用）

速度完全由 docking_server 产生（`/cmd_vel_dock`），经 twist_mux（prio 30）仲裁后
进 collision_monitor —— 和跟随、导航走同一条链路，碰撞保护不会因为回充被绕过。

## 状态机

    IDLE ──低电──> ALIGNING ──对准──> DOCKING ──成功──> DOCKED ──充满──> UNDOCKING ──> IDLE
      ↑               │ 超时                                              │
      └───────────────┴──────────────────────────────────────────────────┘

- `ALIGNING`  视觉伺服把车头对准桩。**这一步为什么必须存在**：`docking_server` 进
              `INITIAL_PERCEPTION` 后只会等检测、不发速度，如果车头偏了 60° 就
              一直等不到 marker，最后返回 904。而对准不能靠「导航到 staging」——
              四驱 skid steer 原地转向时轮子打滑，轮速积分出的转角比真实值高估
              约 35%（实测转 92° 时 odom 报 137°），导航按 odom 判定"朝向已到位"，
              实际相机偏得老远。所以这里用相机 bearing 直接闭环：看不见就慢速扫描，
              看见了就用 P 控制转到竖直角，全程不碰里程计。
- `DOCKING`  发 DockRobot（`navigate_to_staging_pose: false`，位置由本节点负责），
              等 result。此时 docking 的 dock pose 也来自同一路视觉，两者一致。
- `DOCKED`   真机看 `BatteryState.status == CHARGING` 或电量上涨；
              仿真里电池插件不会充电，用 `sim_charging` 按时间累加（见下）
- 充满 → UndockRobot → IDLE，并进入 `full_cooldown` 冷却，避免刚充满又判低电

## 为什么不自己导航到 staging

「从远处开回桩附近」是任务调度层的事（P4），P3 的边界是「已经在桩附近 → 精确
停靠」。所以 `navigate_to_staging` 默认 false；真要开着导航，把参数打开即可，
但要清楚它会受上面说的 odom 打滑影响，得留足 `dock_prestaging_tolerance`。

## 为什么电量判定不用 `percentage`

P1 实测：`charge 4.969/5.0Ah`（99.4%）时 `LinearBatteryPlugin` 报
`percentage = 0.381` —— 这个字段在 Gazebo 里不可信。所以：
  优先 `charge / capacity`，拿不到就用 `voltage` 阈值，`percentage` 只做兜底。

## 仿真里的两个补丁参数（真机必须关）

- `sim_charging: true`   停靠成功后按 `sim_charge_rate` 给电量充电。仿真电池插件
                         只会放电，不打开这个就永远等不到"充满"，状态机后半段
                         无法验证。真机必须设 false，让真实电量说话。
- `sim_battery_topic`    收到 Float32 时用它替代真实电量（默认
                         /dock/sim_battery_ratio）。`sim/p3.sh battery 0.15` 就是
                         往这里发 —— 这样"低电判定"这条分支是真被执行到的，
                         而不是绕过状态机直接调 action。

## 话题 / action

    订阅  /battery_state              sensor_msgs/BatteryState
    订阅  /detected_dock_pose         geometry_msgs/PoseStamped  对准用（base_footprint 系）
    订阅  /dock/marker_visible        std_msgs/Bool
    订阅  /dock/sim_battery_ratio     std_msgs/Float32      （调试注入，可选）
    订阅  /dock/command               std_msgs/String       dock|undock|cancel|auto_on|auto_off
    发布  /dock/state                 std_msgs/String       状态机当前状态
    发布  /dock/inhibit               std_msgs/Bool         true 时跟随节点应让位
                                                           （ALIGNING/DOCKING/DOCKED/UNDOCKING
                                                            + 取消后的 1.5s 抑制期）
    发布  /dock/battery_ratio         std_msgs/Float32      当前用于判定的电量比
    发布  /cmd_vel_dock               geometry_msgs/Twist   仅 ALIGNING 阶段（对准旋转）
    action client  /dock_robot        nav2_msgs/DockRobot
    action client  /undock_robot      nav2_msgs/UndockRobot

## 关于 inhibit 包含 DOCKED

`DOCKED` 必须抑制跟随。充电时人从桩前走过是很平常的事，但跟随节点只要看到
`/person/present` 就会发速度，而它（prio 20）和 docking 已经停止发指令的空档
正好能让车从桩上开走；状态机却仍停在 DOCKED，`sim_charging` 继续按时间积分，
表现为「明明没插上电，电量却越充越多」。

## 关于 cancel

`DockRobot` 的 goal_handle 必须保存下来，`cancel()` 才能真正调用
`cancel_goal_async()`。只把本地 `state` 置为 IDLE 是假取消：docking_server
仍在跑、仍在发 `/cmd_vel_dock`（prio 30 会压过跟随），而 `/dock/inhibit`
已经放开 —— 两个控制器同时抢底盘。
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Bool, Float32, String
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import DockRobot, UndockRobot


# 状态常量（同时也是 /dock/state 的内容，方便直接 grep 日志）
IDLE = 'IDLE'
ALIGNING = 'ALIGNING'
DOCKING = 'DOCKING'
DOCKED = 'DOCKED'
UNDOCKING = 'UNDOCKING'
DISABLED = 'DISABLED'


class DockManager(Node):

    def __init__(self):
        super().__init__('lightdog_dock_manager')

        # 注意：不要声明 use_sim_time —— params 文件里带这个键时 rclpy 会自动
        # 声明它，再 declare 一次直接抛 ParameterAlreadyDeclaredException，
        # 节点起不来。p2 的几个节点都没声明，原因就在这里。

        # ---- 电量判定 ----
        self.declare_parameter('battery_topic', '/battery_state')
        self.declare_parameter('capacity_ah', 5.0)
        self.declare_parameter('low_ratio', 0.25)       # charge/capacity 低于这个值回去
        self.declare_parameter('full_ratio', 0.95)
        self.declare_parameter('low_voltage', 11.90)    # 拿不到 charge 时的兜底
        self.declare_parameter('low_confirm_time', 3.0) # 持续低电多久才动（避免瞬时抖动）
        self.declare_parameter('full_cooldown', 30.0)   # 充满松开后多久内不再判低电

        # ---- 回充 ----
        self.declare_parameter('dock_id', 'home_dock')
        self.declare_parameter('navigate_to_staging', False)
        self.declare_parameter('max_staging_time', 120.0)
        self.declare_parameter('max_retries', 2)
        self.declare_parameter('retry_cooldown', 20.0)
        # 用户手动 cancel 之后的静默期。不设的话，只要电量还低于阈值，
        # 下一秒 tick 就会重新判低电、3s 后再次发起回充 —— 表现为「点了取消，
        # 车掉头又去充电」，取消按钮形同虚设（仿真里实测 3.3s 就重新 DOCKED）。
        self.declare_parameter('cancel_cooldown', 120.0)
        self.declare_parameter('auto_dock', True)

        # ---- 视觉对准（ALIGNING）----
        self.declare_parameter('align_before_dock', True)
        self.declare_parameter('align_tol', 0.10)        # 竖直角容差 (rad) ≈ 5.7°
        self.declare_parameter('align_kp', 1.2)
        self.declare_parameter('align_max_w', 0.45)      # rad/s
        self.declare_parameter('align_scan_rate', 0.35)  # 看不见 marker 时的扫描角速度
        self.declare_parameter('align_timeout', 40.0)    # 对准总超时（墙钟秒）
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_dock')

        # ---- 仿真补丁 ----
        self.declare_parameter('sim_charging', True)
        self.declare_parameter('sim_charge_rate', 1.0)     # Ah/s（仿真时间）
        self.declare_parameter('sim_battery_topic', '/dock/sim_battery_ratio')
        self.declare_parameter('sim_battery_timeout', 5.0)

        # 10Hz：ALIGNING 的视觉伺服要这个频率，2Hz 会来回过冲。
        # 状态机的其它分支都是事件驱动，跑快一点没有副作用。
        self.declare_parameter('tick_hz', 10.0)

        p = self.get_parameter
        self.capacity = float(p('capacity_ah').value)
        self.low_ratio = float(p('low_ratio').value)
        self.full_ratio = float(p('full_ratio').value)
        self.low_voltage = float(p('low_voltage').value)
        self.low_confirm_time = float(p('low_confirm_time').value)
        self.full_cooldown = float(p('full_cooldown').value)

        self.dock_id = str(p('dock_id').value)
        self.navigate_to_staging = bool(p('navigate_to_staging').value)
        self.max_staging_time = float(p('max_staging_time').value)
        self.max_retries = int(p('max_retries').value)
        self.retry_cooldown = float(p('retry_cooldown').value)
        self.cancel_cooldown = float(p('cancel_cooldown').value)
        self.auto_dock = bool(p('auto_dock').value)

        self.align_before_dock = bool(p('align_before_dock').value)
        self.align_tol = float(p('align_tol').value)
        self.align_kp = float(p('align_kp').value)
        self.align_max_w = float(p('align_max_w').value)
        self.align_scan_rate = float(p('align_scan_rate').value)
        self.align_timeout = float(p('align_timeout').value)

        self.sim_charging = bool(p('sim_charging').value)
        self.sim_charge_rate = float(p('sim_charge_rate').value)
        self.sim_battery_timeout = float(p('sim_battery_timeout').value)

        # ---- 运行时状态 ----
        self.state = IDLE
        self.battery = None           # 最近一条 BatteryState
        self.sim_ratio = None         # 调试注入的电量比
        self.sim_ratio_stamp = 0.0
        self.charge_ah = None         # 用于 sim_charging 的积分量
        self.low_since = None         # 连续低电起始时刻
        self.cooldown_until = 0.0     # 充满后的冷却
        self.retry_until = 0.0        # 失败后的冷却
        self.retries = 0
        self.last_error = ''
        self.busy = False             # action 在飞
        self._dock_future = None
        self._undock_future = None
        # 真正取消回充需要 goal_handle，不是 send_goal 返回的 future。
        # 之前这里只存了 future，cancel() 就没法调 cancel_goal_async，
        # 结果是「本地状态回到 IDLE，docking_server 还在跑、还在发 /cmd_vel_dock」。
        self._dock_goal_handle = None
        self._undock_goal_handle = None
        # 取消/失败后的一小段抑制期：docking_server 收到 cancel 到真正停下来
        # 有几个周期的延迟，这期间如果立刻放开 inhibit，跟随会和回充抢方向盘。
        self._inhibit_until = 0.0
        self._last_fb = None
        self._last_tick_ros = None
        # 纪元计数（epoch）。每次发起 / 取消回充都 +1，回调里带上发起时的
        # epoch，对不上就直接丢弃 —— 这样「取消之后迟到的 result」不会被当成
        # 一次真正的停靠失败，也不会污染下一次回充的重试计数。
        self._dock_epoch = 0
        self._undock_epoch = 0

        # 视觉对准用的状态
        self.dock_pose = None          # 最近一次 /detected_dock_pose
        self.marker_visible = False
        self.align_started = 0.0
        self.align_done = False        # 本轮对准是否已完成（防止重复触发）

        sensor_qos = QoSProfile(depth=5,
                                reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST)

        self.create_subscription(BatteryState, str(p('battery_topic').value),
                                 self.on_battery, sensor_qos)
        self.create_subscription(Float32, str(p('sim_battery_topic').value),
                                 self.on_sim_battery, 5)
        self.create_subscription(String, '/dock/command', self.on_command, 5)
        self.create_subscription(PoseStamped, '/detected_dock_pose', self.on_dock_pose, 5)
        self.create_subscription(Bool, '/dock/marker_visible', self.on_marker_visible, 5)

        self.pub_state = self.create_publisher(String, '/dock/state', 10)
        self.pub_inhibit = self.create_publisher(Bool, '/dock/inhibit', 10)
        self.pub_ratio = self.create_publisher(Float32, '/dock/battery_ratio', 10)
        self.pub_vel = self.create_publisher(Twist, str(p('cmd_vel_topic').value), 10)

        self.dock_client = ActionClient(self, DockRobot, '/dock_robot')
        self.undock_client = ActionClient(self, UndockRobot, '/undock_robot')

        rate = max(0.5, float(p('tick_hz').value))
        self.create_timer(1.0 / rate, self.tick)

        self.get_logger().info(
            f'DockManager 就绪：低电 {self.low_ratio:.0%} / {self.low_voltage:.2f}V，'
            f'充满 {self.full_ratio:.0%}，dock_id={self.dock_id}，'
            f'auto_dock={self.auto_dock}，sim_charging={self.sim_charging}')

    # ---------- 回调 ----------

    def on_battery(self, msg: BatteryState):
        self.battery = msg

    def on_sim_battery(self, msg: Float32):
        self.sim_ratio = max(0.0, min(1.0, float(msg.data)))
        self.sim_ratio_stamp = time.monotonic()

    def on_dock_pose(self, msg: PoseStamped):
        self.dock_pose = msg

    def on_marker_visible(self, msg: Bool):
        self.marker_visible = bool(msg.data)

    def on_command(self, msg: String):
        cmd = msg.data.strip().lower()
        self.get_logger().info(f'收到命令: {cmd}')
        if cmd == 'dock':
            self.start_docking(manual=True)
        elif cmd == 'undock':
            self.start_undocking()
        elif cmd == 'cancel':
            self.cancel()
        elif cmd == 'auto_on':
            self.auto_dock = True
        elif cmd == 'auto_off':
            self.auto_dock = False
        else:
            self.get_logger().warn(f'未知命令: {cmd}')

    # ---------- 电量 ----------

    def battery_ratio(self):
        """当前用于判定的电量比（0~1）；完全拿不到电量返回 None。"""
        now = time.monotonic()
        if self.sim_ratio is not None and (now - self.sim_ratio_stamp) <= self.sim_battery_timeout:
            return self.sim_ratio

        # 仿真充电的积分量必须在 battery is None 之前判：gz 的 BatteryState 桥接
        # 要几分钟才出第一帧，而 DOCKED 往往在它之前就发生了。原来先 return None
        # 的话，这段窗口里 charge_ah 再涨也没人读，表现为「停在桩上、电量不动、
        # 也不离桩」。charge_ah 只在 sim_charging 下写入，真机路径不受影响。
        if self.charge_ah is not None:
            return max(0.0, min(1.0, self.charge_ah / self.capacity))

        if self.battery is None:
            return None

        ch, cap = float(self.battery.charge), float(self.battery.capacity)
        if not math.isnan(ch) and not math.isnan(cap) and cap > 0.0:
            return max(0.0, min(1.0, ch / cap))

        v = float(self.battery.voltage)
        if not math.isnan(v) and v > 0.0:
            # 12V 锂电：11.9V~12.6V 线性映射到 0~1（只用于兜底判定）
            return max(0.0, min(1.0, (v - self.low_voltage) / (12.6 - self.low_voltage)))

        pct = float(self.battery.percentage)
        if not math.isnan(pct) and pct > 0.0:
            return max(0.0, min(1.0, pct))
        return None

    # ---------- 状态机 ----------

    def tick(self):
        now = time.monotonic()
        ratio = self.battery_ratio()

        # sim_charging：停靠期间给电量充电（只在 DOCKED 时积分）
        if self.sim_charging and self.state == DOCKED and ratio is not None:
            # 注意这里用 ROS 时钟而不是上面的墙钟 dt：sim_charge_rate 的单位是
            # 「Ah/仿真秒」，用墙钟积分会让充电速度乘上 1/实时因子（本机 0.22x
            # 时快 4.5 倍），P1/P3 记录的充电时长就对不上了。
            ros_now = self.get_clock().now()
            last_ros = getattr(self, '_last_tick_ros', None)
            self._last_tick_ros = ros_now
            if last_ros is None:
                sim_dt = 0.0
            else:
                sim_dt = (ros_now - last_ros).nanoseconds * 1e-9
            sim_dt = max(0.0, min(sim_dt, 5.0))
            base = self.charge_ah if self.charge_ah is not None else ratio * self.capacity
            self.charge_ah = min(self.capacity, base + self.sim_charge_rate * sim_dt)
            ratio = self.charge_ah / self.capacity
        else:
            self._last_tick_ros = None

        if ratio is not None:
            self.pub_ratio.publish(Float32(data=float(ratio)))
        self.pub_state.publish(String(data=self.state))
        # DOCKED 也必须在抑制期里。否则充电时人从旁边走过，跟随节点
        # （prio 20）会把人从桩上开走，而状态机还停在 DOCKED、sim_charging
        # 继续积分 —— 表现为「没插着电，电量却越充越多」。
        inhibit = (self.state in (ALIGNING, DOCKING, DOCKED, UNDOCKING)
                   or now < self._inhibit_until)
        self.pub_inhibit.publish(Bool(data=inhibit))

        if self.state == ALIGNING:
            self._tick_aligning(now)
            return

        if self.state in (DOCKING, UNDOCKING):
            return

        if self.state == DOCKED:
            if ratio is None:
                return
            if ratio >= self.full_ratio:
                self.get_logger().info(f'电量已满（{ratio:.1%}），开始离桩')
                self.charge_ah = None
                self.cooldown_until = now + self.full_cooldown
                self.start_undocking()
            return

        if self.state != IDLE:
            return

        if not self.auto_dock:
            return
        if now < self.cooldown_until or now < self.retry_until:
            return
        if ratio is None:
            return

        low = ratio < self.low_ratio
        if not low and self.battery is not None:
            v = float(self.battery.voltage)
            low = (not math.isnan(v)) and v > 0.0 and v < self.low_voltage

        if not low:
            self.low_since = None
            return

        if self.low_since is None:
            self.low_since = now
            self.get_logger().info(f'电量 {ratio:.1%} 低于阈值，确认中…')
            return

        if (now - self.low_since) >= self.low_confirm_time:
            self.low_since = None
            self.start_docking(manual=False)

    # ---------- 视觉对准 ----------

    def _tick_aligning(self, now):
        """把车头转到正对桩。看不见就慢速扫描，看见了就 P 控制。"""
        if now - self.align_started > self.align_timeout:
            self.get_logger().error(
                f'对准超时（{self.align_timeout:.0f}s 内没把 marker 摆到中间），放弃本次回充')
            self.pub_vel.publish(Twist())
            self.busy = False
            self.state = IDLE
            self.retry_until = now + self.retry_cooldown
            self._inhibit_until = now + 1.0
            return

        tw = Twist()
        if self.marker_visible and self.dock_pose is not None:
            # dock pose 的 position 就是「停靠位在机器人坐标系下的位置」，
            # 它的方位角就是车头该转的角度
            bearing = math.atan2(float(self.dock_pose.pose.position.y),
                                 float(self.dock_pose.pose.position.x))
            if abs(bearing) <= self.align_tol:
                self.pub_vel.publish(Twist())     # 先停住再交给 docking_server
                self.get_logger().info(
                    f'对准完成（偏角 {math.degrees(bearing):+.1f}°），'
                    f'开始停靠（距桩约 {float(self.dock_pose.pose.position.x):.2f}m）')
                self.send_dock_goal()
                return
            tw.angular.z = max(-self.align_max_w, min(self.align_max_w, self.align_kp * bearing))
            if now - getattr(self, '_last_align_log', 0.0) > 1.0:
                self._last_align_log = now
                self.get_logger().info(
                    f'对准中：偏角 {math.degrees(bearing):+.1f}°，w={tw.angular.z:+.2f}')
        else:
            # 看不见就朝一个方向慢扫一圈（0.35rad/s，约 18s 一圈）
            tw.angular.z = self.align_scan_rate
            if now - getattr(self, '_last_align_log', 0.0) > 3.0:
                self._last_align_log = now
                self.get_logger().info('还没看到 marker，慢速扫描中…')
        self.pub_vel.publish(tw)

    # ---------- action ----------

    def start_docking(self, manual=False):
        if self.busy:
            self.get_logger().warn('已有回充流程在跑，忽略')
            return

        self.busy = True
        self.last_error = ''
        self.align_started = time.monotonic()
        self._last_align_log = 0.0

        if self.align_before_dock:
            self.state = ALIGNING
            self.get_logger().info(
                f'发起回充{"（手动）" if manual else "（低电）"}：先做视觉对准')
            self._tick_aligning(self.align_started)
            return

        self.state = DOCKING
        self.send_dock_goal()

    def send_dock_goal(self):
        """真正把 DockRobot goal 发给 docking_server（对准之后调）。"""
        if not self.dock_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('/dock_robot action 不可用（docking_server 没起来？）')
            self.busy = False
            self.state = IDLE
            return

        self.state = DOCKING
        goal = DockRobot.Goal()
        goal.use_dock_id = True
        goal.dock_id = self.dock_id
        goal.navigate_to_staging_pose = self.navigate_to_staging
        goal.max_staging_time = self.max_staging_time

        self.get_logger().info(
            f'发送 DockRobot：dock_id={self.dock_id} '
            f'navigate_to_staging={self.navigate_to_staging}')
        self._dock_future = self.dock_client.send_goal_async(goal, self.on_dock_feedback)
        self._dock_epoch += 1
        epoch = self._dock_epoch
        self._dock_future.add_done_callback(
            lambda f: self.on_dock_goal_response(f, epoch))

    def on_dock_feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        names = {0: 'NONE', 1: 'NAV_TO_STAGING', 2: 'INITIAL_PERCEPTION',
                 3: 'CONTROLLING', 4: 'WAIT_FOR_CHARGE', 5: 'RETRY'}
        name = names.get(fb.state, str(fb.state))
        if name != getattr(self, '_last_fb', None):
            self._last_fb = name
            self.get_logger().info(f'回充进度: {name}')

    def on_dock_goal_response(self, future, epoch):
        goal_handle = future.result()
        if epoch != self._dock_epoch:
            # 这次 goal 属于已经被取消/被新一轮取代的流程
            if goal_handle is not None and goal_handle.accepted:
                self.get_logger().info('过期 dock goal 已被接受，立即取消')
                goal_handle.cancel_goal_async()
            return
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('回充 goal 被拒绝')
            self.fail_docking('goal rejected')
            return
        self._dock_goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(
            lambda f: self.on_dock_result(f, epoch))

    def on_dock_result(self, future, epoch):
        if epoch != self._dock_epoch:
            self.get_logger().info('已取消的回充 goal 返回了结果，忽略（不记失败）')
            return
        self._dock_goal_handle = None
        result = future.result().result
        if result.success:
            self.get_logger().info(
                f'停靠成功（重试 {result.num_retries} 次），进入 DOCKED')
            self.state = DOCKED
            self.retries = 0
            self.busy = False
            # 初始化充电积分量：仿真里真实电池不会自己涨
            if self.sim_charging and self.charge_ah is None:
                r = self.battery_ratio()
                self.charge_ah = (r if r is not None else 0.0) * self.capacity
        else:
            self.fail_docking(f'error_code={result.error_code} {result.error_msg}')

    def fail_docking(self, reason):
        self.last_error = reason
        self._dock_goal_handle = None
        self.pub_vel.publish(Twist())
        self.busy = False
        self.retries += 1
        self.get_logger().error(f'停靠失败（第 {self.retries} 次）：{reason}')
        if self.retries > self.max_retries:
            self.get_logger().error('重试耗尽，放弃自动回充，冷却后再说')
            self.retries = 0
        self.retry_until = time.monotonic() + self.retry_cooldown
        self._inhibit_until = time.monotonic() + 1.0
        self.state = IDLE

    def start_undocking(self):
        if self.busy:
            return
        if not self.undock_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('/undock_robot action 不可用')
            self.state = IDLE
            return
        goal = UndockRobot.Goal()
        goal.dock_type = 'simple_charging_dock'
        goal.max_undocking_time = 30.0

        self.busy = True
        self.state = UNDOCKING
        self.get_logger().info('发起离桩')
        self._undock_epoch += 1
        epoch = self._undock_epoch
        self._undock_future = self.undock_client.send_goal_async(goal)
        self._undock_future.add_done_callback(
            lambda f: self.on_undock_goal_response(f, epoch))

    def on_undock_goal_response(self, future, epoch):
        goal_handle = future.result()
        if epoch != self._undock_epoch:
            if goal_handle is not None and goal_handle.accepted:
                self.get_logger().info('过期离桩 goal 已被接受，立即取消')
                goal_handle.cancel_goal_async()
            return
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('离桩 goal 被拒绝')
            self.busy = False
            self.state = IDLE
            return
        self._undock_goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(
            lambda f: self.on_undock_result(f, epoch))

    def on_undock_result(self, future, epoch):
        if epoch != self._undock_epoch:
            self.get_logger().info('已取消的离桩 goal 返回了结果，忽略')
            return
        self._undock_goal_handle = None
        result = future.result().result
        self.busy = False
        self.state = IDLE
        self._inhibit_until = time.monotonic() + 1.0
        if result.success:
            self.get_logger().info('离桩成功，回到 IDLE')
        else:
            self.get_logger().error(f'离桩失败：{result.error_code} {result.error_msg}')

    def cancel(self):
        """取消当前回充/离桩流程。

        关键是**真的把 goal 取消掉**：只把本地状态置 IDLE 是假取消，
        docking_server 会继续跑并继续发 /cmd_vel_dock（prio 30，会压过跟随），
        而 /dock/inhibit 已经放开 —— 车会一边被回充控制器开、一边被跟随抢。
        """
        now = time.monotonic()
        # 先推进 epoch，让所有在飞的 goal 回调失效；再尽力取消 goal handle。
        # 顺序很重要：如果先 cancel 再推 epoch，取消窗口里刚被接受的 goal
        # 会把 handle 写回来，接着它的 result 又会被当成本轮的结果。
        self._dock_epoch += 1
        self._undock_epoch += 1

        cancelled = []
        for tag, handle in (('dock', self._dock_goal_handle),
                            ('undock', self._undock_goal_handle)):
            if handle is None:
                continue
            try:
                fut = handle.cancel_goal_async()
                fut.add_done_callback(
                    lambda f, t=tag: self._log_cancel(t, f))
                cancelled.append(tag)
            except Exception as e:
                self.get_logger().warn(f'取消 {tag} goal 失败：{e}')

        self._dock_goal_handle = None
        self._undock_goal_handle = None
        self.pub_vel.publish(Twist())    # 对准阶段可能有残留角速度
        self._inhibit_until = now + 1.5  # 等 docking_server 真正停下来
        # 清掉仿真充电积分量：不清的话 battery_ratio() 会一直返回这个冻结值，
        # 取消后再也不会更新，低电判定就永远基于取消那一刻的数字。
        self.charge_ah = None
        self.busy = False
        self.state = IDLE
        self.align_done = False
        # 手动取消要真的「停下来别动」。低电判定还在，不做冷却的话
        # tick 会立刻重新走一遍 low → 确认 → 对准 → 停靠（实测 3.3s 就回到 DOCKED）。
        self.low_since = None
        self.cooldown_until = now + self.cancel_cooldown
        if cancelled:
            self.get_logger().info(
                f'取消回充（已请求取消 {"/".join(cancelled)} goal），'
                f'{self.cancel_cooldown:.0f}s 内不再自动回充')
        else:
            self.get_logger().info(
                f'取消回充（当前没有在飞的 goal），'
                f'{self.cancel_cooldown:.0f}s 内不再自动回充')

    def _log_cancel(self, tag, future):
        try:
            self.get_logger().info(f'{tag} goal 取消结果：{future.result()}')
        except Exception as e:
            self.get_logger().warn(f'{tag} goal 取消回调异常：{e}')


def main():
    rclpy.init()
    node = DockManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
