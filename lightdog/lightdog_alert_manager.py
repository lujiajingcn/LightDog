#!/usr/bin/env python3
"""
LightDog P5：跌倒告警状态机（两级确认）

    订阅 /fall/state      std_msgs/String   standing | lying | absent | unknown
    订阅 /fall/z_p95      std_msgs/Float32
    订阅 /alert/command   std_msgs/String   ack | reset | test
    发布 /alert/state     std_msgs/String   JSON，手机端直接显示
    发布 /alert/voice     std_msgs/String   要机器人说的话（真机接 TTS）
    发布 /recorder/event  std_msgs/String   让 P4 切一段带「事发前」的视频

## 为什么必须两级确认

「检测到人躺在地上」和「人摔倒了需要叫救护车」之间差着十万八千里：
蹲下系鞋带、坐地上看电视、弯腰捡东西、趴着擦地，视觉上全是「很低的一团」。
只靠一级判定就打电话，一周能误报二十次，家人第一时间就会把设备关掉。

所以这里做成两级：

    一级（姿态持续性）：连续躺倒 suspect_hold 秒 —— 排除「弯腰一下又起来」
    二级（交互无响应）：语音问「您还好吗」，再等 confirm_window 秒
                         —— 这段时间里只要站起来就取消；一直躺着才报警

二级的价值在于它引入了**时间**这个维度：真正的跌倒者往往起不来，
而误报者一定会起来。这是纯视觉判据永远拿不到的信息。

## 状态机

    NORMAL ──(躺倒)──> SUSPECTED ──(持续 suspect_hold)──> CONFIRMING
                          │                                   │
                          │(站起)                     ┌───────┴────────┐
                          ↓                           │(站起)          │(超时仍躺倒)
                       NORMAL <───────────────────────┘                ↓
                          ↑                                          ALARM ──(ack)──> ACKED
                          │                                            │                 │
                          └────────(站起持续 recover_hold)── RESOLVED <─┴─────────────────┘

## 「打电话」这件事的边界

选型已定（D2）：**自建 SIP**。Asterisk 跑在本机（或家里的一台常开机器）上，
`lightdog_call.SipDialer` 通过 AMI 让 Asterisk 外呼家属的软电话/手机分机。

三种模式（`call_mode`）：

    sip      走 AMI，能区分「接通 / 无人接听 / 忙 / 拥塞」
    command  外部语音 API 的 CLI 模板，只知道命令是否成功（= 已发出，不知是否接听）
    log      只记录，不真拨（仿真默认）

不管哪种模式，`/alert/state`、日志、手机端弹窗都照常 —— App 推送才是这个产品
最现实的通知通道，电话是兜底。**测试告警永不外呼**（`is_test` 直接返回）：
半夜把女儿的手机打醒一次，这个功能就再也没人敢点了。

三条容易踩的边界：

- 拨号在后台线程里做，`_tick` 只非阻塞地收结果。同步拨号会把 5Hz 状态机
  卡住十几秒，那段时间里人自己站起来了也不会进 RESOLVED。
- `ack` 要**真的挂断**正在响的那一通，不只是「不再拨下一个」。
- `call_command` 仍然保留，作为非 SIP 部署的逃生口；但它不是默认路径。
"""

import json
import os
import sys
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, String

from lightdog_call import SipConfig, make_dialer


class AlertManager(Node):

    NORMAL = 'NORMAL'
    SUSPECTED = 'SUSPECTED'
    CONFIRMING = 'CONFIRMING'
    ALARM = 'ALARM'
    ACKED = 'ACKED'
    RESOLVED = 'RESOLVED'

    def __init__(self):
        super().__init__('lightdog_alert_manager')

        # ---- 一级：姿态持续性 ----
        self.declare_parameter('suspect_hold', 5.0)      # 连续躺倒多少秒算"可疑"
        self.declare_parameter('lying_gap', 1.0)         # 判据抖动容忍（丢帧不算起身）
        # 画面里连续这么久看不到人 -> 判为「人已离开」并回到 NORMAL。
        # 这个参数只在 SUSPECTED 用：unknown（蹲着/被挡住/相机故障）**不算**
        # 人走了，必须靠"明确看到站立"或"明确连续无人"才撤销，否则一次遮挡
        # 就能把已经开始的确认流程抹掉。
        self.declare_parameter('absent_grace', 10.0)

        # ---- 二级：交互无响应 ----
        self.declare_parameter('confirm_window', 15.0)   # 问询后再等多少秒
        self.declare_parameter('recover_hold', 3.0)      # 站起来持续多久算恢复

        # ---- 告警 ----
        self.declare_parameter('resolved_hold', 5.0)     # RESOLVED 显示多久
        self.declare_parameter('contacts_path', '/data/contacts.json')
        # ---- 外呼 ----
        # sip = 自建 SIP（Asterisk AMI）；command = 命令模板；log = 只记录。
        # auto 的解析顺序：有 secret -> sip，有 call_command -> command，否则 log。
        self.declare_parameter('call_mode', 'auto')
        self.declare_parameter('call_command', '')       # 空 = 不启用命令通道
        self.declare_parameter('call_interval', 20.0)    # 两次呼叫间隔
        self.declare_parameter('max_rounds', 3)          # 联系人轮询几轮后放弃
        self.declare_parameter('sip_ami_host', '127.0.0.1')
        self.declare_parameter('sip_ami_port', 5038)
        self.declare_parameter('sip_ami_user', 'lightdog')
        self.declare_parameter('sip_ami_secret', '')
        self.declare_parameter('sip_channel_template', 'PJSIP/{number}')
        self.declare_parameter('sip_context', 'alarm')
        self.declare_parameter('sip_exten', 's')
        self.declare_parameter('sip_ring_timeout', 45.0)  # 响铃多久算没人接
        self.declare_parameter('sip_max_call', 120.0)     # 接通后最多占线多久
        self.declare_parameter('sip_connect_timeout', 5.0)
        # 事件视频窗口。pre 必须盖住「躺倒 suspect_hold + 问询 confirm_window」这段，
        # 否则视频从触发告警那刻往前算 15 秒，起点正好落在人已经躺平之后，
        # 最关键的"怎么倒的"没拍到。25 = 5 + 15 + 5 余量。
        self.declare_parameter('event_pre', 25.0)
        self.declare_parameter('event_post', 10.0)
        self.declare_parameter('auto_reset', True)       # 人站起来后是否自动复位到 NORMAL

        # ---- 话题 ----
        self.declare_parameter('fall_state_topic', '/fall/state')
        self.declare_parameter('fall_z_topic', '/fall/z_p95')
        self.declare_parameter('event_topic', '/recorder/event')
        self.declare_parameter('tick_hz', 5.0)

        p = self.get_parameter
        self.suspect_hold = float(p('suspect_hold').value)
        self.lying_gap = float(p('lying_gap').value)
        self.absent_grace = float(p('absent_grace').value)
        self.confirm_window = float(p('confirm_window').value)
        self.recover_hold = float(p('recover_hold').value)
        self.resolved_hold = float(p('resolved_hold').value)
        self.contacts_path = str(p('contacts_path').value)
        self.call_command = str(p('call_command').value).strip()
        self.call_interval = float(p('call_interval').value)
        self.max_rounds = int(p('max_rounds').value)
        self.sip_secret = str(p('sip_ami_secret').value).strip()
        # 模式解析与拨号器构造放在 logger 可用之后（下面），因为「配了 sip 却
        # 没有 secret」必须留下一条 error，而不是安静地退回仅记录。
        mode = self._resolve_call_mode(p('call_mode').value, self.sip_secret,
                                       bool(self.call_command))
        self.call_mode = mode
        self._dialer = None
        if mode == 'sip':
            if not self.sip_secret:
                self.get_logger().error(
                    'call_mode=sip 但没有配置 sip_ami_secret —— 退化为仅记录。'
                    '（manager.conf 的 secret 与 p5_params.yaml 必须一致）')
                self.call_mode = mode = 'log'
            else:
                self._dialer = make_dialer(
                    'sip',
                    sip=SipConfig(
                        host=str(p('sip_ami_host').value),
                        port=int(p('sip_ami_port').value),
                        username=str(p('sip_ami_user').value),
                        secret=self.sip_secret,
                        channel_template=str(p('sip_channel_template').value),
                        context=str(p('sip_context').value),
                        exten=str(p('sip_exten').value),
                        ring_timeout=float(p('sip_ring_timeout').value),
                        max_call=float(p('sip_max_call').value),
                        connect_timeout=float(p('sip_connect_timeout').value)),
                    log=self.get_logger().info)
        elif mode == 'command':
            if not self.call_command:
                self.get_logger().warn(
                    'call_mode=command 但 call_command 为空 —— 退化为仅记录')
                self.call_mode = mode = 'log'
            else:
                self._dialer = make_dialer('command',
                                           call_command=self.call_command,
                                           log=self.get_logger().info)
        self.event_pre = float(p('event_pre').value)
        self.event_post = float(p('event_post').value)
        self.auto_reset = bool(p('auto_reset').value)

        self.state = self.NORMAL
        self.z_p95 = None
        self.last_lying = None        # 最近一次看到 lying 的 ROS 时间
        self.last_standing = None
        self.last_absent = None
        self.absent_since = None      # 本轮"连续无人"的起点
        self.standing_since = None    # 本轮"连续站立"的起点（恢复判定用）
        # 进入当前状态的时刻。用 None 而不是 0 表示「还没进过任何状态」：
        # 仿真里 use_sim_time 下 /clock 未到时 _now() 就是 0，拿 0 当初始值
        # 会让手机端 NORMAL 一开机就显示「已持续 200+ 秒」（那其实是开机时长）。
        self.since = None
        self.alarm_id = ''
        self.alarm_wall = 0.0
        self.is_test = False          # 手动 test 触发的告警（不拨号、不自动解除）
        self.call_idx = 0
        self.rounds = 0
        self.call_attempts = []
        self.last_call = None
        # 联系人缓存：(mtime_ns, size) -> [{name, phone}]。见 _load_contacts。
        self._contacts_key = object()     # 初始值不可能命中，第一次必读
        self._contacts_cache = []

        self.create_subscription(String, str(p('fall_state_topic').value),
                                 self._on_fall, 10)
        self.create_subscription(Float32, str(p('fall_z_topic').value),
                                 self._on_z, 10)
        self.create_subscription(String, '/alert/command', self._on_command, 10)

        self.pub_state = self.create_publisher(String, '/alert/state', 10)
        self.pub_voice = self.create_publisher(String, '/alert/voice', 10)
        self.pub_event = self.create_publisher(String, str(p('event_topic').value), 10)

        hz = max(1.0, float(p('tick_hz').value))
        self.create_timer(1.0 / hz, self._tick)
        # 1Hz 周期性上报，让手机端看到的 age / 呼叫进度是活的（否则只在状态
        # 切换时推一次，界面上"已持续 3 秒"会一直卡着不动）
        self.create_timer(1.0, self._publish)

        self.get_logger().info(
            f'LightDog alert manager ready: 一级={self.suspect_hold:.0f}s 躺倒 -> '
            f'二级={self.confirm_window:.0f}s 无起身 -> 告警；'
            f'外呼={self.call_mode}'
            + (f'（{self._dialer.describe()}）' if self._dialer else '（仅记录）'))

    # ---------- 工具 ----------

    @staticmethod
    def _resolve_call_mode(mode, secret, has_command):
        """把 `call_mode` 解析成 sip / command / log。

        `auto` 是给「同一份 yaml 同时用于仿真和真机」准备的：真机配了 secret
        就自动走 SIP，仿真里什么都没配就自动只记录，不必维护两份参数文件。
        非法取值退回 log —— 外呼是往外打电话，宁可少打也不要打错。
        """
        mode = str(mode or '').strip().lower()
        if mode in ('', 'auto'):
            if str(secret or '').strip():
                return 'sip'
            if has_command:
                return 'command'
            return 'log'
        if mode in ('off', 'none', 'log', 'command', 'sip'):
            return 'log' if mode in ('off', 'none') else mode
        return 'log'

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _age(self) -> float:
        """在**当前状态**里待了多久。

        `since <= 0` 是「还没进入过任何状态」，不是「1970 年就进来了」。
        仿真里 use_sim_time 打开、/clock 还没到时构造节点，`self.since` 会拿到
        0.0；不特判的话手机端 NORMAL 状态一开机就显示「已持续 232 秒」，
        而那个数字其实是整个仿真跑了多久 —— 看起来像状态机坏了。
        """
        if self.since is None:
            return 0.0
        return max(0.0, self._now() - self.since)

    def _say(self, text: str):
        m = String()
        m.data = text
        self.pub_voice.publish(m)
        self.get_logger().info(f'[语音] {text}')

    def _set_state(self, new: str, why: str = ''):
        if new == self.state:
            return
        self.get_logger().info(
            f'状态 {self.state} -> {new}' + (f'（{why}）' if why else ''))
        self.state = new
        self.since = self._now()
        self._publish()

    # ---------- 回调 ----------

    def _on_fall(self, msg: String):
        st = msg.data.strip().lower()
        now = self._now()
        if st == 'lying':
            self.last_lying = now
            self.last_standing = None
        elif st == 'standing':
            self.last_standing = now
        elif st == 'absent':
            self.last_absent = now
        # unknown：两个姿态都不更新，靠 lying_gap 自然过期
        # （unknown 是「看不清」，不能当成起身 —— 相机坏了的时候尤其如此）

    def _on_z(self, msg: Float32):
        self.z_p95 = float(msg.data)

    def _on_command(self, msg: String):
        cmd = msg.data.strip().lower()
        self.get_logger().info(f'收到命令：{cmd}')
        if cmd == 'ack':
            if self.state in (self.ALARM,):
                self._set_state(self.ACKED, '有人确认已知晓，停止继续呼叫')
                # 「不再拨下一个」和「停止正在响的这一通」是两件事。只做前者的话，
                # 家属手机要一直响到 sip_ring_timeout（默认 45 秒），而界面写着
                # 「已收到确认，不再重复呼叫」。
                self._hangup_call('ack')
                # 语音不能说「正在联系家人」：进 ACKED 之后 _maybe_call 直接返回，
                # 一个电话都不会再打。之前这句和同一行日志（停止继续呼叫）自相矛盾，
                # 家属听到的承诺和系统的行为正好相反。
                self._say('已收到确认，不再重复呼叫')
            else:
                self.get_logger().info('当前不是 ALARM，ack 忽略')
        elif cmd == 'reset':
            self._reset('人工复位')
        elif cmd == 'test':
            # 真实告警进行中**不接受**「模拟告警」——
            # _start_alarm(test=True) 会覆盖 alarm_id / alarm_wall、清空
            # call_attempts，并把 is_test 置真；而 _maybe_call 见到 is_test 一律
            # 直接返回。也就是说一次误点（手机界面或 sim/p5.sh test）就能把正在
            # 进行的真实呼叫**全部静音**，告警记录里的呼叫明细也一起丢。
            if (self.state in (self.CONFIRMING, self.ALARM, self.ACKED)
                    and not self.is_test):
                self.get_logger().warn(
                    f'真实告警进行中（{self.state}），忽略模拟告警请求')
            else:
                self._start_alarm(test=True)
        else:
            self.get_logger().warn(f'未知命令：{cmd}（支持 ack / reset / test）')

    # ---------- 状态机 ----------

    def _is_lying(self) -> bool:
        return (self.last_lying is not None
                and (self._now() - self.last_lying) <= self.lying_gap)

    def _is_standing(self) -> bool:
        return (self.last_standing is not None
                and (self._now() - self.last_standing) <= self.lying_gap)

    def _is_absent(self) -> bool:
        return (self.last_absent is not None
                and (self._now() - self.last_absent) <= self.lying_gap)

    def _tick(self):
        # 维护「本轮连续站立时长」。
        #
        # 不能直接用 now - last_standing：人一直站着时 last_standing 每帧都在刷新，
        # 差值永远是 0，recover_hold 永远到不了（写第一版时就是这个 bug，
        # 表现为人站起来了但告警一直不解除）。必须记一个不变的起点。
        if self._is_standing():
            if self.standing_since is None:
                self.standing_since = self._now()
        else:
            self.standing_since = None

        # 「连续无人」的起点。用它把"人走了"和"看不清"分开：
        # 前者一段宽限时间后复位，后者保持状态不动。
        if self._is_absent():
            if self.absent_since is None:
                self.absent_since = self._now()
        else:
            self.absent_since = None

        if self.state == self.NORMAL:
            if self._is_lying():
                self._set_state(self.SUSPECTED, '检测到躺倒姿态，开始计时')
            return

        if self.state == self.SUSPECTED:
            # 只有「明确看到站立」或「明确连续无人」才撤销。
            # unknown（蹲下、被遮挡、相机断流）保持状态等判据回来 —— 原来写的是
            # 「不是躺着就复位」，一次遮挡或相机故障就能把已经开始的确认流程抹掉。
            if self._is_standing():
                self._set_state(self.NORMAL, '已起身（看到站立），判定为弯腰/蹲下等误报')
                return
            if (self.absent_since is not None
                    and (self._now() - self.absent_since) >= self.absent_grace):
                self._set_state(
                    self.NORMAL,
                    f'连续 {self.absent_grace:.0f}s 画面里没人，判为人已离开')
                return
            if self._is_lying() and self._age() >= self.suspect_hold:
                # 一级确认通过
                self._set_state(self.CONFIRMING, f'连续躺倒 {self.suspect_hold:.0f}s')
                self._say('您还好吗？需要帮忙请回答我。')
            return

        if self.state == self.CONFIRMING:
            if self._is_standing():
                self._set_state(self.NORMAL, '问询后自行起身，取消告警')
                return
            if self._age() >= self.confirm_window:
                # 二级确认通过
                self._start_alarm()
            return

        if self.state in (self.ALARM, self.ACKED):
            # 测试告警不跟随姿态自动解除：否则站在人面前点「模拟一次告警」，
            # 弹窗 3 秒后就自己消失，用户根本看不清界面长什么样。
            if self.standing_since is not None and not self.is_test:
                if (self._now() - self.standing_since) >= self.recover_hold:
                    if self.auto_reset:
                        self._set_state(self.RESOLVED, '人已起身，告警解除')
                        # 告警都解除了，电话还响着就是在骚扰：
                        # 「现场已经没事了，但那通电话还在打」是最容易被投诉的
                        # 一类行为，而且家属接起来只会更慌。
                        self._hangup_call('resolved')
                    else:
                        self.get_logger().info(
                            '人已起身（auto_reset=false，保持告警状态）',
                            throttle_duration_sec=30.0)
                return
            self._maybe_call()
            return

        if self.state == self.RESOLVED:
            if self._age() >= self.resolved_hold:
                self._reset('告警解除后回到正常')
            return

    def _reset(self, why: str):
        self._set_state(self.NORMAL, why)
        self._hangup_call('reset')
        self.alarm_id = ''
        self.alarm_wall = 0.0
        self.is_test = False
        self.call_idx = 0
        self.rounds = 0
        self.call_attempts = []
        self.last_call = None

    # ---------- 告警 ----------

    def _start_alarm(self, test: bool = False):
        self.alarm_wall = time.time()
        self.is_test = bool(test)
        # 上一次告警如果还留着一通在途电话（比如连续两次跌倒），先挂掉：
        # 记录已经被清空，再留着它只会在后台拨给上一位联系人。
        self._hangup_call('new_alarm')
        self.alarm_id = 'alm_%s%s' % (
            time.strftime('%Y%m%d_%H%M%S', time.localtime(self.alarm_wall)),
            '_test' if test else '')
        self.call_idx = 0
        self.rounds = 0
        self.call_attempts = []
        self.last_call = None
        self._set_state(self.ALARM,
                        'test 命令手动触发' if test else
                        f'躺倒 {self.suspect_hold:.0f}s 且问询后 {self.confirm_window:.0f}s 无起身')

        label = '手动测试告警' if test else '检测到跌倒'
        # 让 P4 切一段「事发前」的视频：这是紧急联系人最需要看的东西
        req = {'type': 'fall', 'label': label,
               'pre': self.event_pre, 'post': self.event_post,
               'note': 'alert_manager:%s' % self.alarm_id}
        m = String()
        m.data = json.dumps(req, ensure_ascii=False)
        self.pub_event.publish(m)
        self.get_logger().info(f'已请求事件切片：{m.data}')

        self._say('检测到您可能摔倒了，正在联系家人。')
        self._publish()

    def _load_contacts(self):
        """读紧急联系人，按文件 mtime+size 缓存。

        这个函数被 `_publish()` 以 1Hz 调用，而 `_publish` 在 STATE 状态上报里
        也是 1Hz —— 不缓存就是永久每秒读一次 contacts.json（在 SD 卡上是一直
        在磨损的写放大）。文件被手机端改过时 mtime 会变，缓存自然失效。
        """
        try:
            st = os.stat(self.contacts_path)
            key = (st.st_mtime_ns, st.st_size)
        except FileNotFoundError:
            key = None
        except OSError:
            key = None

        if key is not None and key == self._contacts_key:
            return self._contacts_cache
        # 文件不存在时也缓存，但仍然每次都 stat（一次 syscall，不读盘）

        self._contacts_key = key
        if key is None:
            self._contacts_cache = []
            return self._contacts_cache

        try:
            with open(self.contacts_path, 'r', encoding='utf-8') as f:
                data = json.load(f)
        except FileNotFoundError:
            self._contacts_cache = []
            return self._contacts_cache
        except Exception as e:
            self.get_logger().warn(f'读 {self.contacts_path} 失败：{e}',
                                   throttle_duration_sec=30.0)
            self._contacts_cache = []
            return self._contacts_cache
        if isinstance(data, dict):
            data = data.get('contacts', [])
        out = []
        for c in (data or []):
            if isinstance(c, dict) and str(c.get('phone', '')).strip():
                out.append({'name': str(c.get('name', '紧急联系人')),
                            'phone': str(c.get('phone')).strip()})
        self._contacts_cache = out
        return out

    def _maybe_call(self):
        # 先收上一通的结果。这一步必须在 ACKED 判断**之前**：确认之后那一通的
        # 结局（接通了几秒 / 被 ack 挂断）仍然要落进 call_attempts，否则告警记录
        # 里最后一通永远停在「呼叫中」。
        self._drain_calls()

        if self.state == self.ACKED:
            return                       # 已被确认，不再打扰
        if self.is_test:
            # 测试告警**绝不**真的拨号：用户只是想看看弹窗长什么样，
            # 半夜把女儿的手机打醒一次，这个功能就再也没人敢点了。
            if self.last_call is None:
                self.last_call = self._now()
                self.get_logger().info('测试告警：跳过呼叫（不会打扰任何联系人）')
            return
        now = self._now()

        # 第一次进 ALARM 立刻打，之后按 call_interval 间隔
        if self.last_call is not None and (now - self.last_call) < self.call_interval:
            return
        if self.rounds >= self.max_rounds:
            return
        # 上一通还在响铃/通话中，不要同时拨第二个联系人：两个号码一起响，
        # 家属接起来第一句话是「谁啊」，互相打断反而没人听得清。
        if self._dialer is not None and self._dialer.busy:
            return

        contacts = self._load_contacts()
        if not contacts:
            if self.last_call is None:
                self.last_call = now
                self.get_logger().warn(
                    f'没有紧急联系人（{self.contacts_path} 为空）—— '
                    f'告警已发出，但没有可呼叫的号码。手机端 /api/v1/contacts 可添加。')
            return

        # 轮次标签必须在 call_idx 自增**之前**算：
        # round_no = call_idx // len + 1 才是「这是第几轮的第几个联系人」。
        # 原来写的是自增后取 self.rounds + 1，而 rounds 只在「本轮最后一个
        # 联系人打完」时才 +1，于是 2 个联系人时会打成 1、2、2、3 ——
        # 第一轮看起来只联系了一个人，翻看告警记录时很容易误判成漏打。
        round_no = self.call_idx // len(contacts) + 1
        c = contacts[self.call_idx % len(contacts)]
        self.call_idx += 1
        if self.call_idx % len(contacts) == 0:
            self.rounds += 1
        self.last_call = now

        attempt_no = len(self.call_attempts) + 1
        # tag 用来把异步结果配回这一条 attempt。不能用下标：列表会被
        # _start_alarm/_reset 重置，下标会串到上一次告警的记录上。
        tag = f'{self.alarm_id or "noalarm"}#{attempt_no}'
        attempt = {
            'name': c['name'], 'phone': c['phone'],
            'at_wall': round(time.time(), 3),
            'round': round_no,
            'tag': tag,
            'dialed': False,
            'outcome': 'dialing',
            'outcome_text': '呼叫中',
        }
        self.call_attempts.append(attempt)

        if self._dialer is not None:
            if not self._dialer.dial(c['phone'], c['name'],
                                     tag=tag, attempt=attempt_no):
                # dial() 返回 False 只会在「上一通还没结束」时发生，而上面刚查
                # 过 busy —— 走到这里说明是启动/关闭竞态，记一条失败等下一轮。
                self._mark_call(attempt, 'failed', '外呼通道忙，本轮跳过')
        else:
            self.get_logger().warn(
                f'【仿真】本应拨打 {c["name"]} {c["phone"]}'
                f'（call_mode=log，仅记录）')
            self._mark_call(attempt, 'logged', '仅记录（未配置外呼）')

        self._say(f'正在联系{c["name"]}。')
        self._publish()

    def _mark_call(self, attempt, outcome, text):
        attempt['outcome'] = outcome
        attempt['outcome_text'] = text
        attempt['dialed'] = outcome not in ('failed', 'cancelled', 'timeout')

    def _drain_calls(self):
        """非阻塞收外呼结果，回填到对应的 attempt 上。"""
        if self._dialer is None:
            return
        for r in self._dialer.poll():
            hit = None
            for a in reversed(self.call_attempts):
                if a.get('tag') == r.tag:
                    hit = a
                    break
            if hit is None:
                self.get_logger().warn(
                    f'收到一通无主的呼叫结果（{r.tag} {r.name} {r.text()}）')
                continue
            # 保留 attempt 自己的 name/phone/at_wall/round，其余字段用结果覆盖
            hit.update(r.as_dict())
            hit['dialed'] = r.placed
            self.get_logger().info(
                f'呼叫结果：{r.name} {r.number} -> {r.text()}')
            self._publish()

    def _hangup_call(self, why):
        """挂断正在响/正在通话的那一通。没有在途通话时安静返回。"""
        if self._dialer is None:
            return False
        try:
            if not self._dialer.hangup(why):
                return False
        except Exception as e:                      # 兜底：不能因挂断失败改变状态机
            self.get_logger().warn(f'挂断请求异常：{e}')
            return False
        self.get_logger().info(f'已请求挂断当前通话（{why}）')
        return True

    # ---------- 输出 ----------

    def _publish(self):
        # 测试告警不读联系人：它既不会拨号，界面也不该显示"正在联系 2 位联系人"
        contacts = (self._load_contacts()
                    if self.state in (self.ALARM, self.ACKED) and not self.is_test
                    else [])
        payload = {
            'state': self.state,
            'alarm_id': self.alarm_id,
            'alarm_wall': round(self.alarm_wall, 3),
            'since': 0.0 if self.since is None else round(self.since, 3),
            'age': round(self._age(), 2),
            'z_p95': None if self.z_p95 is None else round(self.z_p95, 3),
            'is_test': bool(self.is_test),
            'contact_count': len(contacts),
            'call_attempts': self.call_attempts[-8:],
            'rounds': self.rounds,
            'max_rounds': self.max_rounds,
            'call_enabled': bool(self._dialer is not None and self._dialer.enabled),
            'call_mode': self.call_mode,
            'call_detail': self._dialer.describe() if self._dialer else '仅记录',
        }
        m = String()
        m.data = json.dumps(payload, ensure_ascii=False)
        self.pub_state.publish(m)

    def destroy_node(self):
        # 节点停机（systemd 重启/上车断电前）必须挂断在途电话：
        # 否则手机会一直响到 Asterisk 的超时，而没有任何人知道为什么。
        if self._dialer is not None:
            try:
                self._dialer.close()
            except Exception as e:
                self.get_logger().warn(f'关闭外呼通道异常：{e}')
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AlertManager()
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
