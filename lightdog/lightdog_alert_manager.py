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

真机上要真的拨号，得接 SIP（Asterisk/FreeSWITCH）或者运营商语音 API。
仿真里两者都没有，所以：

- `call_command` 参数是一个命令模板，含 `{phone}` / `{name}` / `{attempt}`。
  默认空 = **只记录不真拨**。真机把它配成
  `asterisk -rx "channel originate SIP/{phone} extension s@alarm"` 之类即可。
- 无论有没有真拨，都会写 `/alert/state`、记日志、并让手机端弹窗 ——
  手机 App 推送才是这个产品最现实的通知通道，电话是兜底。

这条边界写在代码里，也写在 docs/P5_fall_alert.md 里，免得以后误以为已经能打电话。
"""

import json
import os
import shlex
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, String


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

        # ---- 二级：交互无响应 ----
        self.declare_parameter('confirm_window', 15.0)   # 问询后再等多少秒
        self.declare_parameter('recover_hold', 3.0)      # 站起来持续多久算恢复

        # ---- 告警 ----
        self.declare_parameter('resolved_hold', 5.0)     # RESOLVED 显示多久
        self.declare_parameter('contacts_path', '/data/contacts.json')
        self.declare_parameter('call_command', '')       # 空 = 只记录不真拨
        self.declare_parameter('call_interval', 20.0)    # 两次呼叫间隔
        self.declare_parameter('max_rounds', 3)          # 联系人轮询几轮后放弃
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
        self.confirm_window = float(p('confirm_window').value)
        self.recover_hold = float(p('recover_hold').value)
        self.resolved_hold = float(p('resolved_hold').value)
        self.contacts_path = str(p('contacts_path').value)
        self.call_command = str(p('call_command').value).strip()
        self.call_interval = float(p('call_interval').value)
        self.max_rounds = int(p('max_rounds').value)
        self.event_pre = float(p('event_pre').value)
        self.event_post = float(p('event_post').value)
        self.auto_reset = bool(p('auto_reset').value)

        self.state = self.NORMAL
        self.z_p95 = None
        self.last_lying = None        # 最近一次看到 lying 的 ROS 时间
        self.last_standing = None
        self.standing_since = None    # 本轮"连续站立"的起点（恢复判定用）
        self.since = self._now()      # 进入当前状态的时刻
        self.alarm_id = ''
        self.alarm_wall = 0.0
        self.is_test = False          # 手动 test 触发的告警（不拨号、不自动解除）
        self.call_idx = 0
        self.rounds = 0
        self.call_attempts = []
        self.last_call = None

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
            f'呼叫={"已配置命令" if self.call_command else "仅记录（未配置 call_command）"}')

    # ---------- 工具 ----------

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _age(self) -> float:
        return self._now() - self.since

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
        # unknown / absent：两边都不更新，靠 lying_gap 自然过期

    def _on_z(self, msg: Float32):
        self.z_p95 = float(msg.data)

    def _on_command(self, msg: String):
        cmd = msg.data.strip().lower()
        self.get_logger().info(f'收到命令：{cmd}')
        if cmd == 'ack':
            if self.state in (self.ALARM,):
                self._set_state(self.ACKED, '有人确认已知晓，停止继续呼叫')
                self._say('已收到确认，正在联系家人')
            else:
                self.get_logger().info('当前不是 ALARM，ack 忽略')
        elif cmd == 'reset':
            self._reset('人工复位')
        elif cmd == 'test':
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

        if self.state == self.NORMAL:
            if self._is_lying():
                self._set_state(self.SUSPECTED, '检测到躺倒姿态，开始计时')
            return

        if self.state == self.SUSPECTED:
            if not self._is_lying():
                self._set_state(self.NORMAL, '已起身，判定为弯腰/蹲下等误报')
                return
            if self._age() >= self.suspect_hold:
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

    def _age_since_standing(self) -> float:
        if self.standing_since is None:
            return 0.0
        return self._now() - self.standing_since

    def _reset(self, why: str):
        self._set_state(self.NORMAL, why)
        self.alarm_id = ''
        self.alarm_wall = 0.0
        self.is_test = False
        self.call_idx = 0
        self.rounds = 0
        self.call_attempts = []
        self.last_call = None

    # ---------- 告警 ----------

    def _start_alarm(self, test: bool = False):
        now = self._now()
        self.alarm_wall = time.time()
        self.is_test = bool(test)
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
        try:
            with open(self.contacts_path, 'r', encoding='utf-8') as f:
                data = json.load(f)
        except FileNotFoundError:
            return []
        except Exception as e:
            self.get_logger().warn(f'读 {self.contacts_path} 失败：{e}',
                                   throttle_duration_sec=30.0)
            return []
        if isinstance(data, dict):
            data = data.get('contacts', [])
        out = []
        for c in (data or []):
            if isinstance(c, dict) and str(c.get('phone', '')).strip():
                out.append({'name': str(c.get('name', '紧急联系人')),
                            'phone': str(c.get('phone')).strip()})
        return out

    def _maybe_call(self):
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

        contacts = self._load_contacts()
        if not contacts:
            if self.last_call is None:
                self.last_call = now
                self.get_logger().warn(
                    f'没有紧急联系人（{self.contacts_path} 为空）—— '
                    f'告警已发出，但没有可呼叫的号码。手机端 /api/v1/contacts 可添加。')
            return

        c = contacts[self.call_idx % len(contacts)]
        self.call_idx += 1
        if self.call_idx % len(contacts) == 0:
            self.rounds += 1
        self.last_call = now

        attempt = {
            'name': c['name'], 'phone': c['phone'],
            'at_wall': round(time.time(), 3),
            'round': self.rounds + 1,
            'dialed': False,
        }

        if self.call_command:
            cmd = self.call_command.format(
                phone=c['phone'], name=c['name'],
                attempt=len(self.call_attempts) + 1)
            try:
                # 用 shlex 切分而不是 shell=True：号码/姓名是外部输入，
                # 走 shell 等于把用户的地址簿内容当命令执行。
                r = subprocess.run(shlex.split(cmd), capture_output=True, timeout=15.0)
                attempt['dialed'] = (r.returncode == 0)
                if r.returncode != 0:
                    self.get_logger().warn(
                        f'呼叫命令失败 rc={r.returncode}: '
                        f'{r.stderr.decode(errors="replace")[:160]}')
            except Exception as e:
                self.get_logger().warn(f'呼叫命令异常：{e}')
        else:
            self.get_logger().warn(
                f'【仿真】本应拨打 {c["name"]} {c["phone"]}（未配置 call_command，仅记录）')

        self.call_attempts.append(attempt)
        self._say(f'正在联系{c["name"]}。')
        self._publish()

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
            'since': round(self.since, 3),
            'age': round(self._age(), 2),
            'z_p95': None if self.z_p95 is None else round(self.z_p95, 3),
            'is_test': bool(self.is_test),
            'contact_count': len(contacts),
            'call_attempts': self.call_attempts[-8:],
            'rounds': self.rounds,
            'max_rounds': self.max_rounds,
            'call_enabled': bool(self.call_command),
        }
        m = String()
        m.data = json.dumps(payload, ensure_ascii=False)
        self.pub_state.publish(m)


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
