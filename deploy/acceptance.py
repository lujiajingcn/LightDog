#!/usr/bin/env python3
"""LightDog 上车验收 —— 一条命令跑完 docs/firmware_protocol.md §七 的 10 条。

    python3 deploy/acceptance.py                 # 完整验收（#3 等 10 分钟、#9 等 30 分钟）
    python3 deploy/acceptance.py --quick         # 缩短耗时项（只用于自查，不能签字）
    python3 deploy/acceptance.py --only 4,5,6    # 只跑某几条
    python3 deploy/acceptance.py --auto-only     # 不交互：人工项记为「未验」并退非零

## 为什么是 Python 而不是一串 ros2 命令

两个原因，都是踩过的：

1. `ros2 topic echo` 在本项目容器里订阅不稳定（`docs/hardware_readiness.md` §七 已注明），
   而直接写 rclpy 的探针能收到。更要紧的是它**默认把字符串截到 128 字符**
   —— `/mission/state`、`/alert/state` 都是三百多字符的 JSON，截断后 `json.loads`
   必然失败，被 except 一吞就表现为「状态没在发」。
2. 验收要判的是**时序**（500ms 内断 PWM、相邻两次电量差 < 2%），命令行拼接测不了。

## 三条不肯妥协的原则

**一、测不到的事不报 PASS。**
第 2 条（H 桥 EN 脚电平）要万用表，第 1 条的「轮子物理停转」要眼睛。这些一律标
`待人工`，而不是「脚本都过了所以应该没问题」。一张全是绿勾的假报告，比没有报告危险。

**二、自动部分通过 ≠ 整条通过。**
以第 1 条为例：脚本能证明的是「心跳这个**输入**在 t 时刻真的断了」，
证明不了「MCU 真的把 PWM 断了」。报告里把这两件事分开写。

**三、绝不留下现场。**
急停、被 kill 的服务，退出前一律恢复原状（try/finally）。中途 Ctrl-C 也一样，
因为报告是先写盘再退出的，中断不会丢已完成的部分。

## 退出码

    0  全部 PASS（且没有待人工项）
    1  有 FAIL
    2  没有 FAIL，但有「待人工」未确认 —— 清单没走完，不能当通过
"""

import argparse
import json
import math
import os
import shutil
import subprocess
import sys
import time
from datetime import datetime

try:
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                           QoSReliabilityPolicy)
    from sensor_msgs.msg import BatteryState
    from std_msgs.msg import Bool, Float32, String
except ImportError:
    sys.exit('需要 ROS 2 环境：source /opt/ros/<distro>/setup.bash 后再跑')

# 与 lightdog_qos.SAFETY_QOS 逐字一致。QoS 不兼容时两边根本连不上，而且不报错,
# 只表现为「发出去没人收到」。deploy/ 独立于包，所以内联而不是 import。
SAFETY_QOS = QoSProfile(
    depth=1,
    history=QoSHistoryPolicy.KEEP_LAST,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)

G = '\033[32m'
R = '\033[31m'
Y = '\033[33m'
D = '\033[2m'
B = '\033[1m'
E = '\033[0m'

# 分开写而不是 `A, B, C = 'a', 'b', 'c'`：测试用 ast 抽取这些常量来钉住
# 判定逻辑（元组赋值的 target 是 Tuple 节点，按名字取不到）。
PASS = 'PASS'
FAIL = 'FAIL'
SKIP = 'SKIP'
MANUAL = '待人工'


# --------------------------------------------------------------------- 探针


class Probe(Node):
    """一个节点订阅全部验收相关话题，缓存最新值。"""

    def __init__(self):
        super().__init__('lightdog_acceptance')
        self.beat_n = 0
        self.beat_at = None
        self.estop = None
        self.mux_state = None
        self.mux_state_at = None
        self.cmd = None
        self.cmd_at = None
        self.mission = None
        self.mission_at = None
        self.alert = None
        self.led = None
        self.led_at = None
        self.battery = None
        self.battery_hist = []      # (monotonic, percentage)

        self.create_subscription(Bool, '/safety/heartbeat', self._on_beat, 10)
        self.create_subscription(Bool, '/safety/estop', self._on_estop, SAFETY_QOS)
        self.create_subscription(String, '/cmd_vel_mux_state', self._on_mux, 10)
        self.create_subscription(Twist, '/cmd_vel_muxed', self._on_cmd, 10)
        self.create_subscription(String, '/mission/state', self._on_mission, 10)
        self.create_subscription(String, '/alert/state', self._on_alert, 10)
        self.create_subscription(Float32, '/led/brightness', self._on_led, 10)
        self.create_subscription(BatteryState, '/battery_state', self._on_batt, 5)

        self.pub_estop = self.create_publisher(Bool, '/safety/estop', SAFETY_QOS)
        self.pub_voice = self.create_publisher(String, '/voice/command', 10)
        self.pub_light = self.create_publisher(String, '/light/mode', 10)
        self.pub_ratio = self.create_publisher(
            Float32, '/dock/sim_battery_ratio', 10)

    # 回调只做一件事：记下「值」和「到达时刻」。判定逻辑留在检查函数里，
    # 这样回调不会因为一次 json.loads 失败而安静地什么都不记。
    def _on_beat(self, msg):
        self.beat_n += 1
        self.beat_at = time.monotonic()

    def _on_estop(self, msg):
        self.estop = bool(msg.data)

    def _on_mux(self, msg):
        self.mux_state = msg.data
        self.mux_state_at = time.monotonic()

    def _on_cmd(self, msg):
        self.cmd = msg
        self.cmd_at = time.monotonic()

    def _on_mission(self, msg):
        try:
            self.mission = json.loads(msg.data)
            self.mission_at = time.monotonic()
        except (ValueError, TypeError):
            pass

    def _on_alert(self, msg):
        try:
            self.alert = json.loads(msg.data)
        except (ValueError, TypeError):
            pass

    def _on_led(self, msg):
        self.led = float(msg.data)
        self.led_at = time.monotonic()

    def _on_batt(self, msg):
        now = time.monotonic()
        self.battery = msg
        self.battery_hist.append((now, msg.percentage))
        # 30 分钟 1Hz 也就 1800 条，但真机可能更高频；留 20 分钟窗口够算差值
        if len(self.battery_hist) > 20000:
            del self.battery_hist[:5000]

    # ---------- 工具 ----------

    def pump(self, sec):
        """转 sec 秒，处理回调。"""
        end = time.monotonic() + sec
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait(self, pred, timeout, label=''):
        """等 pred() 变真。返回 (是否等到, 实际耗时)。"""
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.02)
            try:
                if pred():
                    return True, time.monotonic() - t0
            except (TypeError, KeyError, AttributeError):
                pass
        return False, time.monotonic() - t0

    def fresh(self, attr, max_age=8.0):
        at = getattr(self, attr + '_at', None)
        return at is not None and (time.monotonic() - at) < max_age

    def set_estop(self, want, settle=0.6):
        """发 → 回读。返回是否真的到位（不信自己发的）。"""
        for _ in range(5):
            self.pub_estop.publish(Bool(data=want))
            self.pump(0.08)
        ok, _ = self.wait(lambda: self.estop is want, settle + 2.0)
        return ok and self.estop is want

    def settle(self, get, timeout=8.0, tol=0.01, quiet=0.6):
        """等一个连续变化的值停下来，返回收敛后的值。

        为什么不能「等它超过某个阈值就行」：light-auto 是渐进 fade
        （fade_time=2.0s），0.5s 时只到 0.51 —— 判据写成 `> 0.5` 能过，
        但那是在赌 fade 速度，换个参数就翻车。这里改成等「连续 quiet 秒
        变化小于 tol」，判据就和 fade 曲线解耦了。
        """
        t0 = time.monotonic()
        last = None
        last_change = t0
        while time.monotonic() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            cur = get()
            if cur is None:
                continue
            now = time.monotonic()
            if last is None or abs(cur - last) > tol:
                last = cur
                last_change = now
            elif now - last_change >= quiet:
                return cur
        return last

    def zero_cmd(self, need=1.0):
        """速度出口是否零速且稳定。"""
        t0 = time.monotonic()
        while time.monotonic() - t0 < need:
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.cmd is None:
                continue
            if abs(self.cmd.linear.x) > 1e-3 or abs(self.cmd.angular.z) > 1e-3:
                return False
        return True


# --------------------------------------------------------------------- 报告


class Report:

    def __init__(self, quick):
        self.rows = []
        self.quick = quick
        self.t0 = time.monotonic()

    def add(self, no, title, status, detail, evidence=''):
        self.rows.append({'no': no, 'title': title, 'status': status,
                          'detail': detail, 'evidence': evidence})
        col = {PASS: G, FAIL: R, SKIP: D, MANUAL: Y}[status]
        print(f'  {col}{status:^6s}{E}  {no:<2}. {title}')
        if detail:
            print(f'          {detail}')
        if evidence:
            for line in evidence.splitlines()[:6]:
                print(f'          {D}{line}{E}')
        sys.stdout.flush()

    def count(self, st):
        return sum(1 for r in self.rows if r['status'] == st)

    def unacked_skips(self, allow_skip):
        return [r for r in self.rows
                if r['status'] == SKIP and r['no'] not in allow_skip]

    def verdict(self, allow_skip):
        """三档，且**跳过也算没走完**。

        一开始把 SKIP 当成「不参与判定」，结果「第 8 条没测到」的报告
        打出了「全部通过，可以上车」—— 这正是整份脚本最该避免的假绿。
        没测到的项必须有人负责：用 --allow-skip <编号> 显式承认。
        """
        if self.count(FAIL):
            return 1
        if self.count(MANUAL) or self.unacked_skips(allow_skip):
            return 2
        return 0

    def markdown(self, ctx, allow_skip=()):
        lines = [
            '# LightDog 上车验收报告', '',
            f'- 时间：{ctx["when"]}',
            f'- 主机：`{ctx["host"]}`  用户：`{ctx["user"]}`',
            f'- 代码：`{ctx["commit"]}`',
            f'- ROS：`{ctx["distro"]}`',
            f'- 模式：{"**快速（时长已缩短，不能作为验收签字依据）**" if self.quick else "完整"}',
            '',
        ]
        n_pass, n_fail = self.count(PASS), self.count(FAIL)
        n_skip, n_man = self.count(SKIP), self.count(MANUAL)
        head = (f'**结论：{n_pass} 通过 / {n_fail} 失败 / '
                f'{n_man} 待人工 / {n_skip} 跳过**')
        lines += [head, '']

        if n_fail:
            lines += ['> 有未通过项，**车不能上车**。逐条看下方证据。', '']
        elif n_man or self.unacked_skips(allow_skip):
            lines += ['> 脚本能测的都过了，但仍有没测到 / 没确认的项。'
                      '**这些没有结论前，清单不算走完。**', '']

        lines += ['| # | 项目 | 结果 | 说明 |', '|---|---|---|---|']
        # 表格按编号排（清单就该按编号读），终端输出才是执行顺序
        for r in sorted(self.rows, key=lambda x: x['no']):
            mark = {PASS: '✅', FAIL: '❌', SKIP: '—', MANUAL: '👤'}[r['status']]
            detail = r['detail'].replace('|', '\\|')
            if r['status'] == SKIP and r['no'] in allow_skip:
                detail = '（已签字跳过）' + detail
            lines.append(f'| {r["no"]} | {r["title"]} | {mark} {r["status"]} | {detail} |')

        ev = [r for r in sorted(self.rows, key=lambda x: x['no']) if r['evidence']]
        if ev:
            lines += ['', '## 证据', '']
            for r in ev:
                lines += [f'### {r["no"]}. {r["title"]}', '```',
                          r['evidence'].rstrip(), '```', '']

        lines += ['', '## 需要人确认的项', '']
        man = [r for r in sorted(self.rows, key=lambda x: x['no'])
               if r['status'] == MANUAL]
        if not man:
            lines += ['（无）']
        for r in man:
            lines.append(f'- **{r["no"]}. {r["title"]}** —— {r["detail"]}')

        lines += ['', '---', '',
                  '清单来源：`docs/firmware_protocol.md` §七。' ,
                  '自动部分由 `deploy/acceptance.py` 生成；'
                  '标 👤 的项脚本测不到，必须人到现场确认后手工打勾。']
        return '\n'.join(lines) + '\n'


def ask_manual(question, auto_only):
    """人工确认。返回 True/False/None（None = 未确认）。"""
    if auto_only:
        return None
    if not sys.stdin.isatty():
        return None
    try:
        ans = input(f'          {Y}?{E} {question} [y/N/s=跳过] ').strip().lower()
    except (EOFError, KeyboardInterrupt):
        print()
        return None
    if ans in ('y', 'yes'):
        return True
    if ans in ('s', 'skip'):
        return None
    return False


# --------------------------------------------------------------------- 检查


def preflight(p, rep, args):
    """第 0 步：节点在不在。不在的话后面全是「跳过」，那没有意义。"""
    print(f'{B}0. 前置：节点在线{E}')
    p.pump(2.0)
    got = {}
    got['heartbeat'] = p.beat_n > 0
    got['mission'] = p.mission is not None
    got['mux_state'] = p.mux_state is not None

    if all(got.values()):
        rep.add(0, '节点在线', PASS,
                f'心跳 {p.beat_n} 条、mission={p.mission.get("mode")}、'
                f'mux 闸={p.mux_state}')
        return True

    miss = [k for k, v in got.items() if not v]
    rep.add(0, '节点在线', FAIL,
            '收不到：' + '、'.join(miss),
            '先启动：sudo systemctl start lightdog.target\n'
            '再确认：sudo ./deploy/verify.sh')
    return False


def check_1(p, rep, args):
    """看门狗真的断 PWM。

    脚本只能证明「心跳这个输入断了」；「轮子物理停转」和「FAULT_LINK_LOST」
    必须人看。分开记，不含糊。
    """
    print(f'\n{B}1. 看门狗真的断 PWM{E}')

    # 安全前置：车必须静止。这一刀会把 mux 杀掉，正在动的车会失去仲裁
    # —— mux 没了速度出口就没人刷新，但万一有别的节点直连 /cmd_vel 就危险。
    if not p.zero_cmd(need=1.5):
        v = p.cmd.linear.x if p.cmd else float('nan')
        rep.add(1, '看门狗真的断 PWM', FAIL,
                f'车没停（cmd_vel.x={v:.3f}），拒绝执行',
                '这一项会 kill 掉 twist_mux。先在安全场地让车停下、'
                '切到 IDLE，再单独跑：--only 1')
        return
    mode = (p.mission or {}).get('mode')
    if mode in ('FOLLOW', 'DOCKING', 'DOCK_APPROACH', 'ALIGNING', 'UNDOCKING'):
        rep.add(1, '看门狗真的断 PWM', SKIP,
                f'当前 mode={mode}，运动中不测（会失去仲裁）',
                '等回到 IDLE / PATROL 再来，或 --only 1 单独执行')
        return

    tgt = ''
    if args.mux_pid:
        tgt = f'pid {args.mux_pid}'
    elif systemd_unit_exists('lightdog-safety-mux'):
        tgt = 'lightdog-safety-mux'
    if not tgt:
        rep.add(1, '看门狗真的断 PWM', SKIP,
                '找不到 twist_mux 的 PID，也没有 systemd 单元',
                '真机：确认 lightdog-safety-mux.service 已装；\n'
                '手动场景：--mux-pid <twist_mux 的 PID>')
        return

    rate_before = p.beat_n
    p.pump(2.0)
    hz = (p.beat_n - rate_before) / 2.0
    if hz < 5:
        rep.add(1, '看门狗真的断 PWM', FAIL,
                f'心跳只有 {hz:.1f} Hz，MCU 看门狗可能已经超时',
                'MCU 侧判据是 300ms 无心跳即断 PWM，心跳频率必须明显高于它')
        return

    n_at_kill = p.beat_n
    killed = kill_mux(tgt)
    if not killed:
        rep.add(1, '看门狗真的断 PWM', FAIL, f'kill 失败：{tgt}')
        return

    # 判据是「kill 之后心跳基本停了」，不是「距上一条超过 0.5s」——
    # 后者在杀掉之前就已经成立（上一条心跳本来就是几百毫秒前的事），
    # 会得到一个必然为真的等待。
    #
    # 也不能要求严格等于 0：实测 kill 瞬间总有 1 条在途样本被回调收下。
    # 20Hz × 1.2s 本该 24 条，容忍 ≤2 条就够了。
    p.pump(1.2)
    new_beats = p.beat_n - n_at_kill
    stopped = new_beats <= 2
    ev = [f'kill 前心跳 {hz:.1f} Hz',
          f'kill 后 1.2s 内新增 {new_beats} 条（正常应约 {int(hz * 1.2)} 条，'
          f'{"已中断" if stopped else "仍在发"}）']

    # 让 systemd 把它拉回来，否则后面的项全废
    back = False
    if systemd_unit_exists('lightdog-safety-mux'):
        r = run(['systemctl', 'start', 'lightdog-safety-mux'], timeout=20)
        back, _ = p.wait(lambda: (time.monotonic() - (p.beat_at or 0)) < 1.0, 25.0)
        ev.append(f'重启服务：{"已恢复心跳" if back else "仍未恢复"}（rc={r}）')
    else:
        ev.append('手动 PID 模式：脚本不会替你重启，请自己拉起 twist_mux')

    auto_ok = stopped and (back or not systemd_unit_exists('lightdog-safety-mux'))
    if not auto_ok:
        rep.add(1, '看门狗真的断 PWM', FAIL,
                '心跳没断' if not stopped else '服务没能恢复',
                '\n'.join(ev))
        return

    seen = ask_manual(
        '轮子物理停转了吗？故障灯报 FAULT_LINK_LOST 了吗？（≤400ms）', args.auto_only)
    detail = '心跳已中断，MCU 的失联输入条件成立'
    if seen is True:
        rep.add(1, '看门狗真的断 PWM', PASS, detail + '；人工确认轮子停转', '\n'.join(ev))
    elif seen is False:
        rep.add(1, '看门狗真的断 PWM', FAIL,
                '心跳断了但轮子没停 —— 看门狗没生效', '\n'.join(ev))
    else:
        rep.add(1, '看门狗真的断 PWM', MANUAL,
                detail + '；但「轮子停转」未确认（这一步脚本测不到）', '\n'.join(ev))


def check_2(p, rep, args):
    """断的是使能不是指令 —— 纯人工，万用表。"""
    print(f'\n{B}2. 断的是使能不是指令{E}')
    seen = ask_manual(
        '用万用表/示波器看过 H 桥 EN 脚了吗？kill twist_mux 时它变低了吗？', args.auto_only)
    if seen is True:
        rep.add(2, '断的是使能不是指令', PASS, '人工确认：EN 变低')
    elif seen is False:
        rep.add(2, '断的是使能不是指令', FAIL, '人工确认：EN 没有变低')
    else:
        rep.add(2, '断的是使能不是指令', MANUAL,
                '需要万用表看 H 桥 EN 脚（脚本测不到硬件电平）',
                '使能拉高让车低速走 -> kill -9 twist_mux -> 看 EN 脚\n'
                '通过标准：EN 变低，而不是「仍在输出 0 占空比」')


def check_3(p, rep, args):
    """待命不误触发：长时间有心跳、无速度，不许进故障。"""
    secs = 20 if args.quick else 600
    print(f'\n{B}3. 待命不误触发（观测 {secs}s）{E}')
    n0 = p.beat_n
    faults = []
    kicks = []
    t_end = time.monotonic() + secs
    while time.monotonic() < t_end:
        rclpy.spin_once(p, timeout_sec=0.1)
        st = p.mux_state or ''
        if any(k in st.lower() for k in ('fault', 'lost', 'error')):
            faults.append(st)
        if p.cmd is not None and p.fresh('cmd', 1.0):
            if abs(p.cmd.linear.x) > 1e-3 or abs(p.cmd.angular.z) > 1e-3:
                kicks.append(f'{time.monotonic() - t_end + secs:.0f}s: '
                             f'v={p.cmd.linear.x:.3f}')
    hz = (p.beat_n - n0) / secs
    ev = [f'心跳 {hz:.1f} Hz（{p.beat_n - n0} 条 / {secs}s）']

    if faults:
        ev += [f'故障状态：{faults[0]}'] 
        rep.add(3, '待命不误触发', FAIL,
                f'待命期间出现故障状态 {faults[0]!r}', '\n'.join(ev))
        return
    if kicks:
        ev += kicks[:3]
        rep.add(3, '待命不误触发', FAIL,
                '待命期间速度出口冒出了非零速度', '\n'.join(ev))
        return
    if hz < 5:
        rep.add(3, '待命不误触发', FAIL, f'心跳只有 {hz:.1f} Hz，会误触发看门狗',
                '\n'.join(ev))
        return
    note = '（--quick 缩短了时长）' if args.quick else ''
    rep.add(3, '待命不误触发', PASS,
            f'{secs}s 内心跳连续、零速、无故障{note}', '\n'.join(ev))


def check_4(p, rep, args):
    """急停闭锁：置位后必须保持，且重复置位幂等。"""
    print(f'\n{B}4. 急停闭锁{E}')
    try:
        p.set_estop(False)
        ok = p.set_estop(True)
        if not ok:
            rep.add(4, '急停闭锁', FAIL, '置位后回读不到 estop=true',
                    f'latched={p.estop} mux={p.mux_state}')
            return
        latched_ok, _ = p.wait(lambda: p.mux_state == 'estop', 3.0)
        # 幂等：再置一次不能把状态翻过来
        p.pub_estop.publish(Bool(data=True))
        p.pump(0.5)
        stay = p.estop is True and p.mux_state == 'estop'
        ev = [f'置位后：latched={p.estop} mux={p.mux_state}',
              f'再发一次 set 后：latched={p.estop} mux={p.mux_state}']
        if not latched_ok:
            rep.add(4, '急停闭锁', FAIL, '锁存为 true 但 mux 闸不是 estop', '\n'.join(ev))
            return
        if not stay:
            rep.add(4, '急停闭锁', FAIL, '重复置位把状态改掉了（不幂等）', '\n'.join(ev))
            return
        seen = ask_manual(
            '按下物理按钮再松开，车仍然不走吗？（松开后必须仍锁）', args.auto_only)
        if seen is False:
            rep.add(4, '急停闭锁', FAIL, '人工确认：松开按钮后能走了', '\n'.join(ev))
            return
        detail = '软件锁存与 mux 闸一致，重复置位幂等'
        if seen is True:
            rep.add(4, '急停闭锁', PASS, detail + '；物理按钮闭锁已确认', '\n'.join(ev))
        else:
            rep.add(4, '急停闭锁', MANUAL,
                    detail + '；物理按钮未确认',
                    '\n'.join(ev) + '\n物理按钮需在固件侧实现（见 firmware_protocol.md §3.2）')
    finally:
        p.set_estop(False)


def check_5(p, rep, args):
    """急停可继承：kill 掉 mission 再拉起，新进程必须直接进 ESTOP。

    这是唯一一条「纯软件、不碰硬件」的安全语义，也是 Restart=always 最容易
    悄悄毁掉的一条 —— 所以它必须自动测，不能靠人看。
    """
    print(f'\n{B}5. 急停可继承{E}')
    use_sd = systemd_unit_exists('lightdog-mission')
    if not use_sd:
        # 这一项必须「杀掉再拉起」才有意义，而没有 systemd 就没有可靠的拉起方式
        # （杀了谁负责重启？）。与其造一个半吊子参数，不如老实跳。
        rep.add(5, '急停可继承', SKIP,
                '没有 lightdog-mission.service —— 这一项需要「杀掉再拉起」',
                '它验的是「重启后还记不记得急停」，手动 kill 之后没人负责拉起，\n'
                '就没法验。真机上先 ./deploy/install.sh 再跑这一条。')
        return
    try:
        if not p.set_estop(True):
            rep.add(5, '急停可继承', FAIL, '前置失败：置不上急停')
            return
        r = run(['systemctl', 'restart', 'lightdog-mission'], timeout=30)
        if r != 0:
            rep.add(5, '急停可继承', FAIL, f'systemctl restart 失败（rc={r}）')
            return
        # 新进程起来要重新发现 DDS、订阅 TRANSIENT_LOCAL 的锁存样本
        ok, dt = p.wait(
            lambda: p.mission is not None and p.mission.get('estop') is True
            and p.mission.get('mode') == 'ESTOP', 25.0)
        ev = [f'重启后 {dt:.1f}s：mission.estop='
              f'{(p.mission or {}).get("estop")} mode={(p.mission or {}).get("mode")}']
        if ok:
            rep.add(5, '急停可继承', PASS,
                    f'mission 重启后 {dt:.1f}s 内自己进 ESTOP（没有从 IDLE 开始猜）',
                    '\n'.join(ev))
        else:
            rep.add(5, '急停可继承', FAIL,
                    'mission 重启后没有继承急停 —— 它可能以为自己能动', '\n'.join(ev))
    finally:
        p.set_estop(False)


def check_6(p, rep, args):
    """语音急停：/voice/command 注入 -> 锁存 + mux 闸，500ms 内。

    注意这只覆盖「意图解析 -> 刹车」这一段。麦克风 -> ASR 文本那一段脚本测不到，
    要人对着麦克风说 —— 报告里分开写，不混为一谈。
    """
    print(f'\n{B}6. 语音急停{E}')
    if not p.fresh('mux_state', 10.0):
        rep.add(6, '语音急停', SKIP, 'mux 不在线')
        return
    try:
        p.set_estop(False)
        p.pump(0.3)
        p.pub_voice.publish(String(data='急停'))
        ok, dt = p.wait(lambda: p.estop is True and p.mux_state == 'estop', 3.0)
        ev = [f'注入 /voice/command="急停" -> {dt * 1000:.0f}ms 后 '
              f'latched={p.estop} mux={p.mux_state}']
        if not ok:
            rep.add(6, '语音急停', FAIL, '注入「急停」后没有闭锁', '\n'.join(ev))
            return
        if dt > 0.5:
            rep.add(6, '语音急停', FAIL,
                    f'{dt * 1000:.0f}ms 才生效，超过 500ms 判据', '\n'.join(ev))
            return
        seen = ask_manual('对着麦克风真说「急停」，500ms 内断电了吗？', args.auto_only)
        path = f'文本注入 -> 刹车 {dt * 1000:.0f}ms'
        if seen is True:
            rep.add(6, '语音急停', PASS, path + '；真实语音链路已确认', '\n'.join(ev))
        elif seen is False:
            rep.add(6, '语音急停', FAIL, '真实语音链路没刹住（ASR 到 /voice/command 之间断了）')
        else:
            rep.add(6, '语音急停', MANUAL,
                    path + '；ASR 麦克风段未确认',
                    '\n'.join(ev) + '\n脚本只验证了解析->刹车；麦克风->ASR 要人说话确认')
    finally:
        p.set_estop(False)


def check_7(p, rep, args):
    """语音解锁：必须真的解锁，不能被理解成再加一次锁。"""
    print(f'\n{B}7. 语音解锁{E}')
    try:
        if not p.set_estop(True):
            rep.add(7, '语音解锁', FAIL, '前置失败：置不上急停')
            return
        p.pump(0.3)
        p.pub_voice.publish(String(data='解除急停'))
        ok, dt = p.wait(lambda: p.estop is False and p.mux_state != 'estop', 3.0)
        ev = [f'注入 /voice/command="解除急停" -> {dt * 1000:.0f}ms 后 '
              f'latched={p.estop} mux={p.mux_state}']
        if p.mux_state == 'hold':
            # 解锁后闸经常停在 hold，容易被误读成「还锁着」。这两个是不同的东西：
            # hold 是调度层自己在按着（它每拍刷新），estop 才是安全闭锁。
            ev.append('mux=hold 是调度层在按着，不是锁；判据只看闸是不是 estop')
        if ok:
            rep.add(7, '语音解锁', PASS, f'文本注入 -> 解锁 {dt * 1000:.0f}ms', '\n'.join(ev))
        else:
            # 这一条最容易出的错是「解锁被当成加锁」，所以把实际值写进报告
            rep.add(7, '语音解锁', FAIL,
                    f'没解开（latched={p.estop} mux={p.mux_state}）', '\n'.join(ev))
    finally:
        p.set_estop(False)


def check_8(p, rep, args):
    """亮度映射：/light/mode 驱动，看 light-auto 有没有真的把值发给固件。

    ## 为什么不去「回读」/led/brightness

    这条最容易写出假 PASS。真机上 `/led/brightness` 的消费者是**固件**，
    ROS 侧没有任何节点回读它。如果脚本自己发一条再自己订阅到，测的只是
    「消息总线通不通」—— 灯亮不亮一点没碰到，却会打一个绿勾。

    所以这里只做两件事：
      1. 用 `/light/mode` off/on 驱动，确认 light-auto 真的发了 0 和正值
         （这是「链路通」的证据，不是「灯亮」的证据）
      2. 中间值与「无台阶」交给照度计，脚本明确记「待人工」

    没有 light-auto 在跑就直接跳——真机上那属于纯人工项，不该假装测过。
    """
    print(f'\n{B}8. 亮度映射{E}')
    if p.count_publishers('/led/brightness') == 0:
        rep.add(8, '亮度映射', SKIP,
                '/led/brightness 没有发布者（light-auto 没起）',
                '真机上 light-auto 默认启用（deploy/nodes.conf default=yes）。\n'
                '没起的话这条只能人工：照度计 + 手动灌值\n'
                '  ros2 topic pub -r 1 /led/brightness std_msgs/msg/Float32 "{data: 0.5}"')
        return

    try:
        p.pub_light.publish(String(data='off'))
        ok_off, dt_off = p.wait(lambda: p.led is not None and p.led < 0.05, 4.0)
        v_off = p.settle(lambda: p.led)

        p.pub_light.publish(String(data='on'))
        p.pump(0.3)
        p.wait(lambda: p.led is not None and p.led > 0.1, 4.0)
        v_on = p.settle(lambda: p.led)
        # on 是渐进的（fade_time=2s），这里记的是收敛值，不是「某个瞬间过了阈值」
        ev = [f'mode=off -> {dt_off:.1f}s 后收敛到 {v_off}',
              f'mode=on  -> 收敛到 {v_on}（fade 完成后）']
        if not ok_off or v_off is None or v_off >= 0.05:
            rep.add(8, '亮度映射', FAIL,
                    f'发 off 后亮度没降到 0（实际 {v_off}）—— light-auto 没跟', '\n'.join(ev))
            return
        # 判据故意不写「必须到 1.0」：on_brightness 是参数（真机 0.85），
        # 自动部分只负责证明「链路通了、off 真灭了、on 真的亮起来」。
        if v_on is None or v_on < 0.2:
            rep.add(8, '亮度映射', FAIL,
                    f'发 on 后亮度只到 {v_on} —— light-auto 没跟', '\n'.join(ev))
            return
        if v_on - v_off < 0.2:
            rep.add(8, '亮度映射', FAIL,
                    f'on/off 差值只有 {v_on - v_off:.2f}，不够分辨', '\n'.join(ev))
            return

        seen = ask_manual(
            '用照度计看过吗？0.0 全灭、1.0 满亮、中间无台阶？', args.auto_only)
        auto_detail = f'链路通：off={v_off:.2f} on={v_on:.2f}（light-auto 真的发了）'
        if seen is False:
            rep.add(8, '亮度映射', FAIL, '人工确认：实际亮度不符合', '\n'.join(ev))
            return
        if seen is True:
            rep.add(8, '亮度映射', PASS, auto_detail + '；照度计已确认', '\n'.join(ev))
        else:
            rep.add(8, '亮度映射', MANUAL,
                    auto_detail + '；但灯实际亮多少没测（固件侧，脚本测不到）',
                    '\n'.join(ev) +
                    '\n中间值与「无台阶」只能用照度计：'
                    'ros2 topic pub -r 1 /led/brightness std_msgs/msg/Float32 "{data: 0.5}"')
    finally:
        p.pub_light.publish(String(data='auto'))


def check_9(p, rep, args):
    """电量不跳变：**按秒重采样**后判相邻差值 < 2%，无 NaN。

    ## 两个必须防的假 PASS

    1. 直接比较「相邻两条」是没有意义的：仿真里 `/battery_state` 跑在 20Hz，
       相邻两条间隔 50ms，差值当然小。判据必须按**时间**对齐（每秒取一个
       样本），否则频率越高越容易通过 —— 和被测的东西正好相反。
    2. `percentage` 越界时不能当正常值算差值。实测仿真里给的是
       `-4.75e-05`（超出 BatteryState 规定的 0~1），这时「不跳变」通过
       只是因为这个数是常数。数据本身不合规要单独报出来。
    """
    secs = 60 if args.quick else 1800
    print(f'\n{B}9. 电量不跳变（观测 {secs}s）{E}')
    t_end = time.monotonic() + secs
    n0 = len(p.battery_hist)
    while time.monotonic() < t_end:
        rclpy.spin_once(p, timeout_sec=0.2)
    hist = p.battery_hist[n0:]
    if len(hist) < 2:
        rep.add(9, '电量不跳变', SKIP,
                f'{secs}s 内只收到 {len(hist)} 条 /battery_state',
                '电池话题没在发。没测到就是没测到，不能算通过')
        return

    # 按秒重采样：每个 1s 窗口取最后一个样本，和真机 1Hz 的口径一致
    buckets = {}
    for t, v in hist:
        buckets[int(t - hist[0][0])] = v
    series = [buckets[k] for k in sorted(buckets)]

    nan = [v for v in series if v is None or (isinstance(v, float) and math.isnan(v))]
    oob = [v for v in series
           if v is not None and not math.isnan(v) and not (0.0 <= v <= 1.0)]
    rate = len(hist) / max(1.0, secs)
    ev = [f'{len(hist)} 条 / {secs}s（{rate:.1f} Hz），重采样为 {len(series)} 个秒样本']

    if nan:
        ev.append(f'NaN {len(nan)} 个，例：{nan[0]}')
        rep.add(9, '电量不跳变', FAIL,
                f'{len(nan)} 秒的 percentage 是 NaN（低电回充永远不会触发）',
                '\n'.join(ev))
        return

    if oob:
        # 越界不是「跳变」，是数据不合规：不能拿来算差值，也不能算过
        ev.append(f'越界值 {len(oob)} 个，例：{oob[0]:.6g}')
        rep.add(9, '电量不跳变', SKIP,
                f'percentage 超出 [0,1]（{len(oob)}/{len(series)} 个秒样本），'
                f'这条测不出真值',
                '\\n'.join(ev) +
                '\n真机上固件必须把 percentage 归一化到 0.0~1.0'
                '（见 docs/firmware_protocol.md §5.1）。\n'
                '（仿真里这是 gz bridge 给的原始值，同样越界。）\n'
                '现在拿到的是越界常数，判「不跳变」会得到一个毫无意义的通过。')
        return

    jumps = [(i, series[i - 1], series[i]) for i in range(1, len(series))
             if abs(series[i] - series[i - 1]) >= 0.02]
    ev.append(f'{series[0]:.3f} -> {series[-1]:.3f}（{len(series)} 个秒样本）')
    if jumps:
        ev += [f'第 {i} 秒：{a:.3f} -> {b:.3f}（Δ={abs(b - a) * 100:.1f}%）'
               for i, a, b in jumps[:5]]
        rep.add(9, '电量不跳变', FAIL,
                f'{len(jumps)} 次秒级跳变 ≥ 2%（硬件或驱动在抖）', '\n'.join(ev))
    else:
        note = '（--quick 缩短了时长）' if args.quick else ''
        rep.add(9, '电量不跳变', PASS,
                f'{len(series)} 个秒样本全部平滑，无 NaN{note}', '\n'.join(ev))


def check_10(p, rep, args):
    """低电回充：注入 30% -> DOCK_APPROACH -> DOCKING。"""
    print(f'\n{B}10. 低电回充{E}')
    subs = p.count_subscribers('/dock/sim_battery_ratio')
    if subs == 0:
        rep.add(10, '低电回充', SKIP,
                '/dock/sim_battery_ratio 没有订阅者 —— 这是仿真专用注入口',
                '真机上没有这个注入口。改用车规可调电源拉低电压，或\n'
                '把真实电量放到 30% 以下再测。这条在真机上属于人工项。')
        return
    timeout = 60 if args.quick else 150
    print(f'  {Y}注意{E}：这一项会让车自己开始移动（低电 -> 接近桩 -> 停靠）。'
          f'确认场地内无障碍、有人在旁边。3 秒后开始…')
    p.pump(3.0)
    try:
        p.pub_ratio.publish(Float32(data=0.30))
        p.pump(0.5)
        ok_a, dt_a = p.wait(
            lambda: (p.mission or {}).get('mode') in
            ('DOCK_APPROACH', 'DOCKING'), timeout)
        ev = [f'注入 30% -> {dt_a:.1f}s 后 mode={(p.mission or {}).get("mode")}']
        if not ok_a:
            rep.add(10, '低电回充', FAIL, f'{timeout}s 内没有进入回充流程', '\n'.join(ev))
            return
        ok_d, dt_d = p.wait(
            lambda: (p.mission or {}).get('mode') == 'DOCKING'
            or (p.mission or {}).get('dock_state') in ('ALIGNING', 'DOCKING'), timeout)
        ev.append(f'再 {dt_d:.1f}s：mode={(p.mission or {}).get("mode")} '
                  f'dock_state={(p.mission or {}).get("dock_state")}')
        if ok_d:
            rep.add(10, '低电回充', PASS,
                    f'低电 -> 接近 {dt_a:.1f}s -> 停靠 {dt_d:.1f}s', '\n'.join(ev))
        else:
            rep.add(10, '低电回充', FAIL,
                    '进了接近但没进停靠（导航到桩前失败？）', '\n'.join(ev))
    finally:
        # 注入口持续 60s 会自己过期；不断发就会一直停在 30%，这里打回一个高值
        p.pub_ratio.publish(Float32(data=1.0))


# --------------------------------------------------------------------- 环境


def run(cmd, timeout=20):
    try:
        return subprocess.run(cmd, stdout=subprocess.DEVNULL,
                              stderr=subprocess.DEVNULL,
                              timeout=timeout).returncode
    except (OSError, subprocess.TimeoutExpired):
        return 255


def systemd_unit_exists(name):
    if not shutil.which('systemctl'):
        return False
    return run(['systemctl', 'cat', f'{name}.service'], timeout=5) == 0


def kill_mux(tgt):
    if tgt.startswith('pid '):
        # 手动模式允许逗号分隔：`ros2 run` 会留下一个 CLI 父进程 + 一个真正的
        # 脚本子进程，只杀一个的话另一个还挂着 —— 表现为「心跳停了但进程在」
        pids = [x for x in tgt[4:].replace(',', ' ').split() if x]
        rc = 0
        for pid in pids:
            rc |= run(['kill', '-9', pid])
        return rc == 0
    return run(['systemctl', 'kill', '-s', 'KILL', tgt], timeout=15) == 0


def sh(cmd):
    try:
        return subprocess.run(cmd, capture_output=True, text=True,
                              timeout=5).stdout.strip()
    except (OSError, subprocess.TimeoutExpired):
        return ''


def context():
    try:
        import getpass
        user = os.environ.get('SUDO_USER') or getpass.getuser()
    except Exception:                                     # noqa: BLE001
        user = os.environ.get('USER') or '?'
    # 报告要能对上「验的是哪份代码」。容器里 /tmp 不是 git 仓库，此时允许
    # 用 LIGHTDOG_COMMIT 显式传入，否则如实写「未知」而不是瞎猜一个。
    commit = os.environ.get('LIGHTDOG_COMMIT') or (
        sh(['git', '-C', os.path.dirname(os.path.abspath(__file__)),
            'rev-parse', '--short', 'HEAD']) or '未知')
    return {
        'when': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
        'host': sh(['hostname']) or '?',
        'user': user,
        'commit': commit,
        'distro': os.environ.get('ROS_DISTRO') or '?',
    }


# --------------------------------------------------------------------- main


CHECKS = {
    1: ('看门狗真的断 PWM', check_1),
    2: ('断的是使能不是指令', check_2),
    3: ('待命不误触发', check_3),
    4: ('急停闭锁', check_4),
    5: ('急停可继承', check_5),
    6: ('语音急停', check_6),
    7: ('语音解锁', check_7),
    8: ('亮度映射', check_8),
    9: ('电量不跳变', check_9),
    10: ('低电回充', check_10),
}

# 执行顺序和编号顺序不一样，因为**第 1 条会 kill 掉 twist_mux**。
# 按 1→10 跑的话，它一执行完，后面每一条都在「没有速度仲裁」的残局里跑，
# 结果全是假失败（甚至更糟：第 6 条的语音急停根本没人接，却可能碰巧通过）。
#
# 所以：先跑只读的（3/8/9），再跑改急停状态的（4/5/6/7，每条自己 finally 复位），
# 再跑会让车动起来的（10），最后才是破坏性的 1 和纯人工的 2。
ORDER = [3, 8, 9, 4, 5, 6, 7, 10, 2, 1]


def main():
    ap = argparse.ArgumentParser(
        prog='acceptance.py',
        description='LightDog 上车验收：跑 docs/firmware_protocol.md §七 的 10 条')
    ap.add_argument('--quick', action='store_true',
                    help='缩短耗时项（#3 600s->20s、#9 1800s->60s、#10 150s->60s）')
    ap.add_argument('--only', default='',
                    help='只跑指定条目，如 4,5,6')
    ap.add_argument('--auto-only', action='store_true',
                    help='不交互；人工项记为「待人工」，退出码 2')
    ap.add_argument('--mux-pid', default='', help='没有 systemd 时，twist_mux 的 PID'
                                               '（可逗号分隔多个）')
    ap.add_argument('--out', default='', help='报告路径（默认 deploy/acceptance-<时间>.md）')
    ap.add_argument('--no-report', action='store_true', help='只在终端输出')
    ap.add_argument('--allow-skip', default='',
                    help='显式承认某几条「测不了」，如 10 或 8,10（"all" 全接受）。\n'
                         '没在这里列出的跳过项会让退出码变成 2：没测到就是没测到')
    args = ap.parse_args()

    allow_skip = set()
    if args.allow_skip.strip().lower() == 'all':
        allow_skip = set(CHECKS)
    else:
        for tok in args.allow_skip.split(','):
            tok = tok.strip()
            if tok.isdigit():
                allow_skip.add(int(tok))

    want = set()
    if args.only:
        for tok in args.only.split(','):
            tok = tok.strip()
            if tok.isdigit():
                want.add(int(tok))

    print(f'{B}LightDog 上车验收{E}')
    ctx = context()
    print(f'  {ctx["when"]}  {ctx["host"]}  commit={ctx["commit"]}  '
          f'ROS={ctx["distro"]}')
    if args.quick:
        print(f'  {Y}快速模式{E}：耗时项已缩短，这份报告不能作为签字依据')

    rclpy.init()
    p = Probe()
    rep = Report(args.quick)
    try:
        p.pump(2.0)
        if not preflight(p, rep, args):
            print(f'\n{Y}前置不满足，后面的项全部跳过{E}')
            for no in ORDER:
                if no in want or not want:
                    rep.add(no, CHECKS[no][0], SKIP, '前置不满足')
            return finish(rep, ctx, args, allow_skip)

        for no in ORDER:
            if want and no not in want:
                continue
            title, fn = CHECKS[no]
            try:
                fn(p, rep, args)
            except KeyboardInterrupt:
                raise
            except Exception as exc:                      # noqa: BLE001
                # 单项炸了不能带走整份报告 —— 这张纸本身就是交付物
                rep.add(no, title, FAIL, f'检查本身出错：{type(exc).__name__}: {exc}')
        return finish(rep, ctx, args, allow_skip)
    except KeyboardInterrupt:
        print(f'\n{Y}中断{E}：已完成的项仍会写进报告')
        return finish(rep, ctx, args, allow_skip)
    finally:
        try:
            p.destroy_node()
        except Exception:                                 # noqa: BLE001
            pass
        rclpy.try_shutdown()


def finish(rep, ctx, args, allow_skip):
    if not args.no_report:
        out = args.out or os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            'acceptance-' + datetime.now().strftime('%Y%m%d-%H%M%S') + '.md')
        try:
            with open(out, 'w', encoding='utf-8') as f:
                f.write(rep.markdown(ctx, allow_skip))
            print(f'\n报告：{out}')
        except OSError as exc:
            print(f'{R}报告写不出来：{exc}{E}')

    n_pass, n_fail = rep.count(PASS), rep.count(FAIL)
    n_man, n_skip = rep.count(MANUAL), rep.count(SKIP)
    print(f'\n{B}结果{E}：{G}{n_pass} 通过{E} / {R}{n_fail} 失败{E} / '
          f'{Y}{n_man} 待人工{E} / {D}{n_skip} 跳过{E}')
    rc = rep.verdict(allow_skip)
    if rc == 0:
        print(f'{G}全部通过，可以上车{E}')
    elif rc == 1:
        print(f'{R}有未通过项，车不能上车{E}')
    else:
        left = rep.unacked_skips(allow_skip)
        if left:
            print(f'{Y}有 {len(left)} 条根本没测到{E}：'
                  + '、'.join(f'#{r["no"]} {r["title"]}' for r in left))
            print(f'  确认这条在真机上无法自动测（或已人工验过），'
                  f'再用 --allow-skip 逐条签字：'
                  f'--allow-skip {",".join(str(r["no"]) for r in left)}')
        if n_man:
            print(f'{Y}还有 {n_man} 条要人到现场确认{E} —— 清单没走完，不能当通过')
    return rc


if __name__ == '__main__':
    sys.exit(main())
