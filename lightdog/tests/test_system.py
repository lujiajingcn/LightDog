"""系统层离线回归：任务调度（模式仲裁）与语音链路。

这里不测「ROS 能不能跑起来」，测的是两件最容易被改坏的事：

1. **模式优先级**。`decide()` 里 if 的顺序就是安全策略：告警 > 急停 > 回充 >
   跟随 > 巡游。顺序一变，「老人躺在地上、机器人还在跟着人走」这种事不需要
   任何硬件故障就能发生。所以每一条都要单独钉住。
2. **语义歧义**。语音里「别跟着我」包含「跟」，「急停」包含「停」。匹配顺序
   错了，车会朝人开过去 —— 这是整套逻辑里唯一能靠一次错听造成物理后果的地方。

另一组是 P2 遗留的架构坑：跟随节点必须在调度层收回控制权时**停发**，
而不是继续发零速，否则 Nav2 永远拿不到方向盘。
"""

import json
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from harness import (  # noqa: E402
    Bool, Float32, Log, Msg, Pub, String,
    bind, extract_const, extract_funcs, extract_methods, source_of,
)

MISSION = source_of('lightdog_mission.py')
VOICE = source_of('lightdog_voice.py')
MUX = source_of('lightdog_twist_mux.py')


class FakeWall:
    """可控墙钟：抽出来的方法用注入的 `time`，所以能直接快进。"""

    def __init__(self, start=1000.0):
        self.t = float(start)

    def monotonic(self):
        return self.t


# ------------------------------------------------------------------ 调度层夹具

MISSION_METHODS = {
    'tick', 'decide', 'update_approach', '_apply', '_set_reason',
    '_send_follow', '_send_hold', 'staging_pose', '_publish_state',
    '_person_ready', '_dock_fresh', '_alert_fresh', '_alert_holds',
    '_ratio_fresh', '_nav_busy', '_cancel_nav', '_on_command',
    '_on_person', '_on_dock', '_on_ratio', '_on_alert', '_on_estop',
    '_nav_failed',
}


def make_mission(**over):
    m = type('Mission', (), {})()
    m.ESTOP, m.ALERT, m.DOCKING, m.DOCKED = (
        'ESTOP', 'ALERT_HOLD', 'DOCKING', 'DOCKED')
    m.APPROACH, m.FOLLOW, m.PATROL = 'DOCK_APPROACH', 'FOLLOW', 'PATROL'
    m.IDLE, m.MANUAL = 'IDLE', 'MANUAL'

    # 时间窗口
    m.person_timeout = 2.0
    m.dock_timeout = 5.0
    m.alert_timeout = 5.0
    # 开关
    m.auto_mode = True
    m.follow_enabled = True
    m.patrol_enabled = False
    m.patrol_dwell = 5.0
    # 回充接近
    m.dock_approach = True
    m.approach_ratio = 0.35
    m.clear_ratio = 0.45
    m.approach_confirm = 5.0
    m.approach_cooldown = 60.0
    m.approach_timeout = 180.0
    m.dock_x, m.dock_y, m.dock_yaw = -0.5, 0.0, 0.0
    m.staging_distance = 1.0
    m.map_frame = 'map'
    m.nav_action = '/navigate_to_pose'
    # 输入
    m.person_present = False
    m.person_wall = 0.0
    m.dock_state = ''
    m.dock_wall = 0.0
    m.ratio = None
    m.ratio_wall = 0.0
    m.alert_state = ''
    m.alert_wall = 0.0
    m.estop = False
    # 决策状态
    m.manual = False
    m.mode = ''
    m.mode_since = 0.0
    m.reason = ''
    m.approach_wanted = False
    m._approach_manual = False
    m._low_since = None
    m._approach_cooldown_until = 0.0
    m.waypoints = []
    m._wp_index = 0
    m._patrol_wait_until = 0.0
    m._follow_sent = None
    m._follow_beat = 0.0
    m._hold_sent = None
    m._hold_beat = 0.0
    m._nav_state = None
    m._nav_tag = None
    m._nav_epoch = 0
    m._nav_handle = None
    m._nav_sent_at = 0.0
    m._nav_cooldown_until = 0.0
    m._nav_fail = 0
    m._nav_warned_unavailable = False
    # 发布器
    m.pub_follow = Pub()
    m.pub_hold = Pub()
    m.pub_dock = Pub()
    m.pub_state = Pub()
    m.pub_mode = Pub()
    m.log = Log()
    m.get_logger = lambda: m.log
    m.wall = FakeWall()

    for k, v in over.items():
        setattr(m, k, v)

    funcs = extract_methods(
        MISSION, 'Mission', MISSION_METHODS,
        ns={'time': m.wall, 'json': json, 'math': math,
            'Bool': Bool, 'Float32': Float32, 'String': String,
            # _alert_holds 直接引用这个模块常量，不放进来会 NameError
            'ALERT_HOLD_STATES': extract_const(MISSION, 'ALERT_HOLD_STATES')})
    funcs['make_quaternion'] = extract_funcs(
        MISSION, {'make_quaternion'}, ns={'math': math})['make_quaternion']
    funcs['parse_waypoints'] = extract_funcs(
        MISSION, {'parse_waypoints'}, ns={'math': math})['parse_waypoints']
    # _apply 里的导航分支只记录调用，不发真 action
    bind(m, funcs)
    m.nav_calls = []
    m._ensure_nav_approach = lambda now: m.nav_calls.append('approach')
    m._ensure_nav_patrol = lambda now: m.nav_calls.append('patrol')
    return m


def feed(m, person=False, dock='IDLE', ratio=None, alert=None, now=None):
    """一次性布置输入快照（时间戳都用当前墙钟）。"""
    t = m.wall.t if now is None else now
    m.person_present = person
    m.person_wall = t
    m.dock_state = dock
    m.dock_wall = t
    if ratio is not None:
        m.ratio = ratio
        m.ratio_wall = t
    if alert is not None:
        m.alert_state = alert
        m.alert_wall = t


class TestModeArbitration(unittest.TestCase):

    def test_nobody_idle_and_follow_closed(self):
        m = make_mission()
        feed(m, person=False, dock='', ratio=0.9)
        m.tick()
        self.assertEqual(m.mode, m.IDLE)
        self.assertEqual(m.pub_follow.values[-1], False,
                         '无人时必须收回跟随控制权，否则跟随的零速会压住 Nav2')
        self.assertEqual(m.pub_hold.values[-1], False)

    def test_person_starts_follow(self):
        m = make_mission()
        feed(m, person=True, dock='', ratio=0.9)
        m.tick()
        self.assertEqual(m.mode, m.FOLLOW)
        self.assertEqual(m.pub_follow.values[-1], True)

    def test_idle_reason_does_not_stay_stale(self):
        """IDLE 必须有自己的 reason。

        少了这一支，`/mission/state` 会输出「mode=IDLE, reason=检测到人」这种
        自相矛盾的快照 —— 人走了、车停着，状态里却还写着看见人。手机端和
        现场排查都会被它带偏。
        """
        m = make_mission()
        feed(m, person=True, dock='', ratio=0.9)
        m.tick()
        self.assertEqual(m.mode, m.FOLLOW)
        feed(m, person=False, dock='', ratio=0.9)
        m.wall.t += 5.0                     # 让 present 过期
        m.tick()
        self.assertEqual(m.mode, m.IDLE)
        self.assertNotIn('检测到人', m.reason,
                         f'IDLE 沿用了上一轮的理由：{m.reason!r}')
        self.assertTrue(m.reason)

    def test_stale_dock_state_is_ignored(self):
        """dock_manager 死了以后，最后那条 DOCKED 不能永久粘住。"""
        m = make_mission()
        feed(m, person=True, dock='DOCKED', now=m.wall.t)
        m.wall.t += 6.0                     # 超过 dock_state_timeout(5s)
        m.person_wall = m.wall.t            # 人是活着的，只有 dock 状态断了
        self.assertEqual(m.decide(m.wall.t), m.FOLLOW,
                         '回充状态断流后还在按 DOCKED 处理，车会永远不动')

    def test_docking_wins_over_follow(self):
        m = make_mission()
        feed(m, person=True, dock='ALIGNING', ratio=0.2)
        m.tick()
        self.assertEqual(m.mode, m.DOCKING)
        self.assertFalse(m.pub_follow.values[-1], '回充期间跟随必须让位')

    def test_estop_beats_everything(self):
        m = make_mission()
        feed(m, person=True, dock='DOCKING', ratio=0.2, alert='ALARM')
        m.estop = True
        m.tick()
        self.assertEqual(m.mode, m.ESTOP)
        self.assertTrue(m.pub_hold.values[-1], '急停时必须按住轮子')
        self.assertFalse(m.pub_follow.values[-1])

    def test_alert_holds_and_blocks_follow(self):
        m = make_mission()
        feed(m, person=True, dock='IDLE', alert='CONFIRMING')
        m.tick()
        self.assertEqual(m.mode, m.ALERT)
        self.assertTrue(m.pub_hold.values[-1],
                        '告警确认中人可能就躺在轮子前，必须按住')
        self.assertFalse(m.pub_follow.values[-1])

    def test_resolved_releases_hold(self):
        m = make_mission()
        feed(m, person=True, dock='IDLE', alert='RESOLVED')
        m.tick()
        self.assertEqual(m.mode, m.FOLLOW)
        self.assertFalse(m.pub_hold.values[-1],
                         'RESOLVED 表示人已起身，应该放开而不是继续按住')

    def test_stale_alert_does_not_freeze_robot(self):
        """告警节点挂了不能把车永久冻住：超时后按无告警处理。"""
        m = make_mission()
        m.alert_state = 'ALARM'
        m.alert_wall = m.wall.t - 10.0     # 超过 alert_timeout(5s)
        feed(m, person=True, dock='IDLE')
        m.tick()
        self.assertNotEqual(m.mode, m.ALERT)
        self.assertFalse(m.pub_hold.values[-1])

    def test_manual_mode_allows_follow_but_no_auto(self):
        m = make_mission()
        m.manual = True
        feed(m, person=True, dock='IDLE', ratio=0.10)   # 低电也不自动走
        m.tick()
        self.assertEqual(m.mode, m.MANUAL)
        self.assertEqual(m.pub_follow.values[-1], True,
                         '人工接管时跟随应放行，否则接管后连跟随都用不了')
        self.assertEqual(m.nav_calls, [])

    def test_patrol_only_when_enabled_and_nobody(self):
        m = make_mission(patrol_enabled=True, waypoints=[(1.0, 0.0, 0.0)])
        feed(m, person=False, dock='IDLE', ratio=0.9)
        m.tick()
        self.assertEqual(m.mode, m.PATROL)
        self.assertEqual(m.nav_calls, ['patrol'])

        feed(m, person=True, dock='IDLE', ratio=0.9)
        m.tick()
        self.assertEqual(m.mode, m.FOLLOW, '有人时必须中断巡游')


class TestApproach(unittest.TestCase):

    def test_low_battery_needs_confirmation(self):
        m = make_mission()
        feed(m, person=False, dock='IDLE', ratio=0.30)
        m.tick()
        self.assertEqual(m.mode, m.IDLE, '瞬时压降不应该立刻触发回桩')
        self.walk(m, 5.5, dock='IDLE', ratio=0.30)
        self.assertEqual(m.mode, m.APPROACH)
        self.assertIn('approach', m.nav_calls)

    def walk(self, m, secs, step=0.5, **kw):
        """按固定步长推进墙钟并持续刷新输入（模拟 ROS 回调持续到达）。"""
        n = max(1, int(round(secs / step)))
        for _ in range(n):
            m.wall.t += step
            feed(m, now=m.wall.t, **kw)
            m.tick()

    def test_high_battery_never_triggers(self):
        m = make_mission()
        for _ in range(20):
            m.wall.t += 1.0
            feed(m, person=False, dock='IDLE', ratio=0.80, now=m.wall.t)
            m.tick()
        self.assertEqual(m.mode, m.IDLE)
        self.assertEqual(m.nav_calls, [])

    def test_hysteresis_avoids_flapping(self):
        m = make_mission()
        feed(m, person=False, dock='IDLE', ratio=0.30)
        self.walk(m, 5.5, dock='IDLE', ratio=0.30)
        self.assertTrue(m.approach_wanted)

        # 只回到 0.40（高于 0.35 但低于 clear_ratio 0.45）—— 不应该解除
        self.walk(m, 2.0, dock='IDLE', ratio=0.40)
        self.assertTrue(m.approach_wanted, '在阈值附近抖动会导致边走边取消')

        self.walk(m, 2.0, dock='IDLE', ratio=0.50)
        self.assertFalse(m.approach_wanted)

    def test_staging_pose_is_in_front_of_dock(self):
        m = make_mission()
        x, y, yaw = m.staging_pose()
        self.assertAlmostEqual(x, 0.5, places=6)
        self.assertAlmostEqual(y, 0.0, places=6)
        # 朝向要指着桩（-x 方向）
        self.assertAlmostEqual(abs(yaw), math.pi, places=6)

    def test_docking_in_progress_blocks_approach(self):
        m = make_mission()
        for _ in range(12):
            m.wall.t += 1.0
            feed(m, person=False, dock='DOCKING', ratio=0.20, now=m.wall.t)
            m.tick()
            self.assertEqual(m.mode, m.DOCKING)
        self.assertFalse(m.approach_wanted,
                         'dock_manager 已经在停靠，调度层不该再插一脚')


class TestCommandInterface(unittest.TestCase):

    def test_follow_off_then_on(self):
        m = make_mission()
        m._on_command(Msg('follow_off'))
        feed(m, person=True, dock='IDLE', ratio=0.9)
        m.tick()
        self.assertEqual(m.mode, m.IDLE)
        m._on_command(Msg('follow'))
        self.assertEqual(m.mode, m.FOLLOW)

    def test_unknown_command_is_ignored(self):
        m = make_mission()
        feed(m, person=True, dock='IDLE', ratio=0.9)
        m.tick()
        before = list(m.pub_follow.values)
        m._on_command(Msg('launch_the_missiles'))
        self.assertEqual(m.pub_follow.values, before)
        self.assertIn('未知命令', m.log.text())

    def test_status_is_parseable_json(self):
        m = make_mission()
        feed(m, person=True, dock='IDLE', ratio=0.42)
        m.tick()
        raw = m.pub_state.last.data
        st = json.loads(raw)
        self.assertEqual(st['mode'], m.FOLLOW)
        self.assertAlmostEqual(st['battery'], 0.42, places=3)
        self.assertIn('reason', st)


class TestWaypointParsing(unittest.TestCase):

    def test_semicolon_string(self):
        f = extract_funcs(MISSION, {'parse_waypoints'},
                          ns={'math': math})['parse_waypoints']
        pts, bad = f('1.0,0.5,0.0; -1.0,0.5,3.14')
        self.assertEqual(bad, 0)
        self.assertEqual(pts, [(1.0, 0.5, 0.0), (-1.0, 0.5, 3.14)])

    def test_yaw_is_optional(self):
        f = extract_funcs(MISSION, {'parse_waypoints'},
                          ns={'math': math})['parse_waypoints']
        self.assertEqual(f('1,2; 3,4')[0], [(1.0, 2.0, 0.0), (3.0, 4.0, 0.0)])

    def test_bad_entries_are_dropped_not_guessed(self):
        f = extract_funcs(MISSION, {'parse_waypoints'},
                          ns={'math': math})['parse_waypoints']
        pts, bad = f('1.0,0.5; nonsense; 2.0; 3.0,4.0,0.0; nan,1,0')
        self.assertEqual(pts, [(1.0, 0.5, 0.0), (3.0, 4.0, 0.0)])
        self.assertGreaterEqual(bad, 3, '坏路点必须被计数，不能悄悄丢掉')

    def test_empty_means_no_patrol(self):
        f = extract_funcs(MISSION, {'parse_waypoints'},
                          ns={'math': math})['parse_waypoints']
        self.assertEqual(f(''), ([], 0))
        self.assertEqual(f(None), ([], 0))


# ------------------------------------------------------------------ 语音链路

def voice_funcs():
    """抽三个纯函数。它们互相调用、还引用模块常量，ns 要逐层补齐。"""
    rules = extract_const(VOICE, 'RULES')
    known = {name for name, _ in rules} | {'none'}
    norm = extract_funcs(VOICE, {'normalize'})['normalize']
    base = {'json': json, 'RULES': rules, 'KNOWN_INTENTS': known,
            'normalize': norm}
    f = extract_funcs(VOICE, {'normalize', 'match_intent'}, ns=base)
    f.update(extract_funcs(VOICE, {'parse_command'},
                           ns=dict(base, match_intent=f['match_intent'])))
    return f


class TestManualDock(unittest.TestCase):
    """手动回充不能被「自动低电迟滞」吃掉。"""

    def _nav_ready(self, m):
        m.nav_client = type('C', (), {'server_is_ready': lambda self: True})()

    def test_full_battery_still_honours_manual_dock(self):
        m = make_mission()
        self._nav_ready(m)
        feed(m, person=False, dock='IDLE', ratio=0.90)
        m._on_command(String(data='dock'))
        self.assertTrue(m.approach_wanted, '手动 dock 命令应当被接受')
        m.tick()
        self.assertTrue(
            m.approach_wanted,
            '满电时手动回充被 update_approach 当成「电量已恢复」清掉了')
        self.assertEqual(m.mode, m.APPROACH)

    def test_auto_approach_is_still_cleared_when_battery_recovers(self):
        m = make_mission()
        feed(m, person=False, dock='IDLE', ratio=0.90)
        m.approach_wanted = True          # 自动判定留下的
        m.tick()
        self.assertFalse(m.approach_wanted,
                         '自动回充必须在电量回到 clear_ratio 以上时解除')

    def test_manual_dock_cleared_after_nav_failure(self):
        m = make_mission()
        m.approach_wanted = True
        m._approach_manual = True
        m._nav_tag = 'approach'
        m._nav_state = 'active'
        m._nav_failed(0, 'aborted', 'approach')
        self.assertFalse(m.approach_wanted)
        self.assertFalse(m._approach_manual)
        self.assertIn('导航失败', m.log.text())

    def test_manual_dock_still_works_without_approach_mode(self):
        """dock_approach=false 时手动「去充电」也不能被吞。

        那一支在 update_approach 里第一行就把 approach_wanted 清掉，而 decide()
        又不会进 APPROACH —— 命令等于什么都没做，reason 却还写着「先导航到桩前」。
        """
        m = make_mission(dock_approach=False)
        self._nav_ready(m)                 # 就算 Nav2 就绪，也不该走导航
        feed(m, person=False, dock='IDLE', ratio=0.90)

        m._on_command(String(data='dock'))

        self.assertEqual(m.pub_dock.values, ['dock'],
                         '关闭接近模式后手动回充没有交给 dock_manager')


class TestUndocking(unittest.TestCase):

    def test_undocking_is_not_interrupted_by_patrol(self):
        """离桩过程中调度层不能发巡游导航目标。

        UNDOCKING 不在「回充进行中」里的话，mode 会掉到 PATROL，
        _apply 就往同一个 Nav2 发目标 —— 一边离桩一边导航。
        """
        m = make_mission(patrol_enabled=True, waypoints=[(1.0, 0.0, 0.0)])
        feed(m, person=False, dock='UNDOCKING', ratio=0.90)

        m.tick()

        self.assertEqual(m.mode, m.DOCKING)
        self.assertEqual(m.nav_calls, [], '离桩时调度层还在发导航目标')


class TestStopCommand(unittest.TestCase):

    def test_stop_command_stops_everything(self):
        """「停下」不能只停跟随：巡游/回充还在跑的话车照旧往前走。"""
        m = make_mission()
        m.patrol_enabled = True
        m.waypoints = [(1.0, 0.0, 0.0)]
        m.approach_wanted = True
        m._nav_tag = 'patrol'
        m._nav_state = 'active'
        m._on_command(String(data='stop'))
        self.assertFalse(m.follow_enabled)
        self.assertFalse(m.patrol_enabled)
        self.assertFalse(m.approach_wanted)
        self.assertIn('cancel', m.pub_dock.values)
        self.assertEqual(m.mode, m.IDLE)

    def test_stop_does_not_enter_manual(self):
        """MANUAL 仍然给跟随放行（见 _apply），所以「停下」绝不能设成 manual。"""
        m = make_mission()
        m._on_command(String(data='stop'))
        self.assertFalse(m.manual)
        self.assertEqual(m.pub_follow.values[-1], False)


class TestVoiceIntents(unittest.TestCase):

    def setUp(self):
        self.f = voice_funcs()

    def test_negative_phrase_does_not_mean_come(self):
        """「别跟着我」同时含「跟」和「别跟」——顺序错了车会朝人开过来。"""
        self.assertEqual(self.f['match_intent']('别跟着我'), 'stop_follow')
        self.assertEqual(self.f['match_intent']('不要再跟着我了'), 'stop_follow')
        self.assertEqual(self.f['match_intent']('不要跟'), 'stop_follow')

    def test_estop_beats_plain_stop(self):
        self.assertEqual(self.f['match_intent']('急停'), 'estop')
        self.assertEqual(self.f['match_intent']('紧急停止'), 'estop')
        self.assertEqual(self.f['match_intent']('停下'), 'stop')
        self.assertEqual(self.f['match_intent']('别动'), 'stop')

    def test_clear_estop_is_not_swallowed_by_estop(self):
        """「解除急停」含「急停」。

        RULES 里 estop 原本排在 clear_estop 前面，于是唯一的解锁口令被执行成
        「再加一次锁」—— 现场表现是喊多少遍都解不开。这类「子串吞掉父串」的
        顺序错误必须逐条钉住，不能只靠肉眼审意图表。
        """
        self.assertEqual(self.f['match_intent']('解除急停'), 'clear_estop')
        self.assertEqual(self.f['match_intent']('取消急停'), 'clear_estop')
        self.assertEqual(self.f['match_intent']('解除刹车'), 'clear_estop')
        # 反向不能被带偏：光说「急停」「刹车」仍然是加锁
        self.assertEqual(self.f['match_intent']('刹车'), 'estop')

    def test_light_commands(self):
        self.assertEqual(self.f['match_intent']('把灯打开'), 'light_on')
        self.assertEqual(self.f['match_intent']('关灯'), 'light_off')
        self.assertEqual(self.f['match_intent']('灯光自动'), 'light_auto')

    def test_charge_commands(self):
        self.assertEqual(self.f['match_intent']('回去充电'), 'dock')
        self.assertEqual(self.f['match_intent']('去充电'), 'dock')
        self.assertEqual(self.f['match_intent']('离桩'), 'undock')

    def test_patrol_negation_does_not_start_patrol(self):
        """「别巡逻」含「巡逻」。patrol_on 排在前面的话，一句劝阻会让车开始巡游。"""
        for txt in ('别巡逻', '不用巡逻', '停止巡逻', '取消巡逻'):
            self.assertEqual(self.f['match_intent'](txt), 'patrol_off', txt)
        # 正向不能被误伤
        self.assertEqual(self.f['match_intent']('去巡逻'), 'patrol_on')
        self.assertEqual(self.f['match_intent']('开始巡逻'), 'patrol_on')

    def test_charge_negation_does_not_dock(self):
        """「别充电」含「充电」。dock 排前面的话，劝阻会变成派车去充电。"""
        for txt in ('不要充电', '别充电', '不用充电', '不用回充'):
            self.assertEqual(self.f['match_intent'](txt), 'stop_dock', txt)
        self.assertEqual(self.f['match_intent']('去充电'), 'dock')

    def test_stop_maps_to_full_stop(self):
        cmd = extract_const(VOICE, 'MISSION_CMD')
        self.assertEqual(cmd['stop'], 'stop',
                         '「停下」只发 follow_off 的话，巡游中的车不会停')
        self.assertEqual(cmd['stop_dock'], 'cancel')

    def test_undock_keeps_unambiguous_phrases_only(self):
        """「出来」在中文里更常是「过来」，不能当离桩。"""
        # '你出来' 里没有任何「跟/停/充电」关键词，摘掉 '出来' 之后应当匹配不到东西
        self.assertIsNone(self.f['match_intent']('你出来'))
        self.assertEqual(self.f['match_intent']('离桩'), 'undock')

    def test_unknown_text_falls_through(self):
        for txt in ('我不太舒服', '', '   ', '今天天气不错'):
            self.assertIsNone(self.f['match_intent'](txt),
                              f'{txt!r} 不该被匹配成任何动作')

    def test_json_intent_wins_over_text(self):
        payload = json.dumps({'intent': 'light_on', 'text': '关灯', 'confidence': 0.9})
        intent, text, conf = self.f['parse_command'](payload)
        self.assertEqual(intent, 'light_on', 'ASR 已经分好类时不应再用关键词猜')
        self.assertEqual(text, '关灯')
        self.assertAlmostEqual(conf, 0.9)

    def test_unknown_json_intent_falls_back_to_text(self):
        payload = json.dumps({'intent': 'mystery', 'text': '把灯关掉'})
        intent, _, _ = self.f['parse_command'](payload)
        self.assertEqual(intent, 'light_off')

    def test_broken_json_is_treated_as_plain_text(self):
        # 截断的 JSON 解析失败后整串当文本用；它里面确实含「开灯」，
        # 所以会被关键词匹配到 —— 这是有意的降级：宁可执行一个明确写在
        # 括号里的词，也不要因为报文格式坏了就整条丢弃。
        intent, text, _ = self.f['parse_command']('{"text": "开灯"')
        self.assertEqual(intent, 'light_on')
        self.assertIn('{', text)


# ------------------------------------------------------------------ 安全闸

def make_mux():
    from harness import Twist, extract_class  # 局部导入，避免顶部未使用告警
    Src = extract_class(MUX, 'CmdVelSource')
    m = type('Mux', (), {})()
    m.sources = [
        Src('dock', '/cmd_vel_dock', 30, 1.0, 0.50),
        Src('follow', '/cmd_vel_follow', 20, 0.6, 0.30),
        Src('nav', '/cmd_vel_smoothed', 10, 0.6, 0.30),
    ]
    m.zero_on_idle = True
    m.max_lin, m.max_ang = 0.50, 1.50
    m._warned_clamp = False
    m.pub, m.pub_state, m.pub_beat = Pub(), Pub(), Pub()
    m.pub_estop = Pub()
    m.active_name = None
    m._stale = False
    m.estop = False
    m.hold = False
    m._hold_stamp = 0.0
    m.hold_timeout = 3.0
    m.log = Log()
    m.get_logger = lambda: m.log
    m.wall = FakeWall()
    clamp = extract_funcs(MUX, {'clamp_speed'})['clamp_speed']
    bind(m, extract_methods(MUX, 'TwistMux',
                            {'_tick', '_pick', '_on_twist', '_gate',
                             '_on_estop', '_on_hold'},
                            ns={'time': m.wall, 'String': String,
                                'Bool': Bool, 'Twist': Twist,
                                'CmdVelSource': Src,
                                'clamp_speed': clamp}))
    m.Twist = Twist
    return m


def push(m, src_name, v=0.25, w=0.0):
    """往某个源灌一条新鲜指令。"""
    from harness import Twist
    src = next(s for s in m.sources if s.name == src_name)
    tw = Twist()
    tw.linear.x, tw.angular.z = v, w
    m._on_twist(tw, src)


class TestSafetyGate(unittest.TestCase):

    def test_estop_outputs_zero_despite_fresh_source(self):
        m = make_mux()
        m._on_estop(Msg(True))
        for _ in range(3):
            m.wall.t += 0.05
            push(m, 'nav', 0.30)
            m._tick()
        self.assertTrue(all(m.pub.sent[i].is_zero() for i in range(len(m.pub.sent))),
                        '急停期间任何上游都不许把速度透传出去')
        self.assertEqual(m.pub_state.last.data, 'estop')

    def test_estop_needs_explicit_release(self):
        m = make_mux()
        m._on_estop(Msg(True))
        m.wall.t += 0.05
        m._tick()
        m.wall.t += 10.0                   # 时间过去很久，闸不能自己开
        push(m, 'nav', 0.30)
        m._tick()
        self.assertTrue(m.pub.last.is_zero(), '急停是闭锁的，不能超时自解')
        m._on_estop(Msg(False))
        m.wall.t += 0.05
        push(m, 'nav', 0.30)
        m._tick()
        self.assertAlmostEqual(m.pub.last.linear.x, 0.30, places=6)

    def test_estop_is_republished_to_keep_the_latch_alive(self):
        """mux 必须把急停状态回发一份。

        `/safety/estop` 的写者（CLI / 语音 / 网页按钮）都是短命或条件存活的，
        TRANSIENT_LOCAL 的锁存样本随最后一个 writer 退出而消失。没有这层回发，
        「CLI 按下急停 -> 只重启 mission」就会让新 mission 从 estop=False 起步，
        而 mux 还锁着 —— 两个消费者对同一个安全状态各记一套。
        """
        m = make_mux()
        m._on_estop(Msg(True))
        self.assertEqual(m.pub_estop.values, [True],
                         '收到急停后必须回发一份，让锁存有长期持有者')
        # 重复收到同值不能再发（否则和订阅形成回环）
        m._on_estop(Msg(True))
        self.assertEqual(len(m.pub_estop.values), 1)
        m._on_estop(Msg(False))
        self.assertEqual(m.pub_estop.values, [True, False])

    def test_hold_expires_when_mission_dies(self):
        m = make_mux()
        m._on_hold(Msg(True))
        m.wall.t += 0.05
        m._tick()
        self.assertTrue(m.pub.last.is_zero())

        m.wall.t += 5.0                    # 超过 hold_timeout(3s) 且没人刷新
        push(m, 'nav', 0.30)
        m._tick()
        self.assertAlmostEqual(m.pub.last.linear.x, 0.30, places=6,
                               msg='调度层崩溃不该把机器人永久冻住')
        self.assertIn('判定调度层已退出', m.log.text('error'))

    def test_heartbeat_every_tick(self):
        m = make_mux()
        for _ in range(4):
            m.wall.t += 0.05
            m._tick()
        self.assertEqual(len(m.pub_beat.sent), 4,
                         '心跳必须每拍都发，否则固件看门狗会误判上位机死亡')

    def test_hold_does_not_release_estop(self):
        """两个闸分开写：解除 hold 不能顺手把 estop 也解了。"""
        m = make_mux()
        m._on_estop(Msg(True))
        m._on_hold(Msg(True))
        m._on_hold(Msg(False))
        push(m, 'nav', 0.30)
        m.wall.t += 0.05
        m._tick()
        self.assertTrue(m.pub.last.is_zero())
        self.assertEqual(m.pub_state.last.data, 'estop')


# ------------------------------------------------------- 跟随：停发而不是零速

class TestFollowGate(unittest.TestCase):

    def test_disabled_publishes_zero_once_then_silence(self):
        f = make_follower_gate()
        f.enable_cmd = False
        f.enable_stamp = f.wall.t
        for _ in range(5):
            f.wall.t += 0.1
            f._tick()
        self.assertEqual(len(f.pub.sent), 1,
                         '停发前必须发且只发一条零速；继续发零速会永久压住 Nav2')
        self.assertTrue(f.pub.sent[0].is_zero())

    def test_enable_resumes_publishing(self):
        f = make_follower_gate()
        f.enable_cmd = False
        f.enable_stamp = f.wall.t
        f._tick()
        f._on_enable(Msg(True))
        f._tick()
        self.assertEqual(len(f.pub.sent), 2)
        self.assertGreater(f.pub.sent[-1].linear.x, 0.0)

    def test_gate_expires_after_mission_death(self):
        f = make_follower_gate()
        f.enable_cmd = False
        f.enable_stamp = f.wall.t
        f.wall.t += f.enable_timeout + 1.0
        # 人还在（刷新两条时间戳），只有调度层没消息了
        f.last_present = f.pos_wall = f.wall.t
        f._tick()
        self.assertGreater(f.pub.last.linear.x, 0.0,
                           '调度层不再刷新时必须回落到自己的 enabled')
        self.assertIn('判定调度层已退出', f.log.text('error'))

    def test_no_enable_message_keeps_legacy_behaviour(self):
        f = make_follower_gate()
        self.assertIsNone(f.enable_cmd)
        f._tick()
        self.assertGreater(f.pub.last.linear.x, 0.0,
                           '没收到过 /follow/enable 时应按 enabled 工作')


def make_follower_gate():
    from harness import PointStamped, Twist
    f = type('Follower', (), {})()
    f.enabled = True
    f.inhibited = False
    f.inhibit_stamp = 0.0
    f.inhibit_timeout = 5.0
    f.person = PointStamped(1.6, 0.0)
    f.wall = FakeWall()
    f.last_present = f.wall.t
    f.pos_wall = f.wall.t
    f.expected_frame = 'base_link'
    f._warned_frame = ''
    f._warned_pos = False
    f.enable_cmd = None
    f.enable_stamp = 0.0
    f.enable_timeout = 5.0
    f._warned_enable = False
    f._silent = False
    f.timeout = 1.5
    f.target, f.tol, f.stop_d = 1.20, 0.15, 0.65
    f.k_v, f.max_v = 0.8, 0.25
    f.ang_tol, f.k_w, f.max_w = 0.10, 1.2, 1.0
    f.side_only = 0.9
    f.pub = Pub()
    f.log = Log()
    f.get_logger = lambda: f.log
    bind(f, extract_methods(
        source_of('lightdog_follower.py'), 'PersonFollower',
        {'_tick', '_person_lost', '_check_frame', '_follow_gate', '_on_enable'},
        ns={'time': f.wall, 'math': math, 'Twist': Twist, 'Bool': Bool,
            'PointStamped': PointStamped}))
    return f


if __name__ == '__main__':
    unittest.main(verbosity=2)
