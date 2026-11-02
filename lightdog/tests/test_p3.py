"""P3 回归：自动回充状态机。

对应验收脚本 sim/p3.sh，只重放状态机与充电积分的逻辑，不含 ArUco 解算
（那部分要真图像 + cv2.aruco，留给容器里跑）。

覆盖这一轮修的点：
  1. DOCKED 也必须在 /dock/inhibit 里（否则充电时人走过会把车开走，
     状态机却还停在 DOCKED 继续积分 → 「没插着电，电量越充越多」）
  2. sim_charging 用 ROS 时钟积分，不用墙钟（0.22x 实时因子下墙钟会快 4.5 倍）
  3. cancel() 真的调用 goal_handle.cancel_goal_async()，不是只把状态改掉
  4. 取消后迟到的 result 必须丢弃，不能记一次「停靠失败」并累加重试
  5. 低电要连续确认 low_confirm_time 才发起回充（避免电压抖动触发）
"""

import math
import os
import sys
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from harness import (  # noqa: E402
    Bool, Clock, Float32, Future, GoalHandle, Log, Msg, Pub, String, Twist,
    bind, extract_methods, source_of,
)

DOCK = source_of('lightdog_dock_manager.py')

IDLE, ALIGNING, DOCKING, DOCKED, UNDOCKING = (
    'IDLE', 'ALIGNING', 'DOCKING', 'DOCKED', 'UNDOCKING')

CONSTS = {'IDLE': IDLE, 'ALIGNING': ALIGNING, 'DOCKING': DOCKING,
          'DOCKED': DOCKED, 'UNDOCKING': UNDOCKING}


class DockResult:
    def __init__(self, success, error_code=0, error_msg='', num_retries=0):
        self.success = success
        self.error_code = error_code
        self.error_msg = error_msg
        self.num_retries = num_retries


def make_dock(state=IDLE):
    d = type('DockManager', (), {})()
    d.state = state
    d.busy = False
    d.retries = 0
    d.max_retries = 3
    d.auto_dock = True
    d.cooldown_until = 0.0
    d.retry_until = 0.0
    d.retry_cooldown = 10.0
    d.cancel_cooldown = 120.0
    d.low_since = None
    d.low_ratio = 0.25
    d.low_voltage = 11.0
    d.low_confirm_time = 3.0
    d.full_ratio = 0.99
    d.full_cooldown = 60.0
    d._inhibit_until = 0.0
    d.battery = None
    # 充电积分
    d.sim_charging = True
    d.sim_charge_rate = 0.1        # Ah / 仿真秒
    d.capacity = 10.0              # Ah
    d.charge_ah = None
    # 模拟电量注入
    d.sim_ratio = 0.15
    d.sim_ratio_stamp = time.monotonic()
    d.sim_battery_timeout = 5.0
    # 订阅/发布
    d.pub_ratio = Pub()
    d.pub_state = Pub()
    d.pub_inhibit = Pub()
    d.pub_vel = Pub()
    # action 状态
    d._dock_epoch = 0
    d._undock_epoch = 0
    d._dock_goal_handle = None
    d._undock_goal_handle = None
    d.align_done = False
    # 外部依赖
    d.clock = Clock()
    d.log = Log()
    d.get_clock = lambda: d.clock
    d.get_logger = lambda: d.log

    def start_undocking():
        d.log.info('(stub) start_undocking')
        d.state = UNDOCKING

    d.start_undocking = start_undocking
    d.dock_calls = []

    def start_docking(manual=False):
        d.dock_calls.append(manual)
        d.state = DOCKING

    d.start_docking = start_docking
    bind(d, extract_methods(
        DOCK, 'DockManager',
        {'tick', 'battery_ratio', 'cancel', '_log_cancel', 'fail_docking',
         'on_dock_result', 'on_dock_goal_response', '_check_charging'},
        ns={'time': time, 'math': math, 'Float32': Float32, 'String': String,
            'Bool': Bool, 'Twist': Twist, **CONSTS}))
    return d


class TestInhibit(unittest.TestCase):

    def test_docked_holds_inhibit(self):
        """DOCKED 必须抑制跟随。修之前这里是 False，人一经过就被开走。"""
        d = make_dock(DOCKED)
        d.tick()
        self.assertTrue(d.pub_inhibit.last.data,
                        'DOCKED 状态下 /dock/inhibit 是 false')

    def test_idle_releases_inhibit(self):
        d = make_dock(IDLE)
        d.tick()
        self.assertFalse(d.pub_inhibit.last.data)

    def test_inhibit_covers_cancel_window(self):
        """取消后要留 1.5s 抑制：docking_server 还没真正停下来。"""
        d = make_dock(DOCKING)
        d.cancel()
        d.tick()
        self.assertTrue(d.pub_inhibit.last.data, '取消窗口内就放开抑制了')


class TestCharging(unittest.TestCase):

    def test_charging_uses_ros_clock(self):
        """ROS 时间推进 4s、墙钟几乎不动。只有用 ROS 时钟积分才会 +0.4Ah。"""
        d = make_dock(DOCKED)
        d.charge_ah = 5.0
        d.sim_ratio = None                  # 关掉注入，走 charge_ah 分支
        d.tick()                            # 第一拍只记基线
        self.assertAlmostEqual(d.charge_ah, 5.0)

        d.clock.advance(4.0)
        d.tick()
        self.assertAlmostEqual(d.charge_ah, 5.4, places=3,
                               msg='充电量不对：可能在用墙钟积分')
        self.assertAlmostEqual(d.pub_ratio.last.data, 0.54, places=3)

    def test_single_tick_dt_is_clamped(self):
        """时钟跳变（仿真暂停/恢复、ros bag 回放）不能一次灌满整块电池。"""
        d = make_dock(DOCKED)
        d.charge_ah = 1.0
        d.sim_ratio = None
        d.full_ratio = 1.01                 # 别在半路触发离桩
        d.tick()
        d.clock.advance(600.0)              # 跳 10 分钟
        d.tick()
        self.assertAlmostEqual(d.charge_ah, 1.5, places=3,
                               msg='sim_dt 没有上限，时钟一跳电就充满')

    def test_charging_stops_when_full(self):
        d = make_dock(DOCKED)
        d.charge_ah = 9.95
        d.sim_ratio = None
        d.tick()
        d.clock.advance(10.0)
        d.tick()                            # 越过 full_ratio → 应该去离桩
        self.assertEqual(d.state, UNDOCKING)
        self.assertIsNone(d.charge_ah, '离桩时没清积分量，电量会一直卡在满格')
        self.assertIn('电量已满', d.log.text('info'))

    def test_battery_ratio_prefers_injection(self):
        d = make_dock(IDLE)
        d.sim_ratio = 0.33
        self.assertAlmostEqual(d.battery_ratio(), 0.33)
        d.sim_ratio_stamp = time.monotonic() - 6.0     # 超过 5s
        self.assertIsNone(d.battery_ratio(), '注入过期后应回落到真实电池（此处为 None）')


class TestCancel(unittest.TestCase):

    def test_cancel_really_cancels_goal(self):
        d = make_dock(DOCKING)
        h = GoalHandle()
        d._dock_goal_handle = h
        d.charge_ah = 7.0
        d.busy = True

        d.cancel()

        self.assertEqual(h.cancelled, 1, 'cancel_goal_async 没被调用（假取消）')
        self.assertEqual(d.state, IDLE)
        self.assertFalse(d.busy)
        self.assertIsNone(d.charge_ah, '取消后没清积分量，低电判定会一直用冻结值')
        self.assertIsNone(d._dock_goal_handle)
        self.assertTrue(d.pub_vel.last.is_zero(), '取消时没发零速，可能有残留角速度')

    def test_cancel_without_goal_is_safe(self):
        d = make_dock(IDLE)
        d.cancel()
        self.assertEqual(d.state, IDLE)

    def test_cancel_blocks_immediate_redock(self):
        """真机/仿真实测：电量还低时取消，3.3s 后又会自己开去充电 ——
        手机端「取消回充」等于没用。取消必须进静默期。"""
        d = make_dock(DOCKING)
        d.sim_ratio = 0.10
        d.low_since = time.monotonic() - 10.0
        d.cancel()

        self.assertGreater(d.cooldown_until, time.monotonic() + 100)
        self.assertIsNone(d.low_since, '取消后没清确认计时，会带着旧计时立刻重判')

        d.low_since = time.monotonic() - 10.0   # 就算又被判成低电
        d.tick()
        self.assertEqual(d.dock_calls, [], '取消后仍然自动回充了')

    def test_late_result_after_cancel_is_ignored(self):
        """取消后正在飞的 goal 仍会返回 result（success=False）。
        不按 epoch 丢弃的话会被当成失败 → retries+1 → 进重试冷却。"""
        d = make_dock(DOCKING)
        h = GoalHandle()
        d._dock_goal_handle = h
        d.cancel()
        stale_epoch = d._dock_epoch - 1

        d.on_dock_result(Future(type('R', (), {'result': DockResult(False, 901, 'canceled')})()),
                         stale_epoch)

        self.assertEqual(d.retries, 0, '取消被当成了一次停靠失败')
        self.assertEqual(d.state, IDLE)
        self.assertIn('忽略', d.log.text('info'))

    def test_failure_counts_retry(self):
        d = make_dock(DOCKING)
        d.on_dock_result(
            Future(type('R', (), {'result': DockResult(False, 901, 'aborted')})()),
            d._dock_epoch)
        self.assertEqual(d.retries, 1)
        self.assertEqual(d.state, IDLE)
        self.assertTrue(d.pub_vel.last.is_zero())

    def test_success_enters_docked(self):
        d = make_dock(DOCKING)
        d.sim_ratio = None
        d.charge_ah = None
        d.battery = None
        d.on_dock_result(
            Future(type('R', (), {'result': DockResult(True, num_retries=1)})()),
            d._dock_epoch)
        self.assertEqual(d.state, DOCKED)
        self.assertEqual(d.retries, 0)
        self.assertIsNotNone(d.charge_ah, '进入 DOCKED 没初始化积分量，电量不会涨')


class TestLowBattery(unittest.TestCase):

    def test_needs_confirmation_window(self):
        d = make_dock(IDLE)
        d.sim_ratio = 0.10
        d.tick()
        self.assertEqual(d.dock_calls, [], '第一次读到低电就立刻回充了')
        self.assertIsNotNone(d.low_since)

        d.low_since = time.monotonic() - 4.0        # 已确认 4s > 3s
        d.tick()
        self.assertEqual(d.dock_calls, [False], '确认期过了却没发起回充')

    def test_no_dock_when_auto_off(self):
        d = make_dock(IDLE)
        d.auto_dock = False
        d.sim_ratio = 0.10
        d.low_since = time.monotonic() - 10.0
        d.tick()
        self.assertEqual(d.dock_calls, [])

    def test_retry_cooldown_blocks(self):
        d = make_dock(IDLE)
        d.sim_ratio = 0.10
        d.low_since = time.monotonic() - 10.0
        d.retry_until = time.monotonic() + 30.0
        d.tick()
        self.assertEqual(d.dock_calls, [], '重试冷却期内又发起回充了')


if __name__ == '__main__':
    unittest.main(verbosity=2)
