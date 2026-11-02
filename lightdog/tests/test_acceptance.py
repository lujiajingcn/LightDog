"""真机验收脚本（deploy/acceptance.py）里**判据**这一层的回归。

acceptance.py 整体要 rclpy + 真机才能跑，但它的判定逻辑是纯逻辑：
`zero_cmd` 只依赖 self.cmd / self.cmd_at 和 rclpy.spin_once。
所以照样能用 harness 把方法从源码里抽出来，注入一个假 rclpy 直接测。

为什么专门盯这一条：它曾经是假绿 —— 一条速度消息都没收到时直接 return True。
而 `zero_cmd` 是 check_1 的**安全前置**，check_1 接下来要 kill 掉 twist_mux。
也就是说：脚本在完全没有观测到任何东西的情况下，宣称「车是停着的」，
然后去拆掉速度仲裁。这正是本仓库反复强调的「测不到的事不报 PASS」。

这类 bug 的共同点是**只在缺数据时出现**，而缺数据是最难被注意到的状态：
不抛异常、不打日志，只是安静地给出一份「全部通过」。
"""

import os
import sys
import time
import types
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from harness import PKG, extract_methods  # noqa: E402

REPO = os.path.dirname(PKG)
ACCEPTANCE = os.path.join(REPO, 'deploy', 'acceptance.py')


class _V:
    def __init__(self, **kw):
        self.x = self.y = self.z = 0.0
        for k, v in kw.items():
            setattr(self, k, v)


class _Twist:
    def __init__(self, vx=0.0, wz=0.0):
        self.linear = _V(x=vx)
        self.angular = _V(z=wz)


@unittest.skipUnless(os.path.isfile(ACCEPTANCE),
                     '找不到 deploy/acceptance.py（容器只挂包目录）')
class TestZeroCmd(unittest.TestCase):
    """zero_cmd：true 只能来自「真的收到了零速消息」。"""

    def _call(self, mode, need=0.08, min_samples=3):
        """mode: none / one / always / moving。返回 (结果, 探针对象)。"""
        obj = types.SimpleNamespace(cmd=None, cmd_at=None, cmd_samples=0)
        n = [0]

        def spin_once(node, timeout_sec=0.02):
            n[0] += 1
            if mode == 'none':
                return
            if mode == 'one' and n[0] > 1:
                return
            obj.cmd = _Twist(vx=0.2 if mode == 'moving' else 0.0)
            # 每条消息给一个不同的时间戳（用计数代替 monotonic，
            # 免得低分辨率时钟把两条消息压成同一个值）
            obj.cmd_at = n[0]

        shim = types.SimpleNamespace(spin_once=spin_once)
        fn = extract_methods(ACCEPTANCE, 'Probe', ['zero_cmd'],
                             ns={'rclpy': shim, 'time': time})['zero_cmd']
        obj.zero_cmd = types.MethodType(fn, obj)
        return obj.zero_cmd(need=need, min_samples=min_samples), obj

    def test_no_message_at_all_is_not_a_pass(self):
        """一条消息都没有 -> 必须是 False，且计数为 0。

        旧写法在这里 return True，于是 check_1 会拿着「没数据」去 kill
        twist_mux。这是本条测试存在的全部理由。
        """
        res, obj = self._call('none')
        self.assertFalse(res, '没有任何速度消息却判了通过 —— 假绿')
        self.assertEqual(obj.cmd_samples, 0)

    def test_one_single_message_is_not_enough(self):
        """只收到 1 条：证据不足，不能算「稳定零速」。"""
        res, obj = self._call('one', min_samples=3)
        self.assertFalse(res)
        self.assertEqual(obj.cmd_samples, 1,
                         '一条消息被数成了多条（不能拿 cmd is not None 计数）')

    def test_one_message_is_enough_if_asked_for(self):
        """min_samples=1 时同一条消息应当够用 —— 证明阈值是它在起作用。"""
        res, obj = self._call('one', min_samples=1)
        self.assertTrue(res)
        self.assertEqual(obj.cmd_samples, 1)

    def test_steady_zero_with_many_messages_passes(self):
        res, obj = self._call('always')
        self.assertTrue(res)
        self.assertGreaterEqual(obj.cmd_samples, 3)

    def test_moving_robot_fails_immediately(self):
        res, obj = self._call('moving')
        self.assertFalse(res, '车在动却判了零速')


if __name__ == '__main__':
    unittest.main(verbosity=2)
