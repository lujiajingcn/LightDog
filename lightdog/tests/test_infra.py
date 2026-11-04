"""急停命令行（deploy/lightdog_estop.py）里订阅者计数这一层的回归。

I-006 的根因：ROS 的 count_subscribers 会把**发起查询的节点自己**那条订阅
也算进去。estop 控制台为了回读锁存值，自己也订了 /safety/estop —— 于是
`subs == 0` 永远不成立，「当前没有外部订阅者、没人能把车停住」这条告警
就成了死代码。twist_mux 掉线时，工具照样一声不吭。

修复是把「减掉自己建的订阅数」这段纯算术抽成 external_subscribers(total, own)，
不碰 rclpy，供离线直接抽出来测。这个 bug 的隐蔽之处在于：它在**一切正常**
（有人好好听着）时和坏时候（mux 挂了）表现一模一样 —— 都读不出差别，
正因为读不出差别，才必须把算术本身钉死成用例。
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from harness import PKG, extract_funcs  # noqa: E402

REPO = os.path.dirname(PKG)
ESTOP = os.path.join(REPO, 'deploy', 'lightdog_estop.py')


@unittest.skipUnless(os.path.isfile(ESTOP),
                     '找不到 deploy/lightdog_estop.py（容器只挂包目录）')
class TestExternalSubscribers(unittest.TestCase):
    """external_subscribers：从话题订阅总数里减掉本工具自建的，得外部计数。"""

    @classmethod
    def setUpClass(cls):
        # 抽出来的是普通函数；直接挂到类上会被当成未绑定方法、把 self 也传进去。
        # 套 staticmethod 让 self.ext 拿到的就是原始纯函数。
        cls.ext = staticmethod(
            extract_funcs(ESTOP, {'external_subscribers'})['external_subscribers'])

    def test_only_self_subscribed_is_zero(self):
        # 只有控制台自己那条在听：total==own → 外部 0，这正是过去漏报的场景
        self.assertEqual(self.ext(1, 1), 0)

    def test_one_external_subscriber(self):
        # 除自己外还有一个节点（比如 twist_mux）在听
        self.assertEqual(self.ext(2, 1), 1)

    def test_multiple_external_subscribers(self):
        self.assertEqual(self.ext(3, 1), 2)

    def test_no_subscribers_at_all(self):
        self.assertEqual(self.ext(0, 0), 0)

    def test_clamps_below_zero(self):
        # 自建订阅还没被 count 匹配上时 total 会暂小于 own：钳到 0，绝不报负数
        self.assertEqual(self.ext(0, 1), 0)

    def test_accepts_numeric_strings(self):
        # ros2 CLI / 参数偶发以字符串回流，int() 折算后语义不变
        self.assertEqual(self.ext('2', '1'), 1)
        self.assertEqual(self.ext('1', '1'), 0)


if __name__ == '__main__':
    unittest.main(verbosity=2)
