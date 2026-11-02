"""跑全部离线回归。

    python3 lightdog/tests/run.py          # 摘要
    python3 lightdog/tests/run.py -v       # 每个用例一行

依赖：标准库 + PyYAML（缺了会自动跳过参数一致性那组）。
不需要 ROS / Gazebo / rclpy —— 被测逻辑是纯函数级的，见 harness.py 的说明。

这套东西**不能**替代容器里的 P2/P3 验收：它验的是「数据该怎么变」，
不验「Gazebo 里车实际走了多远」。容器里跑时用挂载进来的源码目录：

    docker exec sim python3 /home/ros/linorobot2_ws/src/lightdog/tests/run.py

需要仿真在跑的端到端验收不在这里，而是同目录的 `verify_p4.py` / `verify_p5.py`
/ `verify_p5_extra.py`（故意用 `verify_` 前缀，不被 unittest 自动收集）。
"""

import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

GROUPS = [
    ('test_p2', 'P2 检测 / 跟随 / 仲裁'),
    ('test_p3', 'P3 回充状态机'),
    ('test_robustness', '真机健壮性：外部输入 / 传感器失效 / 充电确认'),
    ('test_system', '系统层：模式仲裁 / 语音语义 / 急停闸'),
    ('test_consistency', '参数与版本一致性'),
]


def main():
    verbose = '-v' in sys.argv or '--verbose' in sys.argv
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()
    for mod, _ in GROUPS:
        suite.addTests(loader.loadTestsFromName(mod))

    result = unittest.TextTestRunner(verbosity=2 if verbose else 1).run(suite)

    total = result.testsRun
    bad = len(result.failures) + len(result.errors)
    skipped = len(result.skipped)
    print()
    print('=' * 58)
    for mod, desc in GROUPS:
        print(f'  {mod:<20} {desc}')
    print('-' * 58)
    print(f'  共 {total} 项：通过 {total - bad - skipped}，'
          f'失败 {bad}，跳过 {skipped}')
    print('=' * 58)
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
