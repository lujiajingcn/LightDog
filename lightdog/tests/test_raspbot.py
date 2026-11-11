"""RASPBOT V2 实车适配回归（离线，不需要 ROS / 串口 / 相机）。

车到了之后，软件里最容易「看起来没问题、上山才爆」的几处：
  1. 四麦轮逆解与等比缩放（截断单轮会悄悄改方向）
  2. 开环里程计的积分（车上没有编码器）
  3. 停发速度 / 急停必须归零（车上没有 IWDG）
  4. 商家串口协议未确认时一个字节都不能发
  5. 商家的 AprilTag 字典不靠猜，且不能误锁
  6. 可动云台下静态外参必须显式拒绝发位姿
  7. 真机配置与 nodes.conf 真的切过去了，而不是只写了文档

被测逻辑一律用 harness 从源码里原样抽取，不在测试里复制实现。
"""

import math
import os
import sys
import time
import types
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from harness import (PKG, Log, Msg, Pub, bind, extract_const,  # noqa: E402
                     extract_funcs, extract_methods, source_of)

try:
    import yaml
except ImportError:                                    # pragma: no cover
    yaml = None

REPO = os.path.dirname(PKG)
CONFIG = os.path.join(PKG, 'config')

RB = source_of('lightdog_raspbot_driver.py')
ARUCO = source_of('lightdog_aruco_dock.py')

RB_FUNCS = extract_funcs(RB, [
    'clamp_speed', 'mecanum_wheel_speeds', 'normalize_wheel_speeds',
    'integrate_openloop', 'cmd_to_send'], ns={'math': math})

ARUCO_FUNCS = extract_funcs(
    ARUCO, ['resolve_dictionary_names'],
    ns={'AUTO_DICTS': extract_const(ARUCO, 'AUTO_DICTS')})


def _load(name):
    with open(os.path.join(CONFIG, name), encoding='utf-8') as f:
        return yaml.safe_load(f) or {}


def _rows():
    """nodes.conf 的 9 列数据行。"""
    out = []
    with open(os.path.join(REPO, 'deploy', 'nodes.conf'), encoding='utf-8') as f:
        for ln in f:
            ln = ln.strip()
            if not ln or ln.startswith('#'):
                continue
            cols = ln.split('|')
            if len(cols) == 9:
                out.append(cols)
    return out


class _FakeAruco:
    def __init__(self, results):
        self.results = results
        self.calls = []

    def detectMarkers(self, gray, dictionary, parameters=None):
        self.calls.append(str(dictionary))
        return self.results.get(str(dictionary), (None, None, []))


class _FakeCv2:
    def __init__(self, results):
        self.aruco = _FakeAruco(results)


class TestMecanumKinematics(unittest.TestCase):
    """四麦轮：逆解 + 正解自洽，超限时等比缩放。"""

    K = 0.0293 + 0.04145       # half_x + half_y（实车尺寸图初值）

    @staticmethod
    def _forward(m, k):
        """标准 X 型麦轮正解，用来验证逆解没有写反符号。"""
        return ((m[0] + m[1] + m[2] + m[3]) / 4.0,
                (-m[0] + m[1] + m[2] - m[3]) / 4.0,
                (-m[0] + m[1] - m[2] + m[3]) / (4.0 * k))

    def test_inverse_then_forward_is_identity(self):
        f = RB_FUNCS['mecanum_wheel_speeds']
        for vx, vy, wz in ((0.2, 0.0, 0.0), (0.0, 0.15, 0.0),
                           (0.0, 0.0, 0.5), (0.18, -0.12, 0.35),
                           (-0.1, 0.07, -0.4)):
            got = self._forward(f(vx, vy, wz, 0.0293, 0.04145), self.K)
            for a, b in zip((vx, vy, wz), got):
                self.assertAlmostEqual(a, b, places=6,
                                       msg=f'逆解/正解不自洽：{vx},{vy},{wz} -> {got}')

    def test_pure_translations_have_expected_signs(self):
        f = RB_FUNCS['mecanum_wheel_speeds']
        self.assertTrue(all(v > 0 for v in f(0.2, 0.0, 0.0, 0.0293, 0.04145)),
                        '纯前进四轮应同向')
        m = f(0.0, 0.2, 0.0, 0.0293, 0.04145)
        self.assertLess(m[0], 0)
        self.assertGreater(m[1], 0)
        self.assertGreater(m[2], 0)
        self.assertLess(m[3], 0)
        m = f(0.0, 0.0, 0.5, 0.0293, 0.04145)
        self.assertEqual([v > 0 for v in m], [False, True, False, True])

    def test_normalize_scales_all_wheels_evenly(self):
        norm = RB_FUNCS['normalize_wheel_speeds']
        src = (0.9, 0.45, -0.6, 0.3)
        got = norm(src, 0.3)
        self.assertLessEqual(max(abs(v) for v in got), 0.3 + 1e-9)
        ratios = [a / b for a, b in zip(got, src)]
        for r in ratios:
            self.assertAlmostEqual(r, ratios[0], places=9,
                                   msg='必须等比缩放，单轮截断会改变运动方向')

    def test_normalize_leaves_small_speeds_untouched(self):
        norm = RB_FUNCS['normalize_wheel_speeds']
        src = (0.1, -0.2, 0.05, 0.0)
        self.assertEqual(norm(src, 0.35), src)
        self.assertEqual(norm(src, 0.0), src)


class TestOpenloopOdomAndSafety(unittest.TestCase):

    def test_straight_and_lateral_integration(self):
        f = RB_FUNCS['integrate_openloop']
        x, y, yaw = f(0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 1.0)
        self.assertAlmostEqual(x, 0.2)
        self.assertAlmostEqual(y, 0.0)
        x, y, yaw = f(0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 1.0)
        self.assertAlmostEqual(y, 0.1)

    def test_turn_uses_midpoint_heading(self):
        f = RB_FUNCS['integrate_openloop']
        x, y, yaw = f(0.0, 0.0, 0.0, 0.2, 0.0, math.pi, 0.5)
        # 半拍转 90°，中点航向 45°：位移应同时进 x 和 y。
        self.assertAlmostEqual(yaw, math.pi / 2.0)
        self.assertAlmostEqual(x, 0.2 * math.cos(math.pi / 4) * 0.5)
        self.assertAlmostEqual(y, 0.2 * math.sin(math.pi / 4) * 0.5)

    def test_zero_dt_is_noop(self):
        f = RB_FUNCS['integrate_openloop']
        self.assertEqual(f(1.0, 2.0, 0.3, 0.2, 0.1, 0.5, 0.0),
                         (1.0, 2.0, 0.3))

    def test_estop_and_stale_cmd_are_zero(self):
        f = RB_FUNCS['cmd_to_send']
        self.assertEqual(f(0.2, 0.1, 0.5, True, 10.0, 9.9, 0.3), (0.0, 0.0, 0.0))
        self.assertEqual(f(0.2, 0.1, 0.5, False, 10.0, 9.0, 0.3), (0.0, 0.0, 0.0))
        self.assertEqual(f(0.2, 0.1, 0.5, False, 10.0, 9.9, 0.3), (0.2, 0.1, 0.5))

    def test_nan_is_clamped_to_zero(self):
        f = RB_FUNCS['clamp_speed']
        self.assertEqual(f(float('nan'), -1.0, 1.0), 0.0)
        self.assertEqual(f(5.0, -1.0, 1.0), 1.0)


class TestAprilTagDictionaries(unittest.TestCase):

    def test_auto_includes_all_four_apriltag_families(self):
        names = ARUCO_FUNCS['resolve_dictionary_names']('auto')
        for n in ('DICT_APRILTAG_16H5', 'DICT_APRILTAG_25H9',
                  'DICT_APRILTAG_36H10', 'DICT_APRILTAG_36H11'):
            self.assertIn(n, names)
        self.assertIn('DICT_4X4_50', names)

    def test_explicit_list_and_empty_fallback(self):
        f = ARUCO_FUNCS['resolve_dictionary_names']
        self.assertEqual(f('DICT_APRILTAG_36H11, DICT_4X4_50'),
                         ['DICT_APRILTAG_36H11', 'DICT_4X4_50'])
        self.assertEqual(f('  AUTO '), f('auto'))
        self.assertEqual(f(None), f('auto'))

    def _detector(self, results, marker_id=0, first='DICT_4X4_50'):
        cv2 = _FakeCv2(results)
        methods = extract_methods(ARUCO, 'ArucoDockDetector',
                                  ['_detect_markers'], ns={'cv2': cv2})
        log = Log()
        obj = types.SimpleNamespace(
            dictionaries=[(n, n, None) for n in results],
            detector_params=None,
            marker_id=marker_id, active_dict=first,
            get_logger=lambda: log)
        bind(obj, methods)
        return obj, cv2, log

    def test_locks_only_the_dictionary_containing_target_id(self):
        results = {
            'DICT_4X4_50': (['a'], np.array([7]), []),
            'DICT_APRILTAG_36H11': (['b'], np.array([0]), []),
        }
        obj, cv2, log = self._detector(results)
        corners, ids, _ = obj._detect_markers('gray')
        self.assertEqual(corners, ['b'], '别的字典检出非目标 id 不能误锁')
        self.assertEqual(int(ids.flatten()[0]), 0)
        self.assertEqual(obj.active_dict, 'DICT_APRILTAG_36H11')
        self.assertIn('自动命中', log.text('info'))
        self.assertEqual(cv2.aruco.calls,
                         ['DICT_4X4_50', 'DICT_APRILTAG_36H11'])

    def test_no_dictionary_matches_returns_last_empty_result(self):
        results = {
            'DICT_4X4_50': (['a'], np.array([3]), []),
            'DICT_APRILTAG_36H11': (['b'], np.array([9]), []),
        }
        obj, cv2, _ = self._detector(results)
        corners, ids, _ = obj._detect_markers('gray')
        self.assertEqual(corners, ['b'])
        self.assertEqual(obj.active_dict, 'DICT_4X4_50')


class TestGimbalGuard(unittest.TestCase):

    def test_unlocked_gimbal_refuses_to_publish_pose(self):
        cv2 = _FakeCv2({})
        methods = extract_methods(
            ARUCO, 'ArucoDockDetector', ['_tick_impl'],
            ns={'time': time, 'math': math, 'np': np, 'cv2': cv2,
                'Bool': Msg})
        log = Log()
        vis = Pub()
        obj = types.SimpleNamespace(gimbal_locked=False, visible=True,
                                    vis_pub=vis, _warned_gimbal=False,
                                    get_logger=lambda: log,
                                    latest=None, camera_matrix=None)
        bind(obj, methods)
        obj._tick_impl()
        self.assertFalse(obj.visible)
        self.assertIsNotNone(vis.last)
        self.assertFalse(vis.last.data)
        self.assertIn('gimbal_locked=false', log.text('error'))
        self.assertTrue(obj._warned_gimbal)


@unittest.skipIf(yaml is None, '宿主机没有 PyYAML')
class TestRealConfigIsActuallySwitched(unittest.TestCase):

    PAIRS = ('p2_params.yaml', 'p3_params.yaml', 'p4_params.yaml',
             'p5_params.yaml', 'system_params.yaml')

    def test_every_real_config_exists_and_has_same_param_keys(self):
        for name in self.PAIRS:
            sim, real = _load(name), _load('real_' + name)
            self.assertEqual(set(sim), set(real), f'real_{name} 节点节不一致')
            for key in sim:
                a = set((sim[key] or {}).get('ros__parameters') or {})
                b = set((real[key] or {}).get('ros__parameters') or {})
                self.assertEqual(a, b, f'real_{name} 的 {key} 参数集不一致')

    def test_real_configs_use_wall_clock(self):
        for name in self.PAIRS:
            data = _load('real_' + name)
            for node, body in data.items():
                params = (body or {}).get('ros__parameters') or {}
                if 'use_sim_time' in params:
                    self.assertIs(params['use_sim_time'], False,
                                  f'real_{name}: {node}.use_sim_time 不是 false')

    def test_depth_modes_are_quoted_off(self):
        for name, node in (('real_p2_params.yaml', '/lightdog_person_detector'),
                           ('real_p5_params.yaml', '/lightdog_fall_detector')):
            v = _load(name)[node]['ros__parameters']['depth_mode']
            self.assertIsInstance(v, str, 'depth_mode 必须是字符串（YAML 1.1 裸 off 会变 bool）')
            self.assertEqual(v, 'off')

    def test_backend_and_apriltag_and_gimbal(self):
        p2 = _load('real_p2_params.yaml')['/lightdog_person_detector']['ros__parameters']
        p3 = _load('real_p3_params.yaml')['/lightdog_aruco_dock']['ros__parameters']
        p4 = _load('real_p4_params.yaml')['/lightdog_server']['ros__parameters']
        self.assertEqual(p2['backend'], 'onnx')
        self.assertEqual(p3['dictionary'], 'auto')
        self.assertIs(p3['gimbal_locked'], True)
        self.assertNotEqual(p4['auth_token'], '', 'auth_token 不能留空（同 WiFi 裸奔）')

    def test_raspbot_driver_defaults_are_safe(self):
        p = _load('raspbot_params.yaml')['/lightdog_raspbot_driver']['ros__parameters']
        self.assertEqual(p['driver_backend'], 'vendor_topic')
        self.assertIs(p['protocol_confirmed'], False)
        self.assertEqual(p['odom_source'], 'openloop')
        self.assertEqual(p['battery_source'], 'none')
        self.assertIs(p['button_latching'], True)
        self.assertGreater(p['cmd_hold'], 0.0)

    def test_nodes_conf_runs_real_driver_not_stm32_bridge(self):
        rows = {r[0]: r for r in _rows()}
        self.assertIn('raspbot', rows)
        r = rows['raspbot']
        self.assertEqual(r[2], 'lightdog_raspbot_driver.py')
        self.assertEqual(r[3], 'raspbot_params.yaml')
        self.assertEqual(r[4], 'yes')
        self.assertEqual(rows['mcu'][4], 'no', '实车没有 STM32，mcu 单元必须停用')
        for name, row in rows.items():
            if row[4] == 'yes' and name != 'raspbot':
                self.assertTrue(row[3].startswith('real_'),
                                f'{name} 还在用仿真配置 {row[3]}')

    def test_every_params_file_in_nodes_conf_exists(self):
        # install.sh:173 会 die，所以它检查的文件必须真的装进 config/。
        for row in _rows():
            self.assertTrue(os.path.isfile(os.path.join(CONFIG, row[3])),
                            f'{row[0]} 指向不存在的参数文件 {row[3]}')

    def test_cmake_installs_the_new_node(self):
        with open(os.path.join(PKG, 'CMakeLists.txt'), encoding='utf-8') as f:
            self.assertIn('lightdog_raspbot_driver.py', f.read())


class TestSafetyInvariants(unittest.TestCase):
    """静态不变量：这几行一旦被删，行为测试不一定抓得到。"""

    def setUp(self):
        with open(RB, encoding='utf-8') as f:
            self.src = f.read()

    def test_unconfirmed_serial_protocol_forces_self_test(self):
        self.assertIn("if self.backend == 'direct_serial' and not self.protocol_confirmed:",
                      self.src)
        self.assertIn('self.backend = \'self_test\'\n            self.protocol_blocked = True',
                      self.src)

    def test_vendor_odom_topic_cannot_loop_back(self):
        self.assertIn('self.vendor_odom_topic == self.odom_topic', self.src)
        self.assertIn("'vendor_odom_topic 不能等于 odom_topic，改用 openloop'", self.src)

    def test_battery_never_filled_with_zero(self):
        self.assertIn("self._publish_battery(float('nan'), float('nan'), float('nan'))",
                      self.src)
        self.assertIn('绝不能填 0', self.src)

    def test_shutdown_sends_zero_speed(self):
        self.assertIn('self._drv_send(0.0, 0.0, 0.0)', self.src)


if __name__ == '__main__':
    unittest.main()
