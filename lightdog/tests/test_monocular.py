#!/usr/bin/env python3
"""普通相机路线（单目 / 无深度）的回归。

两条最要紧的不变量：

1) 单目测距 z = H*fx/px_h 里的 H 必须按检测后端选。color 只框橙色上衣
   （0.62m），onnx/yolo 框整个人（1.70m），差 2.74 倍。曾经写死 0.62，
   换后端后把 1.5m 的人算成 0.56m，而 range_min=0.30 会放行 ——
   车一路往前顶，不报任何错。

2) fall_detector 没有深度时**不能静默**。原来 _tick 第一行是
   `if self.depth is None: return`，普通相机下 /fall/state 一条都不发，
   而 systemd 显示 unit 健康。现在必须持续发 unknown。

宿主机没有 ROS，用 harness 从源码里抽出需要的函数直接跑。
"""

import math
import os
import sys
import time
import types
import unittest

import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from harness import bind, extract_funcs, extract_methods, source_of  # noqa: E402

CONFIG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          '..', 'config')

P2_SRC = source_of('lightdog_person_detector.py')
P5_SRC = source_of('lightdog_fall_detector.py')


def load_p2_funcs():
    ns = {'math': math, 'np': np}
    ns.update(extract_funcs(P2_SRC, ['normalize_depth_mode',
                                     'resolve_mono_height'], ns=ns))
    return ns


def load_p5_funcs():
    ns = {}
    ns.update(extract_funcs(P5_SRC, ['normalize_fall_mode'], ns=ns))
    return ns


F2 = load_p2_funcs()
F5 = load_p5_funcs()


class Log(object):
    def __init__(self):
        self.errors = []
        self.warns = []

    def error(self, msg):
        self.errors.append(msg)

    def warn(self, msg):
        self.warns.append(msg)

    def info(self, msg):
        pass


class TestModeNormalization(unittest.TestCase):
    """参数拼错时不能静默走出一个谁也没预料到的模式。"""

    def test_all_three_modes_are_accepted(self):
        for mode in ('auto', 'on', 'off'):
            got, err = F2['normalize_depth_mode'](mode)
            self.assertEqual(got, mode)
            self.assertIsNone(err)

    def test_case_and_whitespace_are_tolerated(self):
        got, err = F2['normalize_depth_mode']('  OFF ')
        self.assertEqual(got, 'off')
        self.assertIsNone(err)

    def test_unknown_mode_falls_back_to_auto_with_a_reason(self):
        got, err = F2['normalize_depth_mode']('yes')
        self.assertEqual(got, 'auto')
        self.assertIn('yes', err)


class TestMonoHeightByBackend(unittest.TestCase):
    """这一组钉住「按后端自动选参考高度」。"""

    def test_color_uses_torso_height(self):
        self.assertAlmostEqual(F2['resolve_mono_height']('color', 0.62, 1.70), 0.62)

    def test_onnx_and_yolo_use_full_body_height(self):
        for backend in ('onnx', 'yolo'):
            self.assertAlmostEqual(
                F2['resolve_mono_height'](backend, 0.62, 1.70), 1.70)

    def test_failed_onnx_load_falls_back_to_color_height(self):
        """模型加载失败会回退到 color，高度必须跟着回退。

        这是最容易漏的一支：若在 __init__ 里按**配置里写的** backend 选高度，
        而实际运行时后端已回退，就会拿 1.70 去配橙色上衣框，距离偏 2.74 倍。
        """
        configured_backend = 'onnx'
        runtime_backend = 'color'          # _load_onnx 返回 None 后
        self.assertNotEqual(configured_backend, runtime_backend)
        self.assertAlmostEqual(
            F2['resolve_mono_height'](runtime_backend, 0.62, 1.70), 0.62)

    def test_invalid_heights_are_rejected(self):
        for bad in (0, -1.0, float('nan'), float('inf'), 'abc', None):
            self.assertIsNone(
                F2['resolve_mono_height']('onnx', 0.62, bad),
                '非法值 %r 应该被拒绝' % (bad,))

    def test_calibrated_height_is_used_verbatim(self):
        """现场按实测身高标定后不能被四舍五入回默认值。"""
        self.assertAlmostEqual(
            F2['resolve_mono_height']('onnx', 0.62, 1.58), 1.58)


class TestFallMode(unittest.TestCase):
    def test_on_and_off(self):
        for mode in ('on', 'off'):
            got, err = F5['normalize_fall_mode'](mode)
            self.assertEqual(got, mode)
            self.assertIsNone(err)

    def test_unknown_falls_back_to_on(self):
        got, err = F5['normalize_fall_mode']('auto')
        self.assertEqual(got, 'on')
        self.assertIn('auto', err)


class TestBBoxToPoint(unittest.TestCase):
    """_bbox_to_point 的三态深度行为。"""

    RGB = np.zeros((480, 640, 3), dtype=np.uint8)
    BBOX = (300, 100, 340, 299)          # px_h = 200

    def _make(self, backend='onnx', mode='auto', depth=None, mono=None, fy=None):
        ns = {'np': np, 'time': time, 'image_to_depth': lambda m: m}
        fns = extract_methods(P2_SRC, 'PersonDetector', ['_bbox_to_point'], ns=ns)
        log = Log()
        obj = types.SimpleNamespace(
            depth_mode=mode,
            depth=depth,
            depth_wall=time.monotonic(),
            depth_timeout=1.0,
            mono_height=mono if mono is not None
            else F2['resolve_mono_height'](backend, 0.62, 1.70),
            fx=615.0, fy=fy if fy is not None else 615.0,
            cx=320.0, cy=240.0,
            cam_off=[-0.065, 0.0, 0.255],
            rng_min=0.30, rng_max=8.00,
            publish_debug=False,
            _warned_depth_required=False,
            get_logger=lambda: log)
        return bind(obj, fns), log

    @staticmethod
    def _z(result):
        """从 (x_base, y_base, z_base, bbox) 反推出用过的距离 z。"""
        return None if result is None else result[0] + 0.065

    def test_off_mode_never_reads_even_a_valid_depth(self):
        d = np.full((480, 640), 1.5, dtype=np.float32)
        obj, _ = self._make(mode='off', depth=d)
        self.assertAlmostEqual(self._z(obj._bbox_to_point(self.RGB, self.BBOX, None)),
                               1.70 * 615.0 / 200.0, places=4)

    def test_color_and_onnx_differ_by_exactly_the_height_ratio(self):
        """核心回归：同样的框，两个后端算出的距离必须差 1.70/0.62 倍。"""
        zs = {}
        for backend in ('color', 'onnx'):
            obj, _ = self._make(backend=backend, mode='off')
            zs[backend] = self._z(obj._bbox_to_point(self.RGB, self.BBOX, None))
        self.assertIsNotNone(zs['color'])
        self.assertIsNotNone(zs['onnx'])
        self.assertAlmostEqual(zs['onnx'] / zs['color'], 1.70 / 0.62, places=5)

    def test_writing_062_for_onnx_would_have_put_us_inside_stop_distance(self):
        """把文档的「常数陷阱」变成可执行判据。

        px_h=200 时，用 0.62 算出的距离约 1.9m；真实 1.5m 处的人用整人框
        会算出不同值。这里直接钉住：color 的高度用到 onnx 上得到的距离，
        必须明显小于用整人身高算出的距离。
        """
        obj, _ = self._make(backend='onnx', mode='off')
        correct = self._z(obj._bbox_to_point(self.RGB, self.BBOX, None))
        obj.mono_height = 0.62
        wrong = self._z(obj._bbox_to_point(self.RGB, self.BBOX, None))
        self.assertLess(wrong, correct * 0.4)
        self.assertGreater(wrong, 0.30)        # range_min 会放行，正是危险所在

    def test_vertical_pixels_use_fy_not_fx(self):
        """px_h 是竖直像素，单目 z 和 y_cam 都必须用 fy。

        真实相机标定出来 fx != fy 是常态；全用 fx 会让跟随距离按 fy/fx
        系统性偏差，而且不报任何错。
        """
        obj, _ = self._make(mode='off', fy=800.0)
        got = obj._bbox_to_point(self.RGB, self.BBOX, None)
        z = 1.70 * 800.0 / 200.0
        self.assertAlmostEqual(self._z(got), z, places=4)
        v_c = 0.5 * (self.BBOX[1] + self.BBOX[3])
        self.assertAlmostEqual(got[2], -(v_c - 240.0) / 800.0 * z + 0.255,
                               places=4)

    def test_depth_wins_when_available(self):
        d = np.full((480, 640), 1.5, dtype=np.float32)
        obj, _ = self._make(mode='auto', depth=d)
        self.assertAlmostEqual(self._z(obj._bbox_to_point(self.RGB, self.BBOX, None)),
                               1.5, places=4)

    def test_auto_without_depth_falls_back_to_mono(self):
        obj, _ = self._make(mode='auto', depth=None)
        self.assertIsNotNone(obj._bbox_to_point(self.RGB, self.BBOX, None))

    def test_on_mode_without_depth_reports_nobody(self):
        """depth_mode=on 是「不许静默降级」的承诺。"""
        obj, log = self._make(mode='on', depth=None)
        self.assertIsNone(obj._bbox_to_point(self.RGB, self.BBOX, None))
        self.assertTrue(any('depth_mode=on' in m for m in log.errors))

    def test_on_mode_reports_only_once_until_depth_recovers(self):
        obj, log = self._make(mode='on', depth=None)
        for _ in range(5):
            obj._bbox_to_point(self.RGB, self.BBOX, None)
        self.assertEqual(len([m for m in log.errors if 'depth_mode=on' in m]), 1)

    def test_on_mode_recovers_and_clears_the_warning(self):
        obj, log = self._make(mode='on', depth=None)
        obj._bbox_to_point(self.RGB, self.BBOX, None)
        obj.depth = np.full((480, 640), 1.5, dtype=np.float32)
        obj.depth_wall = time.monotonic()
        got = obj._bbox_to_point(self.RGB, self.BBOX, None)
        self.assertAlmostEqual(self._z(got), 1.5, places=4)
        self.assertFalse(obj._warned_depth_required)
        # 再断一次应该能重新报出来
        obj.depth = None
        obj._bbox_to_point(self.RGB, self.BBOX, None)
        self.assertEqual(len([m for m in log.errors if 'depth_mode=on' in m]), 2)

    def test_stale_depth_is_not_used_even_in_auto(self):
        d = np.full((480, 640), 1.5, dtype=np.float32)
        obj, _ = self._make(mode='auto', depth=d)
        obj.depth_wall = time.monotonic() - 10.0
        z = self._z(obj._bbox_to_point(self.RGB, self.BBOX, None))
        self.assertAlmostEqual(z, 1.70 * 615.0 / 200.0, places=4)


class TestYamlLiteralSafety(unittest.TestCase):
    """YAML 1.1 把裸写的 on/off/yes/no 当布尔 —— 停用会静默失效。

    实测：`depth_mode: off` 经 yaml.safe_load 得到 False，而不是字符串。
    这条不变量必须有人守，否则「配了 off 但跌倒检测没停」极难排查。
    """

    def test_yaml_really_does_parse_bare_off_as_false(self):
        self.assertIs(yaml.safe_load('depth_mode: off')['depth_mode'], False)
        self.assertEqual(yaml.safe_load("depth_mode: 'off'")['depth_mode'], 'off')

    def test_both_normalizers_reject_bool_with_an_actionable_hint(self):
        for fn in (F2['normalize_depth_mode'], F5['normalize_fall_mode']):
            for raw in (True, False):
                mode, err = fn(raw)
                self.assertIn('引号', err)
                self.assertIn('YAML', err)
                # 关键：裸写 off（False）绝不能被反转成「启用」。
                if fn is F5['normalize_fall_mode']:
                    self.assertEqual(mode, 'on' if raw else 'off')
                else:
                    self.assertEqual(mode, 'auto')

    def test_checked_in_configs_have_string_depth_modes(self):
        for name in ('p2_params.yaml', 'p5_params.yaml'):
            with open(os.path.join(CONFIG_DIR, name), encoding='utf-8') as f:
                data = yaml.safe_load(f)
            for node, body in data.items():
                params = (body or {}).get('ros__parameters') or {}
                if 'depth_mode' not in params:
                    continue
                self.assertIsInstance(
                    params['depth_mode'], str,
                    '%s 的 %s.depth_mode 必须是字符串（值要加引号）'
                    % (name, node))

    def test_quoted_off_survives_yaml_and_is_accepted_as_off(self):
        raw = yaml.safe_load("depth_mode: 'off'")['depth_mode']
        self.assertEqual(F5['normalize_fall_mode'](raw)[0], 'off')


class TestFallTickNeverGoesSilent(unittest.TestCase):
    """钉住刚修掉的真 bug：无深度时 /fall/state 一条都不发。"""

    def _make(self, mode, depth=None):
        ns = {'np': np, 'time': time}
        fns = extract_methods(P5_SRC, 'FallDetector', ['_tick'], ns=ns)
        log = Log()
        published = []
        obj = types.SimpleNamespace(
            depth_mode=mode, depth=depth, rgb=None, fx=None,
            _warned_no_depth=False, _emit_unknown=lambda: published.append('unknown'),
            get_logger=lambda: log)
        return bind(obj, fns), published, log

    def test_off_mode_publishes_unknown(self):
        obj, pub, _ = self._make('off')
        obj._tick()
        self.assertEqual(pub, ['unknown'])

    def test_off_mode_keeps_publishing_unknown(self):
        obj, pub, _ = self._make('off')
        for _ in range(3):
            obj._tick()
        self.assertEqual(pub, ['unknown'] * 3)

    def test_on_mode_without_depth_publishes_unknown_instead_of_silence(self):
        """改动前这里 pub 是空的 —— unit 健康、状态全无。"""
        obj, pub, log = self._make('on', depth=None)
        obj._tick()
        self.assertEqual(pub, ['unknown'])
        self.assertTrue(any('depth_mode=on' in m for m in log.errors))

    def test_on_mode_depth_warning_is_not_repeated_every_tick(self):
        obj, _, log = self._make('on', depth=None)
        for _ in range(5):
            obj._tick()
        self.assertEqual(len([m for m in log.errors if 'depth_mode=on' in m]), 1)


if __name__ == '__main__':
    unittest.main(verbosity=2)
