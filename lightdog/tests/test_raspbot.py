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

from harness import (PKG, Log, Msg, Pub, Twist, bind,  # noqa: E402
                     extract_const, extract_funcs, extract_methods,
                     source_of)

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
    'integrate_openloop', 'cmd_to_send', 'parse_motor_id_map',
    'wheel_speeds_to_pwm', 'vendor_twist_to_normalized',
    'servo_lock_angles', 'read_key1'], ns={
        'math': math,
        'VENDOR_LINEAR_LIMIT': extract_const(RB, 'VENDOR_LINEAR_LIMIT'),
        'VENDOR_ANGULAR_LIMIT': extract_const(RB, 'VENDOR_ANGULAR_LIMIT'),
        'SERVO_S1_RANGE': extract_const(RB, 'SERVO_S1_RANGE'),
        'SERVO_S2_RANGE': extract_const(RB, 'SERVO_S2_RANGE'),
        'KEY1_ADDRESS': extract_const(RB, 'KEY1_ADDRESS')})

ARUCO_FUNCS = extract_funcs(
    ARUCO, ['resolve_dictionary_names'],
    ns={'AUTO_DICTS': extract_const(ARUCO, 'AUTO_DICTS')})

# vendor_twist_to_normalized 在源码里调用同文件的 clamp_speed；harness 把每个
# 函数抽到独立 globals，这里显式补上依赖（不在测试里复制实现）。
RB_FUNCS['vendor_twist_to_normalized'].__globals__['clamp_speed'] = \
    RB_FUNCS['clamp_speed']


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


class TestOfficialI2CBackend(unittest.TestCase):
    """官方 Raspbot_Lib 是 I2C + 开环 PWM；底层编号 0-3，轮序可推导但仍须实测。"""

    def test_motor_id_map_requires_four_distinct_integers(self):
        f = RB_FUNCS['parse_motor_id_map']
        self.assertEqual(f('0,1,2,3'), (0, 1, 2, 3))
        self.assertEqual(f(' 1, 2, 3, 4 '), (1, 2, 3, 4))
        for bad in ('', None, '0,1,2', '0,1,2,2', 'a,1,2,3', '-1,1,2,3',
                    '9,1,2,3', '0,1,2,3,4'):
            self.assertIsNone(f(bad), f'{bad!r} 不该被当成合法轮序')

    def test_wheel_speeds_map_to_signed_pwm(self):
        f = RB_FUNCS['wheel_speeds_to_pwm']
        self.assertEqual(f((0.35, 0.0, -0.175, 0.35), 0.35, 255.0),
                         (255, 0, -128, 255))
        self.assertEqual(f((0.0, 0.0, 0.0, 0.0), 0.35, 255.0), (0, 0, 0, 0))

    def test_pwm_is_clamped_and_nan_is_zero(self):
        f = RB_FUNCS['wheel_speeds_to_pwm']
        got = f((9.9, -9.9, float('nan'), 0.1), 0.35, 255.0)
        self.assertEqual(got[:2], (255, -255))
        self.assertEqual(got[2], 0, 'NaN 绝不能变成电机指令')

    def test_fractional_pwm_scale_stays_within_official_range(self):
        # Ctrl_Muto 只接受 -255..255。速度钳位后 pwm = 1.0 * 255.5 -> round 256，
        # 所以最后那道整数钳位不是死代码：非整数 motor_pwm_scale 会踩到。
        f = RB_FUNCS['wheel_speeds_to_pwm']
        got = f((0.35, -0.35, 0.0, 0.0), 0.35, 255.5)
        self.assertEqual(got[:2], (255, -255))
        self.assertTrue(all(-255 <= v <= 255 for v in got))

    def test_invalid_scale_sends_all_zero(self):
        f = RB_FUNCS['wheel_speeds_to_pwm']
        for bad in ((0.0, 255.0), (0.35, 0.0), (-0.35, 255.0)):
            self.assertEqual(f((0.2, 0.1, 0.3, 0.4), *bad), (0, 0, 0, 0))

    def test_official_api_names_and_no_encoder_read_are_pinned(self):
        with open(RB, encoding='utf-8') as f:
            src = f.read()
        self.assertIn('Ctrl_Muto', src)
        self.assertIn('Ctrl_WQ2812_brightness_ALL', src)
        self.assertIn("'vendor_i2c'", src)
        self.assertIn("parse_motor_id_map(p('motor_id_map').value)", src)

    def test_official_motor_id_map_is_pinned_and_still_disabled_by_default(self):
        # 协议表 0/1/2/3=L1/L2/R1/R2 + URDF 教程 l1/l2/r1/r2=左前/左后/右前/右后
        # + 扩展板接口图 M1..M4=左上/左下/右上/右下 + 麦轮符号表 -> '0,2,1,3'。
        self.assertEqual(tuple(extract_const(RB, 'OFFICIAL_MOTOR_ID_MAP')),
                         (0, 2, 1, 3))
        p = _load('raspbot_params.yaml')['/lightdog_raspbot_driver']['ros__parameters']
        self.assertEqual(p['motor_id_map'], '',
                         '推导值可以写进注释，但没上机四轮单动验证前不许启用')
        with open(RB, encoding='utf-8') as fh:
            src = fh.read()
        self.assertIn('教程 PDF', src)
        self.assertIn('固件真正吃的是 0-3', src,
                      '官方教程 PDF 的 1-4 是丝印，源码里必须写明这个矛盾')


class TestAprilTagDictionaries(unittest.TestCase):

    def test_auto_includes_all_four_apriltag_families(self):
        names = ARUCO_FUNCS['resolve_dictionary_names']('auto')
        for n in ('DICT_APRILTAG_16H5', 'DICT_APRILTAG_25H9',
                  'DICT_APRILTAG_36H10', 'DICT_APRILTAG_36H11'):
            self.assertIn(n, names)
        self.assertIn('DICT_4X4_50', names)

    def test_official_tag36h11_is_tried_first(self):
        # 官方《09、AprilTag标签码识别》：出厂配套标签是 TAG36H11，源码
        # Detector(families='tag36h11')。放首位能省掉前四次全图 detect，
        # 其余字典仍保留兜底（用户可能自己打印别的族）。
        names = ARUCO_FUNCS['resolve_dictionary_names']('auto')
        self.assertEqual(names[0], 'DICT_APRILTAG_36H11')
        self.assertEqual(len(names), 5)
        self.assertEqual(len(set(names)), 5)

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
        self.assertEqual(p['driver_backend'], 'vendor_i2c',
                         '默认走官方 Raspbot_Lib I2C；官方另有 Docker 内的 '
                         '/driver_node，切 vendor_topic 前要先确认 ROS 域互通')
        self.assertEqual(p['motor_id_map'], '',
                         '电机编号/轮序未实测时必须是空串，节点才会强制不动')
        self.assertEqual(p['led_backend'], 'i2c')
        self.assertIs(p['protocol_confirmed'], False)
        self.assertEqual(p['odom_source'], 'openloop')
        self.assertEqual(p['battery_source'], 'none')
        self.assertIs(p['button_latching'], True)
        self.assertEqual(p['button_backend'], 'i2c')
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

    def test_unmapped_motors_force_self_test(self):
        self.assertIn("if self.backend == 'vendor_i2c' and self.motor_id_map is None:",
                      self.src)
        self.assertIn("self.block_reason = 'motor_id_map 未实测'", self.src)

    def test_vendor_topic_warns_when_nobody_subscribes(self):
        self.assertIn('get_subscription_count()', self.src)
        self.assertIn('ros2 launch yahboomcar_bringup', self.src)
        self.assertIn('/driver_node', self.src)

    def test_i2c_failure_reasserts_zero_speed(self):
        self.assertIn('I2C 电机下发失败，立刻补发全零', self.src)
        self.assertIn('self.bot.Ctrl_Muto(mid, 0)', self.src)

    def test_official_lib_swallows_i2c_errors_so_probe_is_mandatory(self):
        # 官方 Raspbot_Lib 的 write_array 是裸 except + print，_send_i2c 的
        # try/except 抓不到总线掉线，frames_sent 会一直涨而车根本没动。
        self.assertIn('probe = read_key1(self.bot, self.key1_address)', self.src)
        self.assertIn('KEY1 读探针返回 None', self.src)
        self.assertIn('frames_sent_means', self.src)
        self.assertIn('吞掉 I2C 异常', self.src)

    def test_button_i2c_survives_chassis_backend_degradation(self):
        self.assertIn('self.button_bot = self.bot', self.src)
        self.assertIn('底盘后端降级不影响急停按钮', self.src)
        self.assertIn('出厂镜像无人发布它', self.src)

    def test_consecutive_i2c_read_failures_degrade_and_zero(self):
        self.assertIn('self.i2c_read_failures >= self.i2c_max_read_failures',
                      self.src)
        self.assertIn("self.block_reason = 'I2C 链路连续读失败'", self.src)
        self.assertIn('官方库的限位等于没生效', self.src,
                      '官方 Ctrl_Servo 先构造 data 再 clamp，我们侧的限位不能删')

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


class TestOfficialROS2Chassis(unittest.TestCase):
    """官方配套教程 PDF 的一手契约（docs/hardware_arrival.md §十）。"""

    # 官方 04.Car motion control course/0.Mecanum wheel motion principle.pdf
    # 第 5 页的符号表，列顺序是 L1 L2 R1 R2；URDF 教程第 2 页明确
    # l1=左前、l2=左后、r1=右前、r2=右后。我们的返回顺序是 FL,FR,RL,RR。
    OFFICIAL_TABLE = (
        ('前进',            (1.0, 0.0, 0.0),  '++++'),
        ('后退',            (-1.0, 0.0, 0.0), '----'),
        ('左平移',          (0.0, 1.0, 0.0),  '-++-'),
        ('右平移',          (0.0, -1.0, 0.0), '+--+'),
        ('左前',            (1.0, 1.0, 0.0),  '0++0'),
        ('左后',            (-1.0, 1.0, 0.0), '-00-'),
        ('右前',            (1.0, -1.0, 0.0), '+00+'),
        ('右后',            (-1.0, -1.0, 0.0), '0--0'),
        ('逆时针(左转)',     (0.0, 0.0, 1.0),  '--++'),
        ('顺时针(右转)',     (0.0, 0.0, -1.0), '++--'),
        ('停止',            (0.0, 0.0, 0.0),  '0000'),
    )

    @staticmethod
    def _sign(v, eps=1e-9):
        return '0' if abs(v) < eps else ('+' if v > 0 else '-')

    def test_inverse_matches_official_mecanum_sign_table(self):
        f = RB_FUNCS['mecanum_wheel_speeds']
        for label, (vx, vy, wz), expect in self.OFFICIAL_TABLE:
            fl, fr, rl, rr = f(vx, vy, wz, 0.0586, 0.0829)
            # L1 L2 R1 R2 = FL RL FR RR
            got = ''.join(self._sign(v) for v in (fl, rl, fr, rr))
            self.assertEqual(got, expect,
                             f'官方运动「{label}」符号不符：得到 {got}，应为 {expect}')

    def test_official_motor_id_map_reorders_inverse_to_symbol_table(self):
        """端到端：按 '0,2,1,3' 把逆解摆回 L1,L2,R1,R2，逐行对官方符号表。

        这是「轮序推导」能不能真的拿去填 motor_id_map 的判据，而不只是注释
        里写一句。若映射改成 0,1,2,3，左平移/自转这几行会立刻对不上。
        """
        m = tuple(extract_const(RB, 'OFFICIAL_MOTOR_ID_MAP'))
        self.assertEqual(m, (0, 2, 1, 3))
        f = RB_FUNCS['mecanum_wheel_speeds']
        for label, (vx, vy, wz), expect in self.OFFICIAL_TABLE:
            speeds = f(vx, vy, wz, 0.0586, 0.0829)   # FL,FR,RL,RR
            by_id = {mid: s for mid, s in zip(m, speeds)}
            got = ''.join(self._sign(by_id[i]) for i in range(4))
            self.assertEqual(got, expect,
                             f'按官方电机号 0..3 排回 L1..R2 后「{label}」不符：'
                             f'得到 {got}，应为 {expect}')

    def test_official_wheel_order_is_documented_not_guessed(self):
        src = self._driver_src()
        self.assertIn('WHEEL_NAMES', src)
        self.assertIn('l1/l2/r1/r2', src,
                      '官方 URDF 教程的 joint -> 轮位映射必须写进源码注释')
        self.assertIn('M1/M2/M3/M4=左上/左下/右上/右下', src)

    def _driver_src(self):
        with open(RB, encoding='utf-8') as f:
            return f.read()

    def test_vendor_cmd_is_normalized_and_bounded(self):
        f = RB_FUNCS['vendor_twist_to_normalized']
        self.assertEqual(f(0.25, -0.10, 0.5, 1.0, 1.0), (0.25, -0.10, 0.5))
        # 官方值域：linear [-1,1]，angular [-3,3]，越界必须钳住
        self.assertEqual(f(9.0, -9.0, 9.0, 1.0, 1.0), (1.0, -1.0, 3.0))
        self.assertEqual(f(0.5, 0.5, 1.0, 4.0, 2.0), (1.0, 1.0, 2.0))
        # NaN 与非法 gain 一律安全归零，绝不把 NaN 发给 /driver_node
        self.assertEqual(f(float('nan'), 0.1, 0.1, 1.0, 1.0), (0.0, 0.1, 0.1))
        for g in (0.0, -1.0, float('nan')):
            self.assertEqual(f(0.2, 0.2, 0.2, g, 1.0), (0.0, 0.0, 0.0))
        self.assertEqual(f(0.2, 0.2, 0.2, 1.0, 1.0, 0.0, 3.0),
                         (0.0, 0.0, 0.0))

    def test_servo_lock_rejects_out_of_official_range(self):
        f = RB_FUNCS['servo_lock_angles']
        self.assertEqual(f(90, 25), (90, 25))
        self.assertEqual(f(0, 110), (0, 110))
        self.assertEqual(f(180, 0), (180, 0))
        for bad in ((181, 25), (-1, 25), (90, 111), (90, -1),
                    (float('nan'), 25), (90, float('nan')),
                    ('x', 25), (90, None)):
            self.assertIsNone(f(*bad), f'越界/非法角度必须拒绝：{bad}')

    def test_key1_read_maps_to_bool_or_none(self):
        f = RB_FUNCS['read_key1']

        class Bot:
            def __init__(self, value):
                self.value = value
                self.calls = []

            def read_data_array(self, addr, n):
                self.calls.append((addr, n))
                if isinstance(self.value, Exception):
                    raise self.value
                return self.value

        for raw, want in (([1], True), ([0], False), (1, True), (0, False),
                          (b'\x01', True), (b'\x00', False)):
            self.assertIs(f(Bot(raw)), want, f'read_data_array 返回 {raw!r}')
        for raw in (None, [], RuntimeError('i2c'), object(), 'x'):
            self.assertIsNone(f(Bot(raw)), f'读失败必须返回 None：{raw!r}')
        self.assertIsNone(f(None))
        b = Bot([0])
        f(b, 0x0d)
        self.assertEqual(b.calls, [(0x0d, 1)], '必须按官方 0x0d / 1 字节读')

    def test_official_apis_and_topics_are_pinned(self):
        with open(RB, encoding='utf-8') as f:
            src = f.read()
        for token in ('Ctrl_Muto', 'Ctrl_Servo', 'read_data_array',
                      'KEY1_ADDRESS = 0x0d', 'yahboomcar_msgs.msg',
                      '/servo', 'vendor_twist_to_normalized',
                      'ros2 launch yahboomcar_bringup'):
            self.assertIn(token, src, f'官方接口 {token} 不能丢')

    def test_raspbot_config_exposes_official_chassis_params(self):
        p = _load('raspbot_params.yaml')['/lightdog_raspbot_driver']['ros__parameters']
        self.assertEqual(p['vendor_cmd_topic'], '/cmd_vel')
        self.assertEqual(p['vendor_linear_limit'], 1.0)
        self.assertEqual(p['vendor_angular_limit'], 3.0)
        self.assertGreater(p['vendor_linear_gain'], 0.0)
        self.assertGreater(p['vendor_angular_gain'], 0.0)
        self.assertEqual(p['gimbal_backend'], 'none',
                         '示例角度不是标定位，默认不能擅自下发云台指令')
        self.assertEqual(p['servo_topic'], '/servo')
        self.assertEqual(p['key1_address'], 13)   # 官方 0x0d
        self.assertEqual(p['button_backend'], 'i2c',
                         '官方 /driver_node 只发布 /ultrasonic 与 /line_sensor，'
                         '没人发 /raspbot/button；用 topic 会把实体急停按钮废掉')
        self.assertGreater(p['button_poll_hz'], 0.0)
        self.assertGreaterEqual(p['i2c_max_read_failures'], 1)

    # ---- 行为判据：只测纯函数时，把「下发路径」改回旧逻辑不会被发现 ----

    def _driver_methods(self, names):
        return extract_methods(RB, 'RaspbotDriver', names, ns={
            'Twist': Twist, 'Bool': Msg,
            'clamp_speed': RB_FUNCS['clamp_speed'],
            'mecanum_wheel_speeds': RB_FUNCS['mecanum_wheel_speeds'],
            'normalize_wheel_speeds': RB_FUNCS['normalize_wheel_speeds'],
            'wheel_speeds_to_pwm': RB_FUNCS['wheel_speeds_to_pwm'],
            'vendor_twist_to_normalized':
                RB_FUNCS['vendor_twist_to_normalized'],
            'read_key1': RB_FUNCS['read_key1']})

    def test_vendor_topic_send_uses_normalized_cmd(self):
        pub = Pub()
        obj = types.SimpleNamespace(
            backend='vendor_topic', pub_vendor=pub, frames_sent=0,
            vendor_linear_gain=4.0, vendor_angular_gain=2.0,
            vendor_linear_limit=1.0, vendor_angular_limit=3.0)
        bind(obj, self._driver_methods(['_drv_send']))
        obj._drv_send(0.25, -0.5, 2.0)
        self.assertEqual(obj.frames_sent, 1)
        m = pub.last
        # 官方值域 linear [-1,1] / angular [-3,3]，gain 生效且越界钳住
        self.assertEqual((m.linear.x, m.linear.y, m.angular.z),
                         (1.0, -1.0, 3.0))
        self.assertEqual(obj.last_cmd, (0.25, -0.5, 2.0),
                         'diag 的 last_cmd 必须仍是车体 m/s，别被归一化值污染')

    def test_button_poll_ignores_unknown_and_only_fires_on_change(self):
        methods = self._driver_methods(['_service_gimbal_and_button'])
        calls = []

        class Bot:
            def __init__(self, value):
                self.value = value

            def read_data_array(self, addr, n):
                if isinstance(self.value, Exception):
                    raise self.value
                return self.value

        degraded = []

        def make(value, pressed=False):
            obj = types.SimpleNamespace(
                gimbal_backend='none', last_gimbal=0.0, gimbal_period=1.0,
                button_backend='i2c', bot=Bot(value), button_bot=None,
                last_button_poll=0.0, button_poll_hz=2.0, key1_address=13,
                button_pressed=pressed, get_logger=lambda: Log(),
                i2c_read_failures=0, i2c_link_ok=None,
                i2c_max_read_failures=99,
                _degrade_i2c_link=lambda: degraded.append(1),
                _on_button=lambda m: calls.append(bool(m.data)))
            bind(obj, methods)
            return obj

        make([1])._service_gimbal_and_button(10.0)
        self.assertEqual(calls, [True], '按下必须置位软件闭锁')
        calls.clear()
        make([0], pressed=True)._service_gimbal_and_button(10.0)
        self.assertEqual(calls, [False],
                         '松开要交给 _on_button 的闭锁策略，不在轮询里直接解除')
        calls.clear()
        for bad in (RuntimeError('i2c'), [], object()):
            make(bad)._service_gimbal_and_button(10.0)
        self.assertEqual(calls, [],
                         '读失败 = 不知道，绝不能当成松开去动急停闭锁')
        self.assertEqual(degraded, [],
                         '单次读失败不能立刻降级，必须连续 N 次')

    def test_consecutive_i2c_read_failures_trigger_degrade(self):
        """官方库吞写异常，KEY1 周期读是唯一的运行时链路心跳。"""
        methods = self._driver_methods(['_service_gimbal_and_button'])
        degraded = []

        class Broken:
            def read_data_array(self, addr, n):
                return None

        obj = types.SimpleNamespace(
            gimbal_backend='none', last_gimbal=0.0, gimbal_period=1.0,
            button_backend='i2c', bot=Broken(), button_bot=None,
            last_button_poll=0.0, button_poll_hz=100.0, key1_address=13,
            button_pressed=False, get_logger=lambda: Log(),
            i2c_read_failures=0, i2c_link_ok=True, i2c_max_read_failures=3,
            _degrade_i2c_link=lambda: degraded.append(1),
            _on_button=lambda m: None)
        bind(obj, methods)
        for i in range(3):
            obj._service_gimbal_and_button(10.0 + i)
            self.assertIs(obj.i2c_link_ok, False)
        self.assertEqual(obj.i2c_read_failures, 3)
        self.assertEqual(degraded, [1], '连续 3 次读失败必须触发降级')

        # 链路恢复后计数清零、链路状态回 True，且不再重复降级
        obj.bot = types.SimpleNamespace(read_data_array=lambda a, n: [0])
        obj._degrade_i2c_link = lambda: degraded.append(2)
        obj._service_gimbal_and_button(20.0)
        self.assertEqual(obj.i2c_read_failures, 0)
        self.assertIs(obj.i2c_link_ok, True)
        self.assertEqual(degraded, [1])

    def test_gimbal_lock_publishes_periodically_and_respects_limits(self):
        methods = self._driver_methods(['_service_gimbal_and_button',
                                       '_send_gimbal_lock'])

        class ServoMsg:
            def __init__(self):
                self.servo_s1 = self.servo_s2 = None

        class Bot:
            def __init__(self):
                self.calls = []

            def Ctrl_Servo(self, mid, angle):
                self.calls.append((mid, angle))

        pub, bot = Pub(), Bot()
        obj = types.SimpleNamespace(
            gimbal_backend='servo_topic', last_gimbal=0.0, gimbal_period=1.0,
            button_backend='topic', bot=bot, button_bot=None,
            last_button_poll=0.0, button_poll_hz=2.0, button_pressed=False,
            get_logger=lambda: Log(), i2c_read_failures=0, i2c_link_ok=None,
            i2c_max_read_failures=3,
            _gimbal_angles=(90, 25), _servo_msg_type=ServoMsg,
            pub_servo=pub, key1_address=13)
        bind(obj, methods)
        obj._service_gimbal_and_button(10.0)
        self.assertEqual(len(pub.sent), 1)
        self.assertEqual((pub.last.servo_s1, pub.last.servo_s2), (90, 25))
        obj._service_gimbal_and_button(10.5)   # 周期未满，不刷屏
        self.assertEqual(len(pub.sent), 1)
        obj._service_gimbal_and_button(11.0)   # 满一拍，重发防 driver_node 晚起
        self.assertEqual(len(pub.sent), 2)

        obj.gimbal_backend = 'i2c'
        obj._send_gimbal_lock()
        self.assertEqual(bot.calls, [(1, 90), (2, 25)],
                         '官方 Ctrl_Servo(1/2, angle)')
        before = len(pub.sent)
        obj._gimbal_angles = None
        obj._send_gimbal_lock()
        self.assertEqual(len(pub.sent), before, '角度非法时一次都不发')


if __name__ == '__main__':
    unittest.main()
