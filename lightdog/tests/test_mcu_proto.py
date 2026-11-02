"""下位机串口协议（D3 STM32）：编解码、解析边界、黄金向量。

两条线必须同时成立：

1. **Python 与 C 用同一份向量**（`firmware/test/vectors.txt`）。固件跑不到
   `run.py` 里，所以只能靠共享向量对齐 —— 任何一侧改错，`make -C firmware test`
   和这里的 `test_golden_vectors` 会同时红。
2. **解析器要能挨打**：UART 上没有「一次一帧」，半帧、两帧粘连、噪声、
   CRC 错、`LEN` 离谱都是常态。这里把每种情况都钉住，因为它在真机上的表现
   是「偶尔丢一条速度指令」，和「电机偶尔抽一下」长得一模一样。
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from harness import PKG  # noqa: E402

sys.path.insert(0, PKG)

import lightdog_mcu_proto as P  # noqa: E402

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
VECTORS = os.path.join(REPO_ROOT, 'firmware', 'test', 'vectors.txt')


class TestCrc(unittest.TestCase):

    def test_check_value(self):
        # CRC-16/CCITT-FALSE 的标准校验值。改多项式就会立刻红。
        self.assertEqual(P.crc16(b'123456789'), 0x29B1)

    def test_empty(self):
        self.assertEqual(P.crc16(b''), 0xFFFF)

    def test_covers_len_type_payload_only(self):
        frame = P.encode_cmd_vel(0.1, 0.2)
        body = frame[2:-2]
        self.assertEqual(int.from_bytes(frame[-2:], 'little'), P.crc16(body))
        # SYNC 不参与 CRC：把它改掉，CRC 仍然对得上（解析器负责认 SYNC）
        self.assertNotEqual(P.crc16(frame[:5]), int.from_bytes(frame[-2:], 'little'))


class TestRoundTrip(unittest.TestCase):

    def one(self, frame, ftype):
        frames = P.Parser().feed(frame)
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].type, ftype)
        return frames[0].payload

    def test_cmd_vel(self):
        pay = self.one(P.encode_cmd_vel(0.25, -0.7), P.T_CMD_VEL)
        lin, ang = P.decode_cmd_vel(pay)
        self.assertAlmostEqual(lin, 0.25, places=6)
        self.assertAlmostEqual(ang, -0.7, places=6)

    def test_led(self):
        pay = self.one(P.encode_led(0.85), P.T_LED)
        self.assertAlmostEqual(P.decode_led(pay), 0.85, places=6)

    def test_heartbeat_is_empty_payload(self):
        pay = self.one(P.encode_heartbeat(), P.T_HEARTBEAT)
        self.assertEqual(pay, b'')

    def test_estop_roundtrip(self):
        self.assertTrue(P.decode_estop(self.one(P.encode_estop(True), P.T_ESTOP)))
        self.assertFalse(P.decode_estop(self.one(P.encode_estop(False), P.T_ESTOP)))

    def test_battery(self):
        pay = self.one(P.encode_battery(0.42, 24.6, 1.5), P.T_BATTERY)
        pct, volt, amp = P.decode_battery(pay)
        self.assertAlmostEqual(pct, 0.42, places=5)
        self.assertAlmostEqual(volt, 24.6, places=4)
        self.assertAlmostEqual(amp, 1.5, places=5)

    def test_odom(self):
        pay = self.one(P.encode_odom(1.5, -0.25, 3.14159, 0.2, 0.1), P.T_ODOM)
        x, y, yaw, v, w = P.decode_odom(pay)
        self.assertAlmostEqual(x, 1.5, places=5)
        self.assertAlmostEqual(yaw, 3.14159, places=5)

    def test_fault_bits(self):
        mask = P.F_LINK_LOST | P.F_OVERCURRENT
        pay = self.one(P.encode_fault(mask), P.T_FAULT)
        self.assertEqual(P.decode_fault(pay), mask)
        self.assertEqual(P.fault_text(mask), 'link_lost+overcurrent')
        self.assertEqual(P.fault_text(0), 'ok')

    def test_fault_unknown_bit_is_visible(self):
        # 固件加了新故障位而 ROS 侧还没更新时，不能显示成空
        self.assertIn('0x0040', P.fault_text(0x0040))

    def test_decode_rejects_wrong_length(self):
        # 半截 payload 比没有更危险 —— 必须返回 None 而不是猜
        self.assertIsNone(P.decode_cmd_vel(b'\x00' * 4))
        self.assertIsNone(P.decode_odom(b'\x00' * 8))
        self.assertIsNone(P.decode_battery(b''))
        self.assertIsNone(P.decode_estop(b'\x00\x01'))

    def test_encode_rejects_oversize(self):
        with self.assertRaises(ValueError):
            P.encode(P.T_LED, b'\x00' * (P.MAX_PAYLOAD + 1))


class TestParserEdges(unittest.TestCase):
    """每一种都在真机上真的会发生，且症状都是「电机偶尔抽一下」。"""

    def frames(self, data, parser=None):
        return (parser or P.Parser()).feed(data)

    def test_two_frames_in_one_read(self):
        a = P.encode_cmd_vel(0.1, 0.0)
        b = P.encode_heartbeat()
        got = self.frames(a + b)
        self.assertEqual([f.type for f in got], [P.T_CMD_VEL, P.T_HEARTBEAT])

    def test_byte_by_byte(self):
        frame = P.encode_led(0.5)
        p = P.Parser()
        got = []
        for i in range(len(frame)):
            got += p.feed(frame[i:i + 1])
        self.assertEqual(len(got), 1)
        self.assertEqual(got[0].type, P.T_LED)
        self.assertEqual(p.dropped, 0, '逐字节喂不该丢字节')

    def test_garbage_prefix(self):
        p = P.Parser()
        got = p.feed(b'\x01\x02\x03\x04' + P.encode_heartbeat())
        self.assertEqual(len(got), 1)
        self.assertEqual(p.dropped, 4)

    def test_pure_garbage_clears_buffer(self):
        p = P.Parser()
        self.assertEqual(p.feed(b'\x00' * 64), [])
        self.assertEqual(len(p.buf), 0, '没有 SYNC 时必须清空，否则内存会涨')
        self.assertEqual(p.dropped, 64)

    def test_bad_crc_does_not_eat_next_frame(self):
        frame = P.encode_cmd_vel(0.1, 0.0)
        broken = frame[:-1] + bytes([frame[-1] ^ 0xFF])
        nxt = P.encode_heartbeat()
        p = P.Parser()
        got = p.feed(broken + nxt)
        self.assertEqual(p.crc_errors, 1)
        self.assertEqual([f.type for f in got], [P.T_HEARTBEAT],
                         'CRC 错只该丢那一个 SYNC，后面的真帧必须还能解出来')

    def test_bad_len_is_resync_source(self):
        p = P.Parser()
        got = p.feed(bytes([0xAA, 0x55, 0xFF, 0x02, 0x00]))
        self.assertEqual(got, [])
        self.assertEqual(p.len_errors, 1)
        got = p.feed(P.encode_heartbeat())
        self.assertEqual(len(got), 1, 'LEN 离谱之后必须能重新对齐')

    def test_sync_inside_payload(self):
        # 编码器数据里出现 0xAA55 是完全正常的，不能被当成分帧点
        frame = P.encode(0x7E, b'\xaa\x55\xaa\x55')
        got = self.frames(frame)
        self.assertEqual(len(got), 1)
        self.assertEqual(got[0].payload, b'\xaa\x55\xaa\x55')

    def test_unknown_type_is_delivered(self):
        # 固件新增了类型而 ROS 侧没更新：帧要交上来（由桥接节点计数并告警），
        # 而不是在解析层被悄悄丢掉 —— 丢掉会让「固件升级了但没生效」无法排查。
        got = self.frames(P.encode(0x7F, b'\x01'))
        self.assertEqual(len(got), 1)
        self.assertEqual(got[0].type, 0x7F)

    def test_buffer_overflow_guard(self):
        p = P.Parser(max_buffer=64)
        p.feed(b'\x00' * 32)
        p.feed(b'\x00' * 128)          # 没有 SYNC，缓冲会一直涨
        self.assertLessEqual(len(p.buf), 64)
        self.assertGreater(p.dropped, 0)

    def test_stats_shape(self):
        p = P.Parser()
        p.feed(P.encode_heartbeat())
        st = p.stats()
        for k in ('frames', 'crc_errors', 'len_errors', 'dropped_bytes', 'buffered'):
            self.assertIn(k, st)


class TestGoldenVectors(unittest.TestCase):
    """与 `make -C firmware test` 共读这一个文件。"""

    def setUp(self):
        if not os.path.isfile(VECTORS):
            self.skipTest('找不到 firmware/test/vectors.txt（容器只挂包目录）')
        self.rows = []
        with open(VECTORS, encoding='utf-8') as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                name, typ, pay, frame = line.split('|')
                self.rows.append((name, int(typ, 16),
                                  b'' if pay == '-' else bytes.fromhex(pay),
                                  bytes.fromhex(frame)))

    def test_encode_matches(self):
        self.assertGreaterEqual(len(self.rows), 8)
        for name, typ, pay, frame in self.rows:
            with self.subTest(name=name):
                self.assertEqual(P.encode(typ, pay).hex(), frame.hex(),
                                 f'{name}：Python 编码与向量不一致')

    def test_parse_matches(self):
        for name, typ, pay, frame in self.rows:
            with self.subTest(name=name):
                got = P.Parser().feed(frame)
                self.assertEqual(len(got), 1, f'{name}：解析帧数不对')
                self.assertEqual(got[0].type, typ, f'{name}：type 不一致')
                self.assertEqual(got[0].payload, pay, f'{name}：payload 不一致')

    def test_every_type_has_a_vector(self):
        covered = {typ for _, typ, _, _ in self.rows}
        missing = set(P.TYPE_NAMES) - covered
        self.assertEqual(missing, set(),
                         f'这些类型没有黄金向量：{sorted(P.TYPE_NAMES[t] for t in missing)}')


class TestTypeTable(unittest.TestCase):

    def test_down_and_up_do_not_collide(self):
        self.assertEqual(set(P.DOWN_TYPES) & set(P.UP_TYPES), set())

    def test_heartbeat_payload_is_empty_by_contract(self):
        # 「心跳能单独发」是协议的核心：它必须能在不发速度的情况下发出去。
        # 帧体里除了 LEN/TYPE 只有 CRC，没有任何负载。
        frame = P.encode_heartbeat()
        self.assertEqual(frame[:4], P.SYNC + b'\x00\x03')
        self.assertEqual(len(frame), 6)
        self.assertEqual(P.Parser().feed(frame)[0].payload, b'')


if __name__ == '__main__':
    unittest.main(verbosity=2)
