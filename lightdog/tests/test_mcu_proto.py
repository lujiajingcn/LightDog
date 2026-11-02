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
import random
import re
import shutil
import subprocess
import sys
import tempfile
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

    def test_doc_python_and_c_tables_agree(self):
        """帧类型表三处必须一致：文档、Python 常量、C 宏。

        docs/firmware_protocol.md 自己写了这条要求（"新增类型必须同时改三处，
        三处漏一处的后果是帧发出去了但上位机当未知帧丢掉"），但一直没人守。
        漏掉时两侧都不会报错：C 侧照样按长度搬字节，Python 侧把它归到「未知帧」，
        日志里只有一行 warn —— 在真机上表现为「某个功能偶尔不生效」。

        比的是「名字 -> 值」，不是两个集合：集合相等但 0x02/0x03 写反了照样是错的。
        """
        doc_p = os.path.join(REPO_ROOT, 'docs', 'firmware_protocol.md')
        hdr = os.path.join(REPO_ROOT, 'firmware', 'core', 'lightdog_proto.h')
        if not (os.path.isfile(doc_p) and os.path.isfile(hdr)):
            self.skipTest('找不到 docs/ 或 firmware/（容器只挂包目录）')

        table = dict(P.DOWN_TYPES)
        table.update(P.UP_TYPES)
        py_t = {name.upper(): typ for typ, name in table.items()}

        with open(doc_p, encoding='utf-8') as f:
            doc = f.read()
        seg = doc[doc.index('下行（ROS → STM32）'):]
        doc_t = {n: int(v, 16) for v, n in re.findall(
            r'^\|\s*`(0x[0-9A-Fa-f]{2})`\s*\|\s*`?([A-Z_]+)`?\s*\|', seg, re.M)}

        with open(hdr, encoding='utf-8') as f:
            c_src = f.read()
        c_t = {n: int(v, 16) for n, v in re.findall(
            r'#define\s+LD_T_([A-Z_]+)\s+(0x[0-9A-Fa-f]+)', c_src)}

        self.assertTrue(doc_t and py_t and c_t, '有一侧一个类型都没解析到，正则要更新')
        for label, a, b in (('文档 vs Python', doc_t, py_t),
                            ('Python vs C', py_t, c_t)):
            only_a = sorted(set(a) - set(b))
            only_b = sorted(set(b) - set(a))
            diff = {k: (a[k], b[k]) for k in set(a) & set(b) if a[k] != b[k]}
            self.assertEqual(
                (only_a, only_b, diff), ([], [], {}),
                f'{label} 不一致：仅左 {only_a}，仅右 {only_b}，值不同 {diff}')


CROSS = os.path.join(REPO_ROOT, 'firmware', 'test', 'cross_stream.c')
CORE = os.path.join(REPO_ROOT, 'firmware', 'core', 'lightdog_proto.c')
GCC = shutil.which('gcc') or shutil.which('cc')


def _mixed_stream():
    """固定种子的混合流：合法帧 / 噪声 / CRC 错 / 截断，按 6 种循环。

    噪声里刻意保留 0xAA（但让 0x55 跟上的概率很低），因为「数据里出现半个
    SYNC」正是解析器最容易写错的地方；全避开 0xAA 的噪声太温柔。
    """
    rnd = random.Random(20261102)
    parts = []
    good = []
    for i in range(120):
        k = i % 6
        if k == 0:
            fr = P.encode_cmd_vel(round(rnd.uniform(-0.3, 0.3), 3),
                                  round(rnd.uniform(-0.8, 0.8), 3))
            parts.append(fr)
            good.append(fr)
        elif k == 1:
            fr = P.encode_battery(round(rnd.random(), 3), 24.0, 1.5)
            parts.append(fr)
            good.append(fr)
        elif k == 2:
            b = bytearray()
            n = rnd.randrange(1, 45)
            while len(b) < n:
                x = rnd.randrange(256)
                if x == 0xAA and rnd.random() < 0.5:
                    x = 0xA9
                b.append(x)
            parts.append(bytes(b))
        elif k == 3:
            fr = bytearray(P.encode_led(0.5))
            fr[-1] ^= 0xFF                       # CRC 错
            parts.append(bytes(fr))
        elif k == 4:
            fr = P.encode_odom(1.0, 2.0, 0.5, 0.1, 0.2)
            parts.append(fr[:len(fr) // 2])      # 截断，后半永远不来
        else:
            fr = P.encode_heartbeat()
            parts.append(fr)
            good.append(fr)
    return b''.join(parts), good


def _blocks(n):
    """不规则块大小（质数步长）。SYNC、LEN、CRC 都会被切在两次 push 之间。"""
    i = 0
    while i < n:
        blk = (i * 37 + 13) % 17 + 1
        yield i, min(blk, n - i)
        i += blk


def _parse_chunked(stream):
    parser = P.Parser()
    out = []
    for off, ln in _blocks(len(stream)):
        for f in parser.feed(stream[off:off + ln]):
            out.append((f.type, f.payload.hex()))
    return out, parser.stats()


class TestCrossImplementation(unittest.TestCase):
    """一长段「什么都混在一起」的字节流，两侧必须给出同样的帧序列。

    黄金向量覆盖的是「每种边界单独构造一次」；这里覆盖真实串口的形态：噪声、
    截断、CRC 错、粘连按随机顺序出现。两侧一旦在丢弃策略上分叉，症状是
    「偶尔丢一条速度指令」——和「电机抽一下」一样难查。
    """

    def test_chunking_does_not_change_result(self):
        """帧序列与「怎么切块」无关。解析器是有状态的，这条并不自动成立。"""
        stream, _ = _mixed_stream()
        one = [(f.type, f.payload.hex()) for f in P.Parser().feed(stream)]
        chunked, _stats = _parse_chunked(stream)
        self.assertEqual(one, chunked)

    def test_all_but_at_most_one_good_frame_survives(self):
        """被截断的帧会吞掉紧随其后的那一帧 —— 这是无序号协议固有的代价。

        它没有序号、没有转义，所以「LEN 说还有 20 字节没到」时只能等，等来的
        是下一帧的字节 → CRC 失败 → 从 SYNC 处重扫，那一帧已经没了。
        所以断言「最多损失一个」，而不是「必须一个不少」：写成后者会逼着实现
        去猜，而半截的速度指令比丢一帧危险得多。
        """
        stream, good = _mixed_stream()
        frames, stats = _parse_chunked(stream)
        self.assertGreaterEqual(len(frames), len(good) - 1,
                                '合法帧丢了不止一个：%s' % stats)
        self.assertEqual(stats['len_errors'], 0, 'LEN 全是合法值，不该报长度错')
        self.assertGreater(stats['crc_errors'], 0, 'CRC 错的帧没有被统计')

    @unittest.skipUnless(GCC and os.path.isfile(CROSS),
                         '需要 gcc 与 firmware/（容器里通常只有包目录）')
    def test_c_and_python_agree_frame_by_frame(self):
        """同一段流、同一种切块，C 与 Python 必须逐帧一致。"""
        stream, _ = _mixed_stream()
        py_frames, _ = _parse_chunked(stream)
        py_text = ''.join('%02x %s\n' % (t, ph) for t, ph in py_frames)

        tmp = tempfile.mkdtemp()
        spath = os.path.join(tmp, 'stream.bin')
        with open(spath, 'wb') as f:
            f.write(stream)
        exe = os.path.join(tmp, 'cross_stream')
        subprocess.run(
            [GCC, '-std=c99', '-Wall', '-Wextra', '-Wpedantic', '-Werror',
             '-I' + os.path.join(REPO_ROOT, 'firmware', 'core'),
             CORE, CROSS, '-o', exe],
            check=True, capture_output=True)
        r = subprocess.run([exe, spath], capture_output=True, text=True)
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertEqual(r.stdout, py_text,
                         'C 与 Python 的帧序列不一致（固件侧实现分叉了）')


if __name__ == '__main__':
    unittest.main(verbosity=2)
