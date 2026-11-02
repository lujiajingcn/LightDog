"""浸泡采样器（deploy/soak.py）的离线回归。

这个文件测的不是「工具能不能跑」，而是**判定会不会骗人**。浸泡工具只有两种
失败方式值得防：

  1. **假阳性**：RSS 涨了 40MB 就报「内存泄漏」。真实原因通常是 Python 的
     arena 不还、首次 import、OpenCV 线程池 —— 这些曲线都「涨完就平」。
     真泄漏的特征是「到最后一刻还在涨」。判错一次的代价是没人再信这份报告。
  2. **假阴性**：跑完打出「全部正常」，而实际上什么都没采到（节点没起、
     `--match` 写错），或者只跑了 3 分钟也敢说 2 小时没问题。

第 2 条比第 1 条危险得多：假阳性只是浪费一次排查，假阴性会让人带着一个
真泄漏上车。所以 rc 的语义在这里被钉死：3=没采到 / 2=观测不足 / 1=发现泄漏 / 0=通过。

`deploy/` 不在包里（容器只挂 `src/lightdog`），所以这组用例在容器里会跳过，
而不是让 run.py 在 import 阶段崩掉。
"""

import csv
import importlib.util
import json
import os
import re
import sys
import tempfile
import unittest
from contextlib import redirect_stderr, redirect_stdout
from io import StringIO

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from harness import PKG  # noqa: E402

REPO_ROOT = os.path.dirname(PKG)
SOAK = os.path.join(REPO_ROOT, 'deploy', 'soak.py')
HAS_SOAK = os.path.isfile(SOAK)


def _import_soak():
    spec = importlib.util.spec_from_file_location('lightdog_soak', SOAK)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


soak = _import_soak() if HAS_SOAK else None

COLS = ['t', 'wall', 'pid', 'name', 'rss_kb', 'vsz_kb', 'threads', 'fds',
        'fd_sock', 'fd_pipe', 'fd_anon', 'fd_file', 'cpu_s', 'start',
        'fd_limit', 'resolved']


def node(name, pid=1000, start=100, rss=None, threads=None, fds=None,
         sock=None, file=None, anon=None, limit=1024, resolved=1,
         since=0, until=10 ** 9):
    """一个被测进程的曲线。每个指标是一个 i -> 值 的函数。"""
    return {
        'name': name, 'pid': pid, 'start': start, 'limit': limit,
        'resolved': resolved, 'since': since, 'until': until,
        'rss': rss or (lambda i: 200.0),
        'threads': threads or (lambda i: 15),
        'fds': fds or (lambda i: 20),
        'sock': sock or (lambda i: 6),
        'file': file or (lambda i: 3),
        'anon': anon or (lambda i: 4),
    }


def write_csv(path, n, dt=5.0, procs=()):
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=COLS)
        w.writeheader()
        for i in range(n):
            for p in procs:
                if not (p['since'] <= i < p['until']):
                    continue
                w.writerow({
                    't': round(i * dt, 3),
                    'wall': '2026-11-02T00:00:00',
                    'pid': p['pid'], 'name': p['name'],
                    'rss_kb': int(p['rss'](i) * 1024),
                    'vsz_kb': int(p['rss'](i) * 2048),
                    'threads': p['threads'](i),
                    'fds': p['fds'](i),
                    'fd_sock': p['sock'](i),
                    'fd_pipe': 1,
                    'fd_anon': p['anon'](i),
                    'fd_file': p['file'](i),
                    'cpu_s': round(i * 0.5, 2),
                    'start': p['start'],
                    'fd_limit': p['limit'],
                    'resolved': p['resolved'],
                })


def report_of(tmpdir, n, procs, min_duration=1800.0, dt=5.0, name='s.csv'):
    path = os.path.join(tmpdir, name)
    write_csv(path, n, dt, procs)
    buf = StringIO()
    with redirect_stdout(buf):
        rc = soak.report(path, min_duration)
    return rc, buf.getvalue()


@unittest.skipUnless(HAS_SOAK, '找不到 deploy/soak.py（容器只挂包目录）')
class TestJudge(unittest.TestCase):
    """判定函数本身的边界。合成序列比真实数据更能钉死判据。"""

    def seq(self, fn, n=240, dt=5.0):
        return [(i * dt, fn(i)) for i in range(n)]

    def test_flat_is_flat(self):
        r = soak.judge(self.seq(lambda i: 200.0 + (i % 3) * 0.1), 16.0, rel=0.10)
        self.assertEqual(r['verdict'], soak.FLAT)

    def test_grown_then_flat_is_not_a_leak(self):
        """20 分钟涨 40MB 然后不动 —— 这是缓存，不是泄漏。

        这是整个工具最要紧的一条：判错它就等于每次浸泡都在喊狼来了。
        """
        r = soak.judge(self.seq(lambda i: 200.0 + min(i / 4.0, 40.0)),
                       16.0, rel=0.10)
        self.assertNotEqual(r['verdict'], soak.LEAK)
        self.assertEqual(r['verdict'], soak.GROWING)
        self.assertLess(r['tail_rise'], 0.25 * r['growth'])

    def test_linear_rss_growth_is_a_leak(self):
        r = soak.judge(self.seq(lambda i: 200.0 + i * 0.5), 16.0, rel=0.10)
        self.assertEqual(r['verdict'], soak.LEAK)
        self.assertAlmostEqual(r['slope_h'], 360.0, delta=20.0)

    def test_fd_sawtooth_pool_is_not_a_leak(self):
        """连接池：20 个 fd 涨到 38 再掉回去。前后量程重叠，不能判泄漏。"""
        r = soak.judge(self.seq(lambda i: 20 + (i % 7) * 3), 5.0,
                       want_disjoint=True)
        self.assertEqual(r['verdict'], soak.FLAT)
        self.assertFalse(r['disjoint'])

    def test_fd_linear_growth_is_a_leak(self):
        r = soak.judge(self.seq(lambda i: 20 + i // 10), 5.0,
                       want_disjoint=True)
        self.assertEqual(r['verdict'], soak.LEAK)
        self.assertTrue(r['disjoint'])

    def test_thread_staircase_is_a_leak(self):
        """线程泄漏是阶梯形的（一次泄漏就是一个台阶），必须判得出来。"""
        r = soak.judge(self.seq(lambda i: 15 + i // 40), 3.0,
                       want_disjoint=True)
        self.assertEqual(r['verdict'], soak.LEAK)

    def test_thread_oscillation_is_not_a_leak(self):
        r = soak.judge(self.seq(lambda i: 15 + (i % 11) // 4), 3.0,
                       want_disjoint=True)
        self.assertEqual(r['verdict'], soak.FLAT)

    def test_growing_but_not_disjoint_is_only_a_warning(self):
        """在涨，但前后区间仍然重叠 —— 只提示，不下泄漏结论。"""
        # 锯齿上升：每个周期从 20 涨到 39，但每个周期的底在慢慢抬高。
        # 任意时刻的取值区间都和前 1/3 重叠，所以不是泄漏；但它确实在涨。
        r = soak.judge(self.seq(lambda i: 20 + (i % 20) + i // 25), 5.0,
                       want_disjoint=True)
        self.assertEqual(r['verdict'], soak.GROWING)

    def test_one_sample_shows_real_value_not_zero(self):
        """只有一拍时不能显示 0→0：那会被读成「线程数是 0」。"""
        r = soak.judge([(0.0, 17)], 3.0, want_disjoint=True)
        self.assertEqual(r['verdict'], soak.INCONCLUSIVE)
        self.assertEqual(r['head'], 17)
        self.assertEqual(r['tail'], 17)

    def test_shrink_is_reported(self):
        r = soak.judge(self.seq(lambda i: 300.0 - i * 0.5), 16.0, rel=0.10)
        self.assertEqual(r['verdict'], soak.SHRINK)


@unittest.skipUnless(HAS_SOAK, '找不到 deploy/soak.py（容器只挂包目录）')
class TestReport(unittest.TestCase):
    """报告与退出码。rc 的四种取值各自对应一个真实场景。"""

    def setUp(self):
        import tempfile
        self.tmp = tempfile.mkdtemp()

    def tearDown(self):
        import shutil
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_clean_long_run_passes(self):
        rc, out = report_of(self.tmp, 400, [node('lightdog_mission')])
        self.assertEqual(rc, 0, out)
        self.assertIn('未发现泄漏', out)

    def test_leak_sets_rc_1(self):
        rc, out = report_of(self.tmp, 400, [
            node('lightdog_server', rss=lambda i: 200.0 + i * 0.5)])
        self.assertEqual(rc, 1, out)
        self.assertIn('内存', out)

    def test_short_run_never_says_ok(self):
        """10 分钟的数据足以看 fd，但**不足以**给「无泄漏」的结论。"""
        rc, out = report_of(self.tmp, 120, [node('lightdog_mission')])
        self.assertEqual(rc, 2, out)
        self.assertIn('不构成', out)

    def test_min_duration_zero_still_refuses_a_green_light(self):
        """`--min-duration 0` 是调试用的；1 分钟也必须说「不构成结论」。"""
        rc, out = report_of(self.tmp, 12, [node('lightdog_mission')],
                            min_duration=0.0)
        self.assertEqual(rc, 2, out)
        self.assertIn('不构成', out)

    def test_empty_csv_is_blind(self):
        path = os.path.join(self.tmp, 'empty.csv')
        write_csv(path, 0, 5.0, [])
        buf = StringIO()
        with redirect_stdout(buf):
            rc = soak.report(path, 1800.0)
        self.assertEqual(rc, 3)

    def _write_sys(self, n, avail_fn):
        with open(os.path.join(self.tmp, 's.sys.csv'), 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=[
                't', 'wall', 'mem_total_kb', 'mem_avail_kb', 'swap_free_kb',
                'swap_total_kb', 'load1'])
            w.writeheader()
            for i in range(n):
                w.writerow({'t': i * 5.0, 'wall': 'x',
                            'mem_total_kb': 8000000,
                            'mem_avail_kb': avail_fn(i),
                            'swap_free_kb': 0, 'swap_total_kb': 0,
                            'load1': 6.0})

    def test_system_memory_row_is_reported(self):
        """整机内存必须出现在报告里。

        逐进程表只能看见 `--match` 匹配到的进程。树莓派上 gz / Nav2 / 浏览器
        都不在里面，内存被它们吃掉时逐进程表是全绿的 —— 这一行是唯一的兜底。
        曾经因为一次替换没生效，这一行悄悄没打印，而报告照样「通过」。
        """
        path = os.path.join(self.tmp, 's.csv')
        write_csv(path, 400, 5.0, [node('lightdog_recorder')])
        self._write_sys(400, lambda i: 4000000 - i * 1000)
        buf = StringIO()
        with redirect_stdout(buf):
            soak.report(path, 1800.0)
        out = buf.getvalue()
        self.assertIn('可用内存', out)
        self.assertIn('负载', out)

    def test_low_system_memory_is_a_failure(self):
        """整机只剩 5% 可用内存时，逐进程全绿也不能算通过。"""
        path = os.path.join(self.tmp, 's.csv')
        write_csv(path, 400, 5.0, [node('lightdog_recorder')])
        self._write_sys(400, lambda i: 400000 - i * 10)
        buf = StringIO()
        with redirect_stdout(buf):
            rc = soak.report(path, 1800.0)
        self.assertEqual(rc, 1, buf.getvalue())
        self.assertIn('OOM', buf.getvalue())

    def test_process_that_dies_midway_is_reported(self):
        """中途退出、之后没再起来的节点，曲线只是「提前结束」。

        judge 会拿它仅有的那一段算趋势，算出来是一条平线 —— 不加这一条，
        「跑了一半就死了」在报告里和「稳定运行 33 分钟」长得一模一样。
        """
        rc, out = report_of(self.tmp, 400, [
            node('lightdog_voice', pid=100, until=200),
            node('lightdog_mission', pid=200),
        ])
        self.assertEqual(rc, 1, out)
        self.assertIn('再没出现过', out)

    def test_restart_is_detected(self):
        """同名两次启动、任何一拍里只有一份 = 被重启过。不靠 starttime 看不出来。

        同时要钉住**不是**「重复实例」：先死后起和两份同时在跑，处置完全不同。
        """
        rc, out = report_of(self.tmp, 400, [
            node('lightdog_recorder', pid=100, start=111, until=200),
            node('lightdog_recorder', pid=200, start=222, since=200),
        ])
        self.assertEqual(rc, 1, out)
        self.assertIn('中途重启', out)
        self.assertNotIn('同时有', out)

    def test_duplicate_instances_are_detected(self):
        """同一时刻两份同名节点 —— 会互相抢话题，是故障不是泄漏。"""
        rc, out = report_of(self.tmp, 400, [
            node('lightdog_twist_mux', pid=100, start=111),
            node('lightdog_twist_mux', pid=200, start=111),
        ])
        self.assertEqual(rc, 1, out)
        self.assertIn('实例', out)

    def test_duplicate_instances_with_different_starttimes(self):
        """两份实例**同时活着**，但启动时刻不同 —— 必须报「重复实例」。

        这是最容易被写错的一格：判据若只看「有几个 starttime」，那么
        「重启没杀干净、两份同时在跑」会被说成「中途重启过」。
        后者听起来像是系统自己恢复过一次（可以接受），而前者是两份节点
        正在互相抢话题（必须立刻处理）。两种结论的处置完全不同。
        """
        rc, out = report_of(self.tmp, 400, [
            node('lightdog_voice', pid=100, start=111),
            node('lightdog_voice', pid=200, start=222),
        ])
        self.assertEqual(rc, 1, out)
        self.assertIn('同时有 2 个实例', out)
        self.assertNotIn('中途重启', out)

    def test_fallback_names_are_not_duplicate_alarms(self):
        """`--match python` 时十几个无关进程都叫 python3。

        这不是「重复实例」，报出来就是每次浸泡都在喊狼来了。
        """
        rc, out = report_of(self.tmp, 400, [
            node('python3', pid=100, resolved=0),
            node('python3', pid=200, resolved=0),
            node('python3', pid=300, resolved=0),
        ])
        self.assertEqual(rc, 0, out)
        self.assertNotIn('实例', out)

    def test_fd_near_limit_is_flagged(self):
        rc, out = report_of(self.tmp, 400, [
            node('lightdog_server', fds=lambda i: 900, limit=1024)])
        self.assertEqual(rc, 1, out)
        self.assertIn('accept', out)

    def test_socket_leak_names_the_type(self):
        """只有指向 socket 的 fd 在涨，报告必须说清是哪一类。"""
        rc, out = report_of(self.tmp, 400, [
            node('lightdog_server', fds=lambda i: 20 + i // 10,
                 sock=lambda i: 6 + i // 10)])
        self.assertEqual(rc, 1, out)
        self.assertIn('socket', out)

    def test_data_dir_growth_is_reported(self):
        path = os.path.join(self.tmp, 's.csv')
        write_csv(path, 400, 5.0, [node('lightdog_recorder')])
        with open(os.path.join(self.tmp, 's.dir.csv'), 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=['t', 'wall', 'bytes', 'files'])
            w.writeheader()
            for i in range(400):
                w.writerow({'t': i * 5.0, 'wall': 'x',
                            'bytes': 3 * 1024 * 1024 + i * 1024 * 200,
                            'files': 10 + i})
        buf = StringIO()
        with redirect_stdout(buf):
            soak.report(path, 1800.0)
        self.assertIn('数据目录', buf.getvalue())
        self.assertIn('MB/小时', buf.getvalue())

    def test_shrinking_data_dir_says_retention(self):
        """目录变小是**保留策略在工作**，不是「增长率为负」。

        原来这一行会打成 `+-38.1M（-39 个文件）` 和 `≈ -69 MB/小时`：
        前者是格式错误，后者会被读成「磁盘写入速率异常」，
        而事实恰恰相反 —— 那是 `event_retain_days` 在删旧录像。
        """
        path = os.path.join(self.tmp, 's.csv')
        write_csv(path, 400, 5.0, [node('lightdog_recorder')])
        with open(os.path.join(self.tmp, 's.dir.csv'), 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=['t', 'wall', 'bytes', 'files'])
            w.writeheader()
            for i in range(400):
                w.writerow({'t': i * 5.0, 'wall': 'x',
                            'bytes': 500 * 1024 * 1024 - i * 100000,
                            'files': 200 - i // 10})
        buf = StringIO()
        with redirect_stdout(buf):
            rc = soak.report(path, 1800.0)
        out = buf.getvalue()
        self.assertEqual(rc, 0, out)
        self.assertNotIn('+-', out, '负增长被打成了「+-」')
        self.assertNotIn('MB/小时', out, '目录在缩小时不该给增长率')
        self.assertIn('保留策略', out)


@unittest.skipUnless(HAS_SOAK, '找不到 deploy/soak.py（容器只挂包目录）')
class TestProcNaming(unittest.TestCase):
    """进程归组。归错了，报告里就只剩「python3 涨了 300MB」。"""

    def test_ros2_wrapper_is_not_a_node(self):
        """`ros2 run` 的命令行里含节点脚本名，不能当成第二个实例。"""
        cmd = ('/usr/bin/python3 /opt/ros/jazzy/bin/ros2 run lightdog '
               'lightdog_twist_mux.py --ros-args -p x:=1')
        self.assertTrue(soak._is_cli_wrapper(cmd))

    def test_plain_node_is_not_a_wrapper(self):
        cmd = ('python3 /home/ros/ws/install/lightdog/lib/lightdog/'
               'lightdog_twist_mux.py --ros-args')
        self.assertFalse(soak._is_cli_wrapper(cmd))

    def test_name_comes_from_script_not_comm(self):
        """ros2 run 起来的节点 comm 全是 python3，按 comm 归组会并成一条线。"""
        cmd = 'python3 /home/ros/ws/lib/lightdog/lightdog_recorder.py --ros-args'
        self.assertEqual(soak._name(cmd, 'python3'), ('lightdog_recorder', True))

    def test_unmatched_process_falls_back_to_comm(self):
        """兜底出来的名字只用来显示，**不是身份**（第二个返回值是 False）。"""
        self.assertEqual(soak._name('gz sim -r -s world.sdf', 'ruby'),
                         ('ruby', False))

    def test_cjk_padding_counts_two_columns(self):
        """报告表格里的中文列宽：按字符数算会整体错位。"""
        self.assertEqual(soak.dwidth('进程'), 4)
        self.assertEqual(soak.dwidth('lightdog'), 8)
        self.assertEqual(soak.dwidth(soak.pad('进程', 10)), 10)


@unittest.skipUnless(HAS_SOAK, '找不到 deploy/soak.py（容器只挂包目录）')
class TestRunGuards(unittest.TestCase):
    """参数与环境校验。这些错误必须在**建 CSV 之前**拦住。

    原先 `--interval 0` 会进忙循环（实测 1 秒写 2404 拍），而文件已经建好了：
    留下一个巨大的半截 CSV 和一句「观测不足」，比直接拒绝更让人困惑。
    """

    def setUp(self):
        self.tmp = tempfile.mkdtemp()

    def test_zero_interval_is_rejected_before_creating_csv(self):
        out = os.path.join(self.tmp, 'z.csv')
        err = StringIO()
        with redirect_stdout(StringIO()), redirect_stderr(err):
            rc = soak.main(['run', '--duration', '0.2', '--interval', '0',
                            '--out', out])
        self.assertEqual(rc, 2)
        self.assertIn('interval', err.getvalue())
        self.assertFalse(os.path.exists(out),
                         '参数被拒绝时不该留下一个只有表头的 CSV')

    def test_missing_proc_is_explained_not_a_traceback(self):
        """Windows 宿主机上 `listdir('/proc')` 抛 WinError 3。

        裸栈里只有一行「系统找不到指定的路径: '/proc'」，看不出这个工具
        本来就只支持 Linux。这里把 isdir 换成假件模拟宿主机。
        """
        out = os.path.join(self.tmp, 'p.csv')
        err = StringIO()
        real = soak.os.path.isdir
        soak.os.path.isdir = lambda p: False if p == '/proc' else real(p)
        try:
            with redirect_stdout(StringIO()), redirect_stderr(err):
                rc = soak.main(['run', '--duration', '0.1', '--out', out])
        finally:
            soak.os.path.isdir = real
        self.assertEqual(rc, 3)
        self.assertIn('/proc', err.getvalue())
        self.assertIn('Linux', err.getvalue())


@unittest.skipUnless(HAS_SOAK, '找不到 deploy/soak.py（容器只挂包目录）')
@unittest.skipUnless(os.path.isdir('/proc'), '没有 /proc（Windows 宿主机）')
class TestLiveSampling(unittest.TestCase):
    """真读一次 /proc。合成 CSV 测不到「字段取错位」这类问题。"""

    def test_snapshot_reads_this_process(self):
        pat = re.compile(r'python')
        rows, _d, _s = soak.snapshot(pat, '')
        self.assertTrue(rows, '一个 python 进程都没采到')
        me = os.getpid()
        mine = [r for r in rows if r['pid'] == me]
        self.assertTrue(mine, '没采到自己（pid %d）' % me)
        r = mine[0]
        for col in ('rss_kb', 'vsz_kb', 'threads', 'fds', 'cpu_s', 'start'):
            self.assertIn(col, r)
        self.assertGreater(r['rss_kb'], 0)
        self.assertGreaterEqual(r['threads'], 1)
        self.assertGreater(r['start'], 0)

    def test_fd_breakdown_does_not_exceed_total(self):
        rows, _d, _s = soak.snapshot(re.compile(r'python'), '')
        for r in rows:
            parts = r['fd_sock'] + r['fd_pipe'] + r['fd_anon'] + r['fd_file']
            self.assertLessEqual(parts, r['fds'],
                                 '%s 的分类 fd 加起来超过了总数' % r['name'])

    def test_no_match_returns_nothing_rather_than_inventing(self):
        rows, _d, _s = soak.snapshot(re.compile(r'zzz-不可能存在的进程名-zzz'), '')
        self.assertEqual(rows, [])

    def test_run_writes_all_three_csvs(self):
        """端到端跑一次 run()。

        这条抓到过一个真实故障：`_sysinfo()` 把 `/proc/meminfo` 的原始键名
        （MemAvailable）原样交给 DictWriter，而列名是 mem_avail_kb —— 
        DictWriter 遇到多余的键是**抛异常**，于是采到第一拍、主 CSV 已落盘、
        进程随即退出，表现成「soak.csv 里只有一行」。不跑一遍根本看不出来。
        """
        import tempfile
        tmp = tempfile.mkdtemp()
        out = os.path.join(tmp, 's.csv')
        buf = StringIO()
        with redirect_stdout(buf), open(os.devnull, 'w') as devnull:
            old_err = sys.stderr
            sys.stderr = devnull
            try:
                rc = soak.main(['run', '--duration', '1.2',
                                '--interval', '0.4', '--match', 'python',
                                '--out', out, '--watch-dir', tmp])
            finally:
                sys.stderr = old_err
        self.assertIn(rc, (0, 2), '采样器自己崩了：%s' % buf.getvalue()[-500:])
        for suffix, min_rows in (('', 1), ('.dir.csv', 1), ('.sys.csv', 1)):
            path = out if not suffix else os.path.splitext(out)[0] + suffix
            self.assertTrue(os.path.isfile(path), '没生成 %s' % path)
            with open(path, newline='') as f:
                rows = list(csv.DictReader(f))
            self.assertGreaterEqual(len(rows), min_rows, '%s 没有数据行' % path)

    def test_auto_named_csv_is_reported_in_json(self):
        """不传 --out 时，尾部那个机器可读字段必须报**真实**落盘路径。

        原先报的是 `{"csv": ""}`，而文件其实叫 `soak-<时间>.csv` ——
        下游按这个字段取文件会取不到，偏偏日志里看起来一切正常。
        """
        tmp = tempfile.mkdtemp()
        cwd = os.getcwd()
        os.chdir(tmp)
        try:
            buf = StringIO()
            with redirect_stdout(buf), redirect_stderr(StringIO()):
                soak.main(['run', '--duration', '0.3', '--interval', '0.1',
                           '--match', 'python'])
        finally:
            os.chdir(cwd)
        tail = [ln for ln in buf.getvalue().splitlines() if ln.startswith('{')]
        self.assertTrue(tail, '没有输出机器可读尾巴')
        info = json.loads(tail[-1])
        self.assertTrue(info['csv'], 'csv 字段是空的，而文件已经落了盘')
        self.assertTrue(os.path.isfile(os.path.join(tmp, info['csv'])),
                        '报出来的路径找不到文件：%r' % info['csv'])

    def test_run_with_unmatchable_pattern_refuses_to_pass(self):
        """什么都没采到必须 rc=3，不能打「未发现泄漏」。"""
        import tempfile
        tmp = tempfile.mkdtemp()
        out = os.path.join(tmp, 'blind.csv')
        buf = StringIO()
        with redirect_stdout(buf), open(os.devnull, 'w') as devnull:
            old = sys.stderr
            sys.stderr = devnull
            try:
                rc = soak.main(['run', '--duration', '1', '--interval', '0.2',
                                '--match', 'zzz-不可能-zzz', '--out', out])
            finally:
                sys.stderr = old
        self.assertEqual(rc, 3)
