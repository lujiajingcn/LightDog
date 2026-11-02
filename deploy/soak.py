#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""LightDog 浸泡采样器：只读 /proc，不 import rclpy、不连网络、不起线程。

## 为什么要单独一个工具，而不是在节点里加统计

浸泡测试要回答的唯一问题是「连续跑 N 小时之后，这台机器会不会变慢 / 撑爆」。
如果采样器本身就跑在节点里，那么节点卡死、DDS 起不来、内存涨到 OOM 的时候，
**采样器会跟着一起死** —— 最后只留下一段「数据在崩溃前戛然而止」的曲线，
而最需要数据的那一刻恰好没有。所以这里全部是标准库，读 /proc，写 CSV。

## 采什么，为什么是这些

| 指标 | 为什么是它 |
|---|---|
| RSS / VSZ | 找内存泄漏。**但 RSS 涨不等于泄漏**，见下面的判定 |
| 线程数 | 最干净的泄漏信号：没有分配器噪声，一个线程就是一个整数 |
| fd 数（按类型分开） | 泄漏最典型的表现。分类型才知道是 socket 泄漏（连接线程没退）还是文件泄漏（录像没 release） |
| CPU 累计 tick | 能抓到「活着但在空转」（忙等 / 死循环重连） |
| starttime | 进程重启后 pid 会变，starttime 也会变。**不记它就分不清「稳定运行 2 小时」和「被 systemd 重启了 40 次」** |
| 目录字节数 | 存储卡会不会被写满；录像 / 事件切片是持续小文件写 |

## 判定为什么要这么写（这是整个工具最容易被做错的地方）

「2 小时里 RSS 涨了 30MB」几乎总是**误报**：Python 的 arena 不还、import 缓存、
OpenCV 的线程池、glibc 的 malloc 阈值，都会让 RSS 单调上涨然后**停在某个平台上**。
真泄漏的特征不是「涨了」，而是「到最后一刻还在涨」。

所以每个指标都按「前 1/3 中位」vs「后 1/3 中位」比较，并且要求**量程不相交**
（后 1/3 的最小值 > 前 1/3 的最大值）才算持续增长 —— 锯齿形的连接池会
前后重叠，被判成正常；线性泄漏的量程必然分开。RSS 因为噪声大，两条都放宽：
中位数抬升 ≥ max(绝对阈值, 起始值的 10%) 且**最后 1/4 相对前 1/4 仍在涨**
（一条平掉的曲线在这里 tail_rise≈0）。

还有一个反向的错误同样危险：观测时间不够长却打出「全部正常」。
所以 `--min-duration`（默认 30 分钟）以内不给 OK，只给 INCONCLUSIVE（rc=2）。
慢速内存泄漏是每小时几 MB 的量级，10 分钟的数据**不可能**分得出来，
在这里给绿灯等于骗人。

## 用法

    # 跑一次 2 小时浸泡（在容器里或树莓派上）
    python3 deploy/soak.py run --duration 7200 --out /tmp/soak.csv

    # 事后重新分析同一份 CSV（不用重跑）
    python3 deploy/soak.py report /tmp/soak.csv

退出码：0 正常 / 1 发现泄漏 / 2 观测不足（不构成结论）/ 3 一个进程都没采到
"""

import argparse
import csv
import json
import os
import re
import signal
import sys
import time
import unicodedata
from datetime import datetime

PAGE = os.sysconf('SC_PAGE_SIZE') if hasattr(os, 'sysconf') else 4096
CLK = os.sysconf('SC_CLK_TCK') if hasattr(os, 'sysconf') else 100

#: 默认匹配「我们自己的节点」而不是全部进程：树莓派上连 gz 都没有，
#: 而在仿真容器里把所有进程都采下来会让 CSV 里 90% 是 Nav2 / rviz。
DEFAULT_MATCH = r'lightdog_[a-z0-9_]+\.py|lightdog-[a-z]+'

OK = 'OK'
LEAK = 'LEAK'
INCONCLUSIVE = 'INCONCLUSIVE'
BLIND = 'BLIND'
GROWING = 'GROWING'
FLAT = 'FLAT'
SHRINK = 'SHRINK'


# ---------------------------------------------------------------- /proc 读取

def _pids():
    for e in os.listdir('/proc'):
        if e.isdigit():
            yield int(e)


def _cmdline(pid):
    try:
        with open('/proc/%d/cmdline' % pid, 'rb') as f:
            raw = f.read()
    except (IOError, OSError):
        return ''
    # cmdline 用 \0 分隔；ros2 run 出来的节点 argv[1] 就是脚本路径，
    # 所以原样拼成一行再匹配，比只匹配 argv[0] 可靠。
    return raw.replace(b'\x00', b' ').decode('utf-8', 'replace').strip()


def _stat(pid):
    """/proc/<pid>/stat -> dict。comm 里可能有空格和括号，必须从最后一个 ')' 切。"""
    try:
        with open('/proc/%d/stat' % pid, 'rb') as f:
            s = f.read().decode('utf-8', 'replace')
        rp = s.rindex(')')
        comm = s[s.index('(') + 1:rp]
        f = s[rp + 2:].split()
        # 字段编号从 1 开始（1=pid, 2=comm, 3=state），切掉 pid/comm 后
        # index 0 对应字段 3，所以「字段 N」= f[N - 3]。
        return {
            'comm': comm,
            'utime': int(f[11]),
            'stime': int(f[12]),
            'threads': int(f[17]),
            'start': int(f[19]),
        }
    except (IOError, OSError, ValueError, IndexError):
        return None


def _statm(pid):
    """/proc/<pid>/statm：只要 vsz 与 rss（单位页）。比 status 少一次解析。"""
    try:
        with open('/proc/%d/statm' % pid, 'rb') as f:
            fld = f.read().split()
        return int(fld[0]) * PAGE, int(fld[1]) * PAGE
    except (IOError, OSError, ValueError, IndexError):
        return None


def _fds(pid):
    """返回 (总数, 分类计数)。分类靠 readlink 的目标串 —— 类型比总数有用得多：
    sock 涨说明连接/线程没退，file 涨说明录像没 release，anon 涨说明
    eventfd/epoll 泄漏（每个 Queue 都带一个，是 asyncio/threading 的常见残留）。"""
    d = '/proc/%d/fd' % pid
    try:
        names = os.listdir(d)
    except (IOError, OSError):
        return None
    n = {'sock': 0, 'pipe': 0, 'anon': 0, 'file': 0, 'dev': 0}
    for x in names:
        try:
            t = os.readlink(os.path.join(d, x))
        except OSError:
            continue                      # 读的瞬间被关掉了：跳过，不扣总数
        if t.startswith('socket:'):
            n['sock'] += 1
        elif t.startswith('pipe:'):
            n['pipe'] += 1
        elif t.startswith('anon_inode:'):
            n['anon'] += 1
        elif t.startswith('/dev/'):
            n['dev'] += 1
        else:
            n['file'] += 1
    return len(names), n


def _fd_limit(pid):
    """软限制。fd 泄漏最惨的形态不是「涨」，是「涨到 1024 然后 accept 全部失败」。"""
    try:
        with open('/proc/%d/limits' % pid) as f:
            for line in f:
                if line.startswith('Max open files'):
                    return int(line.split()[3])
    except (IOError, OSError, ValueError, IndexError):
        pass
    return 0


def _is_cli_wrapper(cmd):
    """`ros2 run` / `ros2 launch` 是调度器，不是节点。

    它们的命令行里含有节点脚本名（`ros2 run lightdog lightdog_twist_mux.py`），
    所以会被正则一起匹配进来。不排掉的话报告里每个节点都出现两行 —— 一行是真
    节点、一行是 28MB 空转的包装进程 —— 而「同名两个实例」在报告里是**故障**
    级别的问题（本项目真发生过两份节点抢话题）。真正的节点是包装进程的子进程，
    本来就会被采到。
    """
    for tok in cmd.split():
        if os.path.basename(tok) == 'ros2':
            return True
    return False


def _name(cmd, comm):
    """归一化进程名 -> (名字, 这个名字是不是「身份」)。

    优先看 argv 里的脚本名而不是 comm：`ros2 run` 起来的所有节点 comm 都是
    `python3`，按 comm 归组会把十几个节点并成一条线 —— 那样报告里只剩
    「python3 涨了 300MB」，等于什么都没说。

    第二个返回值是**必须有**的。用 comm 兜底时（比如 `--match python`），
    十几个不相干的 python 进程全叫 `python3`；如果把「同名多实例」当成故障去报，
    每次都会刷出「python3 同时有 11 个实例在跑」—— 假警报比漏报更毁工具。
    所以 comm 兜底出来的名字只用来显示，不参与「重复实例 / 中途重启」的判定。
    """
    for tok in cmd.split():
        b = os.path.basename(tok)
        if re.match(r'^lightdog[-_][A-Za-z0-9_]+(\.py)?$', b):
            return re.sub(r'\.py$', '', b), True
    return comm, False


def _sysinfo():
    """整机内存与负载。

    为什么在一份「按进程」的报告里还要采整机：`--match` 默认只匹配 lightdog
    的节点，而树莓派上 gz / Nav2 / rviz 都不在里面。如果内存是被那些进程吃掉的，
    逐进程的表会全绿 —— 而机器已经在 OOM 边缘。这一行是唯一能兜住那种情况的数据。
    """
    # 键名直接用 CSV 的列名：DictWriter 对不上的键会**抛异常**而不是忽略，
    # 而这里已经采到第一拍、主 CSV 也写下去了 —— 表现是「跑 5 秒就退出，
    # soak.csv 里只有一行」。别把 meminfo 的原始键名透到 dict 里。
    rename = {'MemTotal': 'mem_total_kb', 'MemAvailable': 'mem_avail_kb',
              'SwapFree': 'swap_free_kb', 'SwapTotal': 'swap_total_kb'}
    d = {}
    try:
        with open('/proc/meminfo') as f:
            for line in f:
                k, _, v = line.partition(':')
                if k in rename:
                    d[rename[k]] = int(v.split()[0])
    except (IOError, OSError, ValueError):
        return None
    try:
        with open('/proc/loadavg') as f:
            d['load1'] = float(f.read().split()[0])
    except (IOError, OSError, ValueError, IndexError):
        d['load1'] = 0.0
    return d


def _dir_bytes(path):
    """目录总字节 + 文件数。只 scandir，不 stat 每个文件的 inode 详情。"""
    total = 0
    count = 0
    try:
        for root, _dirs, files in os.walk(path):
            for fn in files:
                try:
                    total += os.path.getsize(os.path.join(root, fn))
                    count += 1
                except OSError:
                    pass
    except OSError:
        return 0, 0
    return total, count


# ---------------------------------------------------------------- 采样

def snapshot(pat, watch_dir):
    """采一拍。返回 (rows, dirrow)。"""
    rows = []
    for pid in _pids():
        cmd = _cmdline(pid)
        if not cmd or not pat.search(cmd) or _is_cli_wrapper(cmd):
            continue
        st = _stat(pid)
        if st is None:
            continue                       # 这一拍正好退出
        mem = _statm(pid)
        if mem is None:
            continue
        fd = _fds(pid)
        vsz, rss = mem
        name, resolved = _name(cmd, st['comm'])
        rows.append({
            'pid': pid,
            'name': name,
            'resolved': 1 if resolved else 0,
            'rss_kb': rss // 1024,
            'vsz_kb': vsz // 1024,
            'threads': st['threads'],
            'fds': fd[0] if fd else -1,
            'fd_sock': fd[1]['sock'] if fd else -1,
            'fd_pipe': fd[1]['pipe'] if fd else -1,
            'fd_anon': fd[1]['anon'] if fd else -1,
            'fd_file': fd[1]['file'] if fd else -1,
            'cpu_s': round((st['utime'] + st['stime']) / float(CLK), 2),
            'start': st['start'],
            'fd_limit': _fd_limit(pid),
        })
    drow = None
    if watch_dir:
        b, c = _dir_bytes(watch_dir)
        drow = {'bytes': b, 'files': c}
    return rows, drow, _sysinfo()


COLUMNS = ['t', 'wall', 'pid', 'name', 'rss_kb', 'vsz_kb', 'threads', 'fds',
           'fd_sock', 'fd_pipe', 'fd_anon', 'fd_file', 'cpu_s', 'start',
           'fd_limit', 'resolved']


def run(args):
    pat = re.compile(args.match)
    out = args.out or time.strftime('soak-%Y%m%d-%H%M%S.csv')
    stop = {'flag': False}

    def _stop(_sig, _frm):
        stop['flag'] = True
    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    t0 = time.monotonic()
    n = 0
    seen_any = False
    fh = open(out, 'w', newline='')
    w = csv.DictWriter(fh, fieldnames=COLUMNS)
    w.writeheader()
    fh.flush()
    dh = None
    if args.watch_dir:
        dh = open(os.path.splitext(out)[0] + '.dir.csv', 'w', newline='')
        dw = csv.DictWriter(dh, fieldnames=['t', 'wall', 'bytes', 'files'])
        dw.writeheader()
        dh.flush()
    sh = open(os.path.splitext(out)[0] + '.sys.csv', 'w', newline='')
    sw = csv.DictWriter(sh, fieldnames=['t', 'wall', 'mem_total_kb',
                                        'mem_avail_kb', 'swap_free_kb',
                                        'swap_total_kb', 'load1'])
    sw.writeheader()
    sh.flush()

    sys.stderr.write('采集中：%s，间隔 %.1fs，上限 %.0fs，输出 %s\n'
                     % (args.match, args.interval, args.duration, out))
    try:
        while not stop['flag']:
            el = time.monotonic() - t0
            if el > args.duration:
                break
            rows, drow, sysd = snapshot(pat, args.watch_dir)
            if rows:
                seen_any = True
            wall = datetime.now().strftime('%Y-%m-%dT%H:%M:%S')
            for r in rows:
                r['t'] = round(el, 3)
                r['wall'] = wall
                w.writerow(r)
            fh.flush()
            if drow is not None:
                drow['t'] = round(el, 3)
                drow['wall'] = wall
                dw.writerow(drow)
                dh.flush()
            if sysd is not None:
                sysd['t'] = round(el, 3)
                sysd['wall'] = wall
                sw.writerow(sysd)
                sh.flush()
            n += 1
            # 用绝对时间睡到下一拍，而不是 sleep(interval)：采样本身要花
            # 几十毫秒，累加下去 2 小时会漂出几分钟（横轴全是歪的）。
            # 第 n 拍应该落在 t0 + n*interval —— 写成 (n+1) 会每拍跳过一个点，
            # 2 小时的采样数直接减半。
            nxt = t0 + n * args.interval
            gap = nxt - time.monotonic()
            while gap > 0 and not stop['flag']:
                time.sleep(min(gap, 0.2))
                gap = nxt - time.monotonic()
    finally:
        fh.close()
        sh.close()
        if dh is not None:
            dh.close()

    el = time.monotonic() - t0
    sys.stderr.write('采样结束：%.0fs / %d 拍\n' % (el, n))
    if not seen_any:
        sys.stderr.write('一个进程都没匹配到（--match=%r）。\n' % args.match)
        sys.stderr.write('这不等于「系统正常」：什么都没看着。检查节点是不是没起。\n')
        return 3
    return report(out, args.min_duration)


# ---------------------------------------------------------------- 分析

def median(xs):
    s = sorted(xs)
    n = len(s)
    if not n:
        return 0.0
    return float(s[n // 2]) if n % 2 else (s[n // 2 - 1] + s[n // 2]) / 2.0


def theil_sen(pairs):
    """中位数斜率。对 GC/一次性分配造成的尖峰不敏感，比最小二乘稳。

    为了 2 小时 × 5 秒 = 1440 点也不至于算 100 万对，最多取 120 个点。
    """
    pts = pairs[:]
    if len(pts) < 2:
        return 0.0
    step = max(1, len(pts) // 120)
    pts = pts[::step]
    slopes = []
    for i in range(len(pts)):
        for j in range(i + 1, len(pts)):
            dx = pts[j][0] - pts[i][0]
            if dx > 0:
                slopes.append((pts[j][1] - pts[i][1]) / dx)
    return median(slopes) if slopes else 0.0


def judge(series, floor, rel=0.0, want_disjoint=False):
    """series: [(t, v)] 升序。

    返回 dict(verdict, growth, slope_h, disjoint, tail_rise, head, tail)。
    判据用「前 1/3 中位 → 后 1/3 中位」，并且要求增长**持续到最后一刻**。
    """
    n = len(series)
    if n < 6:
        # 拍数不够不判，但**必须把当前值填进去**：显示 0→0 会让人以为
        # 「线程数和 fd 都是 0」，那是完全不同的故障。
        v = float(series[-1][1]) if n else 0.0
        return {'verdict': INCONCLUSIVE, 'growth': 0.0, 'slope_h': 0.0,
                'disjoint': False, 'tail_rise': 0.0,
                'head': v, 'tail': v, 'n': n}
    k = max(2, n // 3)
    head_v = [v for _, v in series[:k]]
    tail_v = [v for _, v in series[-k:]]
    head = median(head_v)
    tail = median(tail_v)
    growth = tail - head
    # 最后 1/4 相对前 1/4 是否还在涨：区分「线性泄漏」和「涨完就平」
    q = max(1, n // 4)
    tail_rise = median([v for _, v in series[-q:]]) - \
        median([v for _, v in series[-2 * q:-q]])
    disjoint = min(tail_v) > max(head_v)

    need = max(floor, rel * abs(head))
    if growth <= -need:
        verdict = SHRINK
    elif growth < need:
        verdict = FLAT
    elif want_disjoint and not disjoint:
        verdict = GROWING
    elif tail_rise < 0.25 * growth:
        # 涨到某个平台就停了：这是缓存/arena/线程池，不是泄漏
        verdict = GROWING
    else:
        verdict = LEAK
    return {'verdict': verdict, 'growth': growth, 'slope_h': theil_sen(series) * 3600,
            'disjoint': disjoint, 'tail_rise': tail_rise,
            'head': head, 'tail': tail, 'n': n}


def load_csv(path):
    """读回 CSV -> (samples, dir_samples)。samples 是按拍的 [{pid: row}]。"""
    with open(path, newline='') as f:
        rows = list(csv.DictReader(f))
    by_t = {}
    for r in rows:
        t = float(r['t'])
        rec = {
            'pid': int(r['pid']), 'name': r['name'],
            'rss_kb': int(r['rss_kb']), 'vsz_kb': int(r['vsz_kb']),
            'threads': int(r['threads']), 'fds': int(r['fds']),
            'fd_sock': int(r['fd_sock']), 'fd_pipe': int(r['fd_pipe']),
            'fd_anon': int(r['fd_anon']), 'fd_file': int(r['fd_file']),
            'cpu_s': float(r['cpu_s']), 'start': int(r['start']),
            'fd_limit': int(r.get('fd_limit') or 0),
            # 老版本 CSV 没有这一列：那时只采 lightdog 节点，一律当可信。
            'resolved': int(r.get('resolved') or 1),
        }
        by_t.setdefault(t, {})[rec['pid']] = rec
    samples = [(t, by_t[t]) for t in sorted(by_t)]
    ds = []
    dp = os.path.splitext(path)[0] + '.dir.csv'
    if os.path.isfile(dp):
        with open(dp, newline='') as f:
            for r in csv.DictReader(f):
                ds.append((float(r['t']), int(r['bytes']), int(r['files'])))
    sy = []
    sp = os.path.splitext(path)[0] + '.sys.csv'
    if os.path.isfile(sp):
        with open(sp, newline='') as f:
            for r in csv.DictReader(f):
                sy.append({
                    't': float(r['t']),
                    'mem_total_kb': int(r['mem_total_kb']),
                    'mem_avail_kb': int(r['mem_avail_kb']),
                    'swap_free_kb': int(r['swap_free_kb']),
                    'swap_total_kb': int(r['swap_total_kb']),
                    'load1': float(r['load1']),
                })
    return samples, ds, sy


def dwidth(s):
    return sum(2 if unicodedata.east_asian_width(c) in 'WF' else 1 for c in s)


def pad(s, n):
    s = str(s)
    return s + ' ' * max(0, n - dwidth(s))


def mb(kb):
    return '%.0fM' % (kb / 1024.0)


def analyze(samples, ds, min_duration):
    """把逐拍数据切成「每个进程一条时间线」，逐条判定。

    关键点：**按 (name, pid) 分别成线**，不按 name 合并。同名多 pid 意味着
    有重复实例（本项目真的发生过：`ros2 launch` 被杀之后子节点被 init 收养，
    重启后两份节点同时抢话题），这是比泄漏更严重的故障，必须单独报出来。
    """
    if not samples:
        return None
    t_first = samples[0][0]
    t_last = samples[-1][0]
    dur = t_last - t_first

    lines = {}                            # (name, pid) -> [(t, rec)]
    dup = {}                              # name -> 同一时刻出现过的最大实例数
    for t, procs in samples:
        cnt = {}
        for pid, r in procs.items():
            lines.setdefault((r['name'], pid), []).append((t, r))
            if r['resolved']:
                cnt[r['name']] = cnt.get(r['name'], 0) + 1
        for k, v in cnt.items():
            if v > dup.get(k, 0):
                dup[k] = v

    procs = []
    resolved_names = set()
    for (name, pid), pts in lines.items():
        if pts[0][1]['resolved']:
            resolved_names.add(name)
        series = {
            'rss_mb': [(t, r['rss_kb'] / 1024.0) for t, r in pts],
            'threads': [(t, r['threads']) for t, r in pts],
            'fds': [(t, r['fds']) for t, r in pts],
            'fd_sock': [(t, r['fd_sock']) for t, r in pts],
            'fd_file': [(t, r['fd_file']) for t, r in pts],
            'fd_anon': [(t, r['fd_anon']) for t, r in pts],
        }
        last = pts[-1][1]
        first = pts[0][1]
        cpu = last['cpu_s'] - first['cpu_s']
        procs.append({
            'name': name, 'pid': pid, 'n': len(pts),
            'rss': judge(series['rss_mb'], 16.0, rel=0.10),
            'threads': judge(series['threads'], 3.0, want_disjoint=True),
            'fds': judge(series['fds'], 5.0, want_disjoint=True),
            'fd_sock': judge(series['fd_sock'], 4.0, want_disjoint=True),
            'fd_file': judge(series['fd_file'], 4.0, want_disjoint=True),
            'fd_anon': judge(series['fd_anon'], 4.0, want_disjoint=True),
            'first': first, 'last': last,
            'cpu_avg': (cpu / dur * 100.0) if dur > 0 else 0.0,
            'cpu_total': cpu,
        })

    # 同名多 pid：既有「重复实例」也有「中途重启」，用 starttime 区分
    by_name = {}
    for p in procs:
        if p['name'] in resolved_names:
            by_name.setdefault(p['name'], []).append(p)
    restarted = []
    for name, ps in by_name.items():
        if len(ps) > 1:
            st = sorted(set(p['first']['start'] for p in ps))
            restarted.append((name, len(ps), len(st)))
    return {'dur': dur, 'n_samples': len(samples), 'procs': procs,
            'dup': dup, 'restarted': restarted, 'dir': ds,
            't_first': t_first, 't_last': t_last}


def report(path, min_duration):
    samples, ds, sy = load_csv(path)
    if not samples:
        print('CSV 里没有任何样本：%s' % path)
        return 3
    a = analyze(samples, ds, min_duration)
    a['sys'] = sy
    dur = a['dur']
    leaks = []
    warns = []
    # 一分钟以内的数据无论如何都不给绿灯：`--min-duration 0` 是给人调试用的，
    # 而「观测 0.6 秒 → 未发现泄漏」这种结论比没有结论危险得多。
    floor_secs = max(min_duration, 60.0)

    print('')
    print('=' * 96)
    print('LightDog 浸泡报告   %s' % datetime.now().strftime('%Y-%m-%d %H:%M:%S'))
    print('观测 %.1f 分钟（%.0fs）/ %d 拍 / %d 个进程 / 数据 %s'
          % (dur / 60.0, dur, a['n_samples'], len(a['procs']), path))
    print('=' * 96)
    hdr = (pad('进程', 24) + pad('拍数', 6) + pad('RSS 首→末', 16)
           + pad('MB/h', 8) + pad('线程', 10) + pad('FD', 10) + pad('CPU%', 7))
    print(hdr)
    print('-' * 96)

    # 按 RSS 末值排序：最占内存的排前面，出问题时一眼看到
    for p in sorted(a['procs'], key=lambda x: -x['last']['rss_kb']):
        r = p['rss']
        th = p['threads']
        fd = p['fds']
        print(pad(p['name'], 24)
              + pad(p['n'], 6)
              + pad('%s→%s' % (mb(p['first']['rss_kb']), mb(p['last']['rss_kb'])), 16)
              # 拍数太少时不给斜率：16 个点的 Theil-Sen 会被锯齿带偏，
              # 出现过「RSS 首→末 +6M」配「-298 MB/h」这种自相矛盾的一行。
              + pad(('%+.1f' % r['slope_h']) if p['n'] >= 30 else '--', 8)
              + pad('%d→%d' % (th['head'], th['tail']), 10)
              + pad('%d→%d' % (fd['head'], fd['tail']), 10)
              + pad('%.1f' % p['cpu_avg'], 7))
        for key, label in (('rss', '内存'), ('threads', '线程'),
                           ('fds', 'fd 总数'), ('fd_sock', 'socket'),
                           ('fd_file', '文件'), ('fd_anon', 'eventfd/epoll')):
            if p[key]['verdict'] == LEAK:
                leaks.append('%s 的%s持续增长：%+.1f（%.1f/小时，量程%s）'
                             % (p['name'], label, p[key]['growth'],
                                p[key]['slope_h'],
                                '不相交' if p[key]['disjoint'] else '重叠'))
            elif p[key]['verdict'] == GROWING:
                warns.append('%s 的%s在涨但已趋平：%+.1f（多为缓存/连接池，未判泄漏）'
                             % (p['name'], label, p[key]['growth']))
        lim = p['last']['fd_limit']
        if lim and p['last']['fds'] >= 0.8 * lim:
            leaks.append('%s 的 fd 已用到 %d/%d（≥80%%，再涨就是 accept 失败）'
                         % (p['name'], p['last']['fds'], lim))

    print('-' * 96)
    for name, cnt, starts in a['restarted']:
        if starts > 1:
            leaks.append('%s 出现了 %d 次不同的启动（starttime 变化）→ 中途重启过'
                         % (name, starts))
        elif cnt > 1:
            leaks.append('%s 同时有 %d 个实例在跑 → 重复实例，两份会互相抢话题'
                         % (name, cnt))
    for name, cnt in sorted(a['dup'].items()):
        if cnt > 1 and not any(name == r[0] for r in a['restarted']):
            leaks.append('%s 在某一拍里有 %d 个实例' % (name, cnt))

    if a['dir']:
        d = a['dir']
        grow = (d[-1][1] - d[0][1]) / 1024.0 / 1024.0
        rate = grow / (dur / 3600.0) if dur > 0 else 0.0
        print(pad('数据目录', 24) + pad('', 6)
              + pad('+%.1fM（%d 个文件）' % (grow, d[-1][2] - d[0][2]), 34)
              + '≈ %.0f MB/小时' % rate)
        print('-' * 96)
        # 512MB 的卡、涨到 90% 之前告警。这里只提示速率，不判泄漏。
        if rate > 200:
            warns.append('数据目录增长 %.0f MB/小时，存储卡会被写满；'
                         '核对 event_retain_days / max_event_mb' % rate)

    if a['sys']:
        sy0, sy1 = a['sys'][0], a['sys'][-1]
        total = sy1['mem_total_kb'] or 1
        print(pad('系统', 24) + pad('', 6)
              + pad('可用内存 %s→%s（总 %s）'
                    % (mb(sy0['mem_avail_kb']), mb(sy1['mem_avail_kb']),
                       mb(total)), 34)
              + '负载 %.2f' % sy1['load1'])
        print('-' * 96)
        drop = (sy0['mem_avail_kb'] - sy1['mem_avail_kb']) / float(total)
        if sy1['mem_avail_kb'] < 0.12 * total:
            leaks.append('可用内存只剩 %s（总 %s）：已经在 OOM 边缘，'
                         '逐进程表看着正常也没用'
                         % (mb(sy1['mem_avail_kb']), mb(total)))
        elif drop > 0.30:
            warns.append('可用内存从 %s 掉到 %s（-%.0f%%）：可能是某个没被 '
                         '--match 采到的进程在涨'
                         % (mb(sy0['mem_avail_kb']), mb(sy1['mem_avail_kb']),
                            drop * 100))

    for line in warns:
        print('    提示  ' + line)
    for line in leaks:
        print('    问题  ' + line)
    print('-' * 96)

    if leaks:
        print('结论：发现 %d 个问题（rc=1）' % len(leaks))
        return 1
    if dur < floor_secs:
        print('结论：本次观测 %.1f 分钟 < %.0f 分钟，**不构成「无泄漏」的结论**（rc=2）'
              % (dur / 60.0, floor_secs / 60.0))
        print('      fd / 线程这类整数指标的泄漏几分钟就能看出来，上面没有报说明大概率没有；')
        print('      慢速内存泄漏是每小时几 MB 的量级，必须跑满 %d 分钟才有意义。'
              % int(floor_secs / 60))
        return 2
    print('结论：%.1f 分钟内未发现泄漏（rc=0）' % (dur / 60.0))
    return 0


# ---------------------------------------------------------------- CLI

def main(argv=None):
    ap = argparse.ArgumentParser(
        description='LightDog 长时间浸泡采样（只读 /proc，不依赖 ROS）')
    sub = ap.add_subparsers(dest='cmd')

    r = sub.add_parser('run', help='采样并出报告')
    r.add_argument('--duration', type=float, default=7200.0, help='采样总时长（秒），默认 2 小时')
    r.add_argument('--interval', type=float, default=5.0, help='采样间隔（秒）')
    r.add_argument('--match', default=DEFAULT_MATCH, help='匹配 cmdline 的正则')
    r.add_argument('--out', default='', help='CSV 输出路径')
    r.add_argument('--watch-dir', default='', help='同时统计这个目录的字节数与文件数')
    r.add_argument('--min-duration', type=float, default=1800.0,
                   help='少于这么多秒不给「无泄漏」结论（默认 30 分钟）')

    p = sub.add_parser('report', help='分析已经采好的 CSV')
    p.add_argument('csv')
    p.add_argument('--min-duration', type=float, default=1800.0)

    args = ap.parse_args(argv)
    if args.cmd == 'report':
        return report(args.csv, args.min_duration)
    if args.cmd != 'run':
        ap.print_help()
        return 2

    # 只在这一个入口用 json —— 留一个机器可读的尾巴，方便以后接 CI。
    # 放在这里而不是每拍打印，是为了不把 stdout 变成日志流。
    rc = run(args)
    print(json.dumps({'csv': args.out or '', 'rc': rc}, ensure_ascii=False))
    return rc


if __name__ == '__main__':
    sys.exit(main())
