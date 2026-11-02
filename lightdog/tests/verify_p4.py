#!/usr/bin/env python3
"""P4 端到端验收：录像可回放 + HTTP 接口 + 事件切片 + 联系人。

与同目录 test_*.py 的区别：那几个是**离线** AST 测试，不连仿真；
这个必须**仿真在跑**（P4 的 recorder + server 已拉起、相机在出图）。

用法（容器内）：
    docker exec sim python3 /home/ros/linorobot2_ws/src/lightdog/tests/verify_p4.py
    docker exec sim bash -lc 'bash sim/p4.sh probe'   # 项目自带的接口/Range 自检

退出码 0 = 全通过。
"""
import json
import os
import socket
import sys
import time
import urllib.request

import cv2
import numpy as np

BASE = 'http://127.0.0.1:8000'
FAIL = []


def check(name, ok, detail=''):
    print(('PASS  ' if ok else 'FAIL  ') + name + (f'   {detail}' if detail else ''))
    if not ok:
        FAIL.append(name)


def get(path, timeout=10):
    with urllib.request.urlopen(BASE + path, timeout=timeout) as r:
        return r.status, r.read(), dict(r.headers)


def post(path, obj, method='POST', timeout=10):
    data = json.dumps(obj).encode()
    req = urllib.request.Request(BASE + path, data=data, method=method,
                                 headers={'Content-Type': 'application/json'})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return r.status, r.read()
    except urllib.error.HTTPError as e:
        return e.code, e.read()


def video_stats(buf):
    """mp4 字节 -> (帧数, 采样帧均值, 形状, 帧间最大差)。"""
    p = '/tmp/_p4_probe.mp4'
    with open(p, 'wb') as f:
        f.write(buf)
    cap = cv2.VideoCapture(p)
    n, means, shape = 0, [], None
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        n += 1
        if n % 20 == 1:
            means.append(float(frame.mean()))
            shape = frame.shape
    cap.release()
    return n, (float(np.mean(means)) if means else 0.0), shape, \
        (max(means) - min(means) if means else 0.0)


def main():
    print('=== 1. 静态页 / 健康检查 ===')
    st, body, _ = get('/')
    check('GET / 静态页', st == 200 and b'<html' in body.lower(), f'{st} {len(body)}B')
    st, body, _ = get('/api/v1/health')
    h = json.loads(body)
    check('GET /api/v1/health', st == 200 and h.get('ok') is True, str(h))

    print('\n=== 2. 状态 ===')
    s = json.loads(get('/api/v1/status')[1])
    cam, rec = s['camera'], s['recorder']
    check('status.camera 有帧', cam['has_frame'] and cam['fps'] > 0,
          f"fps={cam['fps']:.2f} {cam['width']}x{cam['height']} frames={cam['frames']} "
          f"dropped={cam['dropped']}")
    check('status.recorder 在录', rec['recording'] and rec['segments'] > 0,
          f"segments={rec['segments']} last_fps={rec['last_fps']} bytes={rec['total_bytes']}")
    check('status.alert 字段存在', 'alert' in s, s['alert']['state'])

    print('\n=== 3. 快照 JPEG ===')
    st, body, hdr = get('/api/v1/snapshot.jpg')
    img = cv2.imdecode(np.frombuffer(body, np.uint8), cv2.IMREAD_COLOR)
    check('snapshot 是合法 JPEG', st == 200 and hdr.get('Content-Type', '').startswith('image/'),
          f"{st} {len(body)}B {hdr.get('Content-Type')}")
    check('snapshot 能解码且非全黑', img is not None and img.mean() > 5,
          f'shape={None if img is None else img.shape} '
          f'mean={None if img is None else round(float(img.mean()), 1)}')

    print('\n=== 4. 分段录像可回放 ===')
    clips = json.loads(get('/api/v1/clips')[1])['clips']
    check('clips 列表非空', len(clips) > 0, f'{len(clips)} 段')
    check('clips 带时段元数据（手机端显示时长用）',
          all(k in clips[-1] for k in ('seq', 't0', 't1', 'frames', 'duration')),
          json.dumps({k: clips[-1].get(k) for k in ('file', 'duration', 'frames')},
                     ensure_ascii=False))
    latest = clips[-1]
    st, vbody, vhdr = get('/api/v1/clips/' + latest['file'], timeout=20)
    n, mean, shape, rng = video_stats(vbody)
    check('最新分段可下载', st == 200 and len(vbody) > 5000,
          f"{latest['file']} {len(vbody)}B ct={vhdr.get('Content-Type')}")
    check('分段能解码出帧', n > 50, f'frames={n} shape={shape}')
    check('回放画面非全黑', mean > 5, f'mean={round(mean, 1)} 帧间差={round(rng, 2)}')
    check('索引帧数与实测一致', abs(n - latest.get('frames', n)) <= max(10, n * 0.2),
          f"索引={latest.get('frames')} 实测={n} 时长={latest.get('duration')}s")

    print('\n=== 5. MJPEG 实时流 ===')
    try:
        sk = socket.create_connection(('127.0.0.1', 8000), timeout=8)
        sk.sendall(b'GET /api/v1/stream.mjpg HTTP/1.1\r\nHost: x\r\n\r\n')
        buf, frames, t0 = b'', 0, time.time()
        while time.time() - t0 < 6 and frames < 3:
            chunk = sk.recv(65536)
            if not chunk:
                break
            buf += chunk
            frames = buf.count(b'\xff\xd8\xff')
        sk.close()
        check('stream 返回 multipart', b'multipart/x-mixed-replace' in buf,
              buf.split(b'\r\n')[0][:40].decode(errors='replace'))
        check('stream 至少 2 帧 JPEG', frames >= 2, f'{frames} 帧 / 6s')
    except Exception as e:
        check('stream 连接', False, repr(e))

    print('\n=== 6. 事件切片（P4 核心：事发前 N 秒）===')
    before = {e['id'] for e in json.loads(get('/api/v1/events')[1])['events']}
    st, body = post('/api/v1/events', {'type': 'test', 'label': 'P4 验收-手动触发',
                                       'pre_sec': 8.0, 'post_sec': 8.0})
    check('POST /api/v1/events 受理', st == 202,
          f'{st} {body.decode(errors="replace")[:120]}')

    ev, deadline = None, time.time() + 90
    while time.time() < deadline:
        evs = json.loads(get('/api/v1/events')[1])['events']
        new = [e for e in evs if e['id'] not in before]
        if new and new[0].get('frames'):
            ev = new[0]
            break
        time.sleep(3)

    if ev is None:
        check('事件切片生成', False, '90s 内没看到新事件')
    else:
        check('事件切片生成', True, f"id={ev['id']} frames={ev.get('frames')}")
        m = json.loads(get('/api/v1/events/' + ev['id'])[1])
        check('事件 meta 记了 pre/post/duration',
              m.get('pre_sec') == 8.0 and m.get('post_sec') == 8.0 and m.get('duration') == 16.0,
              f"pre={m.get('pre_sec')} post={m.get('post_sec')} dur={m.get('duration')}")
        st, vb, vh = get(f"/api/v1/events/{ev['id']}/video", timeout=20)
        n2, mean2, shape2, _ = video_stats(vb)
        check('事件视频可下载', st == 200 and len(vb) > 5000,
              f'{len(vb)}B ct={vh.get("Content-Type")}')
        check('事件视频能解码', n2 > 30, f'frames={n2} shape={shape2}')
        check('事件视频非全黑', mean2 > 5, f'mean={round(mean2, 1)}')
        tb = get(f"/api/v1/events/{ev['id']}/thumb")[1]
        timg = cv2.imdecode(np.frombuffer(tb, np.uint8), cv2.IMREAD_COLOR)
        check('事件缩略图可解码', timg is not None,
              f'{len(tb)}B {None if timg is None else timg.shape}')

    print('\n=== 7. 联系人读写 ===')
    st, body = post('/api/v1/contacts', {'contacts': [
        {'name': '张阿姨', 'phone': '13800000001', 'primary': True},
        {'name': '社区值班', 'phone': '13800000002'},
    ]}, method='PUT')
    check('PUT /api/v1/contacts', st == 200, body.decode(errors='replace')[:100])
    cs = json.loads(get('/api/v1/contacts')[1])
    check('联系人已落盘并可读回', st == 200 and len(cs['contacts']) == 2,
          json.dumps(cs['contacts'], ensure_ascii=False))
    check('contacts.json 真的写到了 /data', os.path.isfile('/data/contacts.json'),
          f'{os.path.getsize("/data/contacts.json")}B')

    print('\n=== 8. 告警接口 ===')
    a = json.loads(get('/api/v1/alarms')[1])
    check('GET /api/v1/alarms', a.get('current') is not None,
          f"current={a.get('current', {}).get('state')} 历史={len(a.get('alarms', []))}")

    print('\n=== 汇总 ===')
    print('FAILED: ' + (', '.join(FAIL) if FAIL else 'none'))
    return 1 if FAIL else 0


if __name__ == '__main__':
    sys.exit(main())
