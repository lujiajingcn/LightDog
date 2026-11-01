/* LightDog 手机端逻辑
 *
 * 三条设计取舍，都是为了"在手机上真的能用"：
 *
 * 1. 实时画面不用 <img src="stream.mjpg">。
 *    iOS Safari 对 multipart/x-mixed-replace 的支持时好时坏，失败了还静默黑屏。
 *    这里改成 fetch + ReadableStream 自己从字节流里切 JPEG（扫 FFD8…FFD9），
 *    拿到 Blob 再画到 canvas —— 所有现代手机浏览器都支持，而且能自己控制重连。
 *
 * 2. 令牌必须能放进 query。
 *    <video> 和 <img> 没法带自定义 header，所以 /api/v1/* 全部同时接受
 *    ?token=。页面首次访问带 ?token=xxx 时会被存进 localStorage。
 *
 * 3. 事件和分段分开列。
 *    事件是"要给人看的"（有事发前后 20 秒），分段是"卡里顺便留的原始素材"，
 *    用途不同，混在一起只会让人找不到东西。
 */

'use strict';

const API = '/api/v1';
const LS_TOKEN = 'lightdog.token';

const S = {
  token: '',
  online: false,
  streamAbort: null,
  streamTimer: null,
  streamFails: 0,
  pollTimer: null,
  statusTimer: null,
  lastFrameAt: 0,
  frameCount: 0,
  fpsShown: 0,
  contacts: [],
};

/* ------------------------------------------------------------------ */
/* 基础工具                                                            */
/* ------------------------------------------------------------------ */

const $ = (id) => document.getElementById(id);

function withToken(url) {
  if (!S.token) return url;
  return url + (url.includes('?') ? '&' : '?') + 'token=' + encodeURIComponent(S.token);
}

async function api(path, opts) {
  const res = await fetch(withToken(API + path), opts);
  if (res.status === 401) {
    throw new Error('令牌无效，请在「设置」里填写访问令牌');
  }
  if (!res.ok) {
    let detail = '';
    try { detail = (await res.json()).error || ''; } catch (_) {}
    throw new Error(detail || ('HTTP ' + res.status));
  }
  return res.json();
}

function toast(msg, ms) {
  const el = $('toast');
  el.textContent = msg;
  el.classList.add('show');
  clearTimeout(toast._t);
  toast._t = setTimeout(() => el.classList.remove('show'), ms || 2200);
}

function fmtTime(sec) {
  if (!sec) return '--';
  const d = new Date(sec * 1000);
  const now = new Date();
  const sameDay = d.toDateString() === now.toDateString();
  const hm = String(d.getHours()).padStart(2, '0') + ':' +
             String(d.getMinutes()).padStart(2, '0') + ':' +
             String(d.getSeconds()).padStart(2, '0');
  return sameDay ? hm : (d.getMonth() + 1) + '月' + d.getDate() + '日 ' + hm;
}

function fmtBytes(n) {
  if (!n && n !== 0) return '--';
  if (n < 1024) return n + 'B';
  if (n < 1048576) return (n / 1024).toFixed(0) + 'KB';
  if (n < 1073741824) return (n / 1048576).toFixed(1) + 'MB';
  return (n / 1073741824).toFixed(2) + 'GB';
}

function fmtSecs(s) {
  s = Math.max(0, Math.round(s || 0));
  if (s < 60) return s + '秒';
  if (s < 3600) return Math.floor(s / 60) + '分' + (s % 60 ? (s % 60) + '秒' : '');
  return Math.floor(s / 3600) + '小时' + Math.floor((s % 3600) / 60) + '分';
}

/* ------------------------------------------------------------------ */
/* 顶栏 / 状态                                                         */
/* ------------------------------------------------------------------ */

function setConn(state) {
  const dot = $('conn');
  dot.className = 'dot ' + state;
  S.online = state === 'on';
}

function renderStatus(st) {
  setConn('on');

  const b = st.battery && st.battery.ratio;
  const pct = (b === null || b === undefined) ? null : Math.round(b * 100);
  $('batt-val').textContent = pct === null ? '--' : pct + '%';
  const fill = $('batt-fill');
  fill.style.width = pct === null ? '0%' : Math.max(3, Math.min(100, pct)) + '%';
  fill.className = pct === null ? '' : (pct <= 15 ? 'crit' : (pct <= 25 ? 'low' : ''));

  const dockState = (st.dock && st.dock.state) || '';
  const rec = st.recorder || {};
  const cam = st.camera || {};

  // 第二行：一句话说清机器人在干嘛
  let line = '待机';
  if (dockState === 'DOCKING') line = '正在回充…';
  else if (dockState === 'DOCKED') line = '充电中';
  else if (dockState === 'ALIGNING') line = '对准充电桩…';
  else if (dockState === 'UNDOCKING') line = '离桩中…';
  else if (dockState === 'DISABLED') line = '回充已禁用';
  else if (st.person && st.person.present) line = '跟随中';
  else if (dockState === 'IDLE') line = '待机';
  $('state-text').textContent = line;

  $('fps-text').textContent = cam.has_frame
    ? `${cam.width}×${cam.height} · ${cam.fps.toFixed(1)}fps`
    : '无画面';

  // 贴片
  const chips = [];
  chips.push(`<span class="chip ${rec.recording ? 'ok' : 'bad'}">录像 <b>${rec.recording ? '开' : '关'}</b></span>`);
  if (rec.segments !== undefined) {
    chips.push(`<span class="chip">分段 <b>${rec.segments}</b> · ${rec.total_mb || 0}MB</span>`);
  }
  if (st.storage && st.storage.free_mb !== undefined) {
    chips.push(`<span class="chip">剩余 <b>${(st.storage.free_mb / 1024).toFixed(1)}GB</b></span>`);
  }
  chips.push(`<span class="chip ${st.person && st.person.present ? 'ok' : ''}">人 <b>${st.person && st.person.present ? '在' : '无'}</b></span>`);
  if (st.motion && st.motion.source) {
    chips.push(`<span class="chip ${st.motion.source === 'stop' ? '' : 'busy'}">运动源 <b>${st.motion.source}</b></span>`);
  }
  if (rec.events !== undefined) {
    chips.push(`<span class="chip">事件 <b>${rec.events}</b></span>`);
  }
  $('chips').innerHTML = chips.join('');

  $('rec-flag').classList.toggle('on', !!rec.recording);

  // 设置页的设备信息
  //
  // 「地址」用 location.host（浏览器实际访问的地址）而不是 st.device.lan_ip：
  // 后者是服务端进程视角的本机 IP，跑在 Docker 里时会是容器地址 172.17.x.x，
  // 手机根本连不上，显示出来只会让人以为配错了。location.host 永远是对的。
  const kv = [
    ['设备', st.device.name],
    ['地址', location.host],
    ['运行时长', fmtSecs(st.device.uptime_s)],
    ['相机', cam.has_frame ? `${cam.width}×${cam.height} @ ${cam.fps.toFixed(1)}fps` : '无画面'],
    ['录像分段', (rec.segments || 0) + ' 段 / ' + (rec.total_mb || 0) + 'MB'],
    ['存储剩余', st.storage && st.storage.free_mb !== undefined
      ? (st.storage.free_mb / 1024).toFixed(1) + 'GB / ' + (st.storage.total_mb / 1024).toFixed(1) + 'GB' : '--'],
    ['回充状态', dockState || '--'],
    ['联系人', (st.contacts || 0) + ' 个'],
  ];
  $('device-info').innerHTML = kv
    .map(([k, v]) => `<dt>${k}</dt><dd>${v}</dd>`).join('');
}

async function pollStatus() {
  try {
    const st = await api('/status');
    renderStatus(st);
  } catch (e) {
    setConn('off');
    $('state-text').textContent = e.message.slice(0, 40);
  }
}

/* ------------------------------------------------------------------ */
/* 实时画面：fetch 流 + JPEG 切片                                       */
/* ------------------------------------------------------------------ */

function makeJpegSplitter(onFrame) {
  let buf = new Uint8Array(0);
  return function push(chunk) {
    const nb = new Uint8Array(buf.length + chunk.length);
    nb.set(buf);
    nb.set(chunk, buf.length);
    buf = nb;

    for (;;) {
      // 找 SOI
      let start = -1;
      for (let i = 0; i + 1 < buf.length; i++) {
        if (buf[i] === 0xFF && buf[i + 1] === 0xD8) { start = i; break; }
      }
      if (start < 0) {
        // 没找到帧头；只留最后 1 字节（可能是 0xFF）
        buf = buf.slice(Math.max(0, buf.length - 1));
        return;
      }
      // 找 EOI（熵编码段的字节填充保证 FFD9 只出现在帧尾）
      let end = -1;
      for (let i = start + 2; i + 1 < buf.length; i++) {
        if (buf[i] === 0xFF && buf[i + 1] === 0xD9) { end = i + 1; break; }
      }
      if (end < 0) {
        buf = buf.slice(start);
        return;
      }
      const frame = buf.subarray(start, end + 1);
      buf = buf.slice(end + 1);
      onFrame(frame);
    }
  };
}

const canvas = $('screen');
const ctx = canvas.getContext('2d', { alpha: false });

const pushJpeg = makeJpegSplitter((bytes) => {
  const blob = new Blob([bytes], { type: 'image/jpeg' });
  if (window.createImageBitmap) {
    createImageBitmap(blob).then((bmp) => {
      if (canvas.width !== bmp.width || canvas.height !== bmp.height) {
        canvas.width = bmp.width;
        canvas.height = bmp.height;
      }
      ctx.drawImage(bmp, 0, 0);
      bmp.close && bmp.close();
      onFrameDrawn();
    }).catch(() => {});
  } else {
    // 老浏览器：走 Image + object URL
    const url = URL.createObjectURL(blob);
    const img = new Image();
    img.onload = () => {
      if (canvas.width !== img.width || canvas.height !== img.height) {
        canvas.width = img.width;
        canvas.height = img.height;
      }
      ctx.drawImage(img, 0, 0);
      URL.revokeObjectURL(url);
      onFrameDrawn();
    };
    img.onerror = () => URL.revokeObjectURL(url);
    img.src = url;
  }
});

function onFrameDrawn() {
  const now = performance.now();
  S.frameCount++;
  // dt <= 0 必须挡掉：一帧的 SOI/EOI 被切成两段送来、或者两帧在同一毫秒内画完，
  // 都会让 1000/dt 变成 Infinity，之后 fpsShown 就永久是 Infinity 了（实测踩到）。
  if (S.lastFrameAt && now > S.lastFrameAt) {
    const inst = 1000 / (now - S.lastFrameAt);
    S.fpsShown = S.fpsShown ? (S.fpsShown * 0.8 + inst * 0.2) : inst;
  }
  S.lastFrameAt = now;
  $('screen-overlay').classList.add('hide');
  $('screen-meta').textContent =
    `${canvas.width}×${canvas.height} · ${S.fpsShown.toFixed(1)}fps`;
}

function stopStream() {
  if (S.streamAbort) {
    try { S.streamAbort.abort(); } catch (_) {}
    S.streamAbort = null;
  }
  clearTimeout(S.streamTimer);
  S.streamTimer = null;
}

function scheduleReconnect(ms) {
  clearTimeout(S.streamTimer);
  S.streamTimer = setTimeout(startStream, ms);
}

async function startStream() {
  stopStream();
  const ctrl = new AbortController();
  S.streamAbort = ctrl;

  const overlay = $('screen-overlay');
  if (!S.lastFrameAt) overlay.textContent = '正在连接画面…';

  try {
    const res = await fetch(withToken(API + '/stream.mjpg'), {
      signal: ctrl.signal,
      cache: 'no-store',
    });
    if (res.status === 401) {
      overlay.classList.remove('hide');
      overlay.textContent = '令牌无效，请在「设置」里填写';
      setConn('off');
      return;
    }
    if (!res.ok || !res.body) throw new Error('HTTP ' + res.status);

    S.streamFails = 0;
    setConn('on');
    const reader = res.body.getReader();
    for (;;) {
      const { value, done } = await reader.read();
      if (done) break;
      if (value && value.length) pushJpeg(value);
    }
    // 服务端正常关流（比如重启）—— 稍等再连
    throw new Error('stream ended');
  } catch (e) {
    if (ctrl.signal.aborted) return;   // 主动断开，不重连
    S.streamFails++;
    const wait = Math.min(8000, 500 * Math.pow(1.6, Math.min(6, S.streamFails)));
    overlay.classList.remove('hide');
    overlay.textContent = S.streamFails > 2
      ? `画面断开，${Math.round(wait / 1000)}秒后重试…`
      : '正在连接画面…';
    if (S.streamFails >= 3) setConn('wait');
    scheduleReconnect(wait);
  }
}

/* ------------------------------------------------------------------ */
/* 事件 / 分段                                                         */
/* ------------------------------------------------------------------ */

function tagClass(type) {
  if (type === 'fall') return 'fall';
  if (type === 'manual') return 'manual';
  return '';
}

async function loadEvents() {
  const box = $('events');
  try {
    const data = await api('/events');
    const list = data.events || [];
    $('events-empty').style.display = list.length ? 'none' : 'block';

    box.innerHTML = list.map((ev) => {
      const hasVideo = ev.has_video;
      const thumb = hasVideo
        ? `<img src="${withToken(API + '/events/' + encodeURIComponent(ev.id) + '/thumb.jpg')}" loading="lazy" alt="">`
        : `<span class="ph">无画面</span>`;
      const dur = (ev.frames && ev.fps) ? (ev.frames / ev.fps).toFixed(0) + '秒' : '';
      return `<div class="item" data-event="${encodeURIComponent(ev.id)}">
        <div class="thumb">${thumb}</div>
        <div class="item-body">
          <div class="item-title">${ev.label || ev.type}</div>
          <div class="item-sub">${fmtTime(ev.wall)}${dur ? ' · ' + dur : ''}${hasVideo ? '' : ' · 无视频'}</div>
        </div>
        <span class="item-tag ${tagClass(ev.type)}">${ev.type}</span>
      </div>`;
    }).join('');

    box.querySelectorAll('[data-event]').forEach((el) => {
      el.addEventListener('click', () => openEvent(decodeURIComponent(el.dataset.event), list));
    });
  } catch (e) {
    $('events-empty').style.display = 'block';
    $('events-empty').textContent = '读取事件失败：' + e.message;
  }
}

async function loadClips() {
  try {
    const data = await api('/clips');
    const list = (data.clips || []).slice().reverse().slice(0, 20);
    $('clips-hint').textContent = `${(data.clips || []).length} 段`;
    $('clips').innerHTML = list.map((c) => `
      <div class="item" data-clip="${encodeURIComponent(c.file)}">
        <div class="item-body">
          <div class="item-title">${c.file}</div>
          <div class="item-sub">${fmtTime(c.mtime)} · ${fmtBytes(c.bytes)}</div>
        </div>
        <span class="item-tag">播放</span>
      </div>`).join('');
    $('clips').querySelectorAll('[data-clip]').forEach((el) => {
      el.addEventListener('click', () => {
        const name = decodeURIComponent(el.dataset.clip);
        openPlayer('录像 · ' + name,
          withToken(API + '/clips/' + encodeURIComponent(name)), '');
      });
    });
  } catch (e) {
    $('clips-hint').textContent = '读取失败';
  }
}

function openEvent(id, list) {
  const ev = (list || []).find((x) => x.id === id);
  if (!ev) return;
  if (!ev.has_video) {
    toast('这个事件没有视频（' + (ev.error || '未知原因') + '）');
    return;
  }
  const dur = (ev.frames && ev.fps) ? (ev.frames / ev.fps).toFixed(0) + '秒' : '';
  openPlayer(ev.label || ev.type,
    withToken(API + '/events/' + encodeURIComponent(id) + '/video'),
    `${fmtTime(ev.wall)} · ${dur}${ev.frames ? ' · ' + ev.frames + ' 帧' : ''}`);
}

function openPlayer(title, src, meta) {
  $('player-title').textContent = title;
  $('player-meta').textContent = meta || '';
  const v = $('player-video');
  v.src = src;
  $('player').hidden = false;
  v.play().catch(() => {});
}

function closePlayer() {
  const v = $('player-video');
  v.pause();
  v.removeAttribute('src');
  v.load();
  $('player').hidden = true;
}

/* ------------------------------------------------------------------ */
/* 联系人                                                              */
/* ------------------------------------------------------------------ */

function renderContacts() {
  const box = $('contacts');
  if (!S.contacts.length) {
    box.innerHTML = '<p class="empty">还没有紧急联系人</p>';
    return;
  }
  box.innerHTML = S.contacts.map((c, i) => `
    <div class="item" data-idx="${i}">
      <div class="item-body">
        <div class="item-title">${c.name}${c.primary ? ' <span class="item-tag">首选</span>' : ''}</div>
        <div class="item-sub">${c.phone}</div>
      </div>
      <button class="btn ghost small" data-del="${i}">删除</button>
    </div>`).join('');
  box.querySelectorAll('[data-del]').forEach((el) => {
    el.addEventListener('click', async (e) => {
      e.stopPropagation();
      const i = Number(el.dataset.del);
      const next = S.contacts.slice();
      next.splice(i, 1);
      await saveContacts(next);
    });
  });
}

async function loadContacts() {
  try {
    const data = await api('/contacts');
    S.contacts = data.contacts || [];
    renderContacts();
  } catch (e) {
    $('contacts').innerHTML = '<p class="empty">读取失败：' + e.message + '</p>';
  }
}

async function saveContacts(list) {
  try {
    const data = await api('/contacts', {
      method: 'PUT',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ contacts: list }),
    });
    S.contacts = data.contacts || [];
    renderContacts();
    toast('已保存 ' + S.contacts.length + ' 个联系人');
  } catch (e) {
    toast('保存失败：' + e.message);
  }
}

/* ------------------------------------------------------------------ */
/* 动作                                                                */
/* ------------------------------------------------------------------ */

async function takeSnapshot() {
  try {
    const res = await fetch(withToken(API + '/snapshot.jpg'), { cache: 'no-store' });
    if (!res.ok) throw new Error('HTTP ' + res.status);
    const blob = await res.blob();
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = 'lightdog_' + Date.now() + '.jpg';
    document.body.appendChild(a);
    a.click();
    a.remove();
    setTimeout(() => URL.revokeObjectURL(url), 4000);
    toast('已保存到相册/下载');
  } catch (e) {
    toast('截图失败：' + e.message);
  }
}

async function triggerEvent() {
  const btn = $('btn-event');
  btn.disabled = true;
  btn.textContent = '已提交…';
  try {
    await api('/events', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ type: 'manual', label: '手动标记', pre: 10, post: 10 }),
    });
    toast('已标记，约 10 秒后可在「事件」里看到');
  } catch (e) {
    toast('触发失败：' + e.message);
  } finally {
    setTimeout(() => { btn.disabled = false; btn.textContent = '标记事件'; }, 1500);
  }
}

/* ------------------------------------------------------------------ */
/* 视图切换 / 启动                                                     */
/* ------------------------------------------------------------------ */

function switchView(name) {
  document.querySelectorAll('.view').forEach((v) => {
    v.classList.toggle('active', v.id === 'view-' + name);
  });
  document.querySelectorAll('#tabs button').forEach((b) => {
    b.classList.toggle('active', b.dataset.view === name);
  });
  // hash 深链：手机上把某一页加书签 / 分享给家人时能直达
  try { history.replaceState(null, '', '#' + name); } catch (_) {}
  if (name === 'events') { loadEvents(); loadClips(); }
  if (name === 'settings') { loadContacts(); pollStatus(); }
}

function boot() {
  // 令牌：URL 优先，其次 localStorage
  const q = new URLSearchParams(location.search);
  if (q.get('token')) {
    S.token = q.get('token');
    try { localStorage.setItem(LS_TOKEN, S.token); } catch (_) {}
    // 存好后把 token 从地址栏抹掉，避免被截图/分享带出去
    history.replaceState(null, '', location.pathname);
  } else {
    try { S.token = localStorage.getItem(LS_TOKEN) || ''; } catch (_) {}
  }
  $('token-input').value = S.token;

  document.querySelectorAll('#tabs button').forEach((b) => {
    b.addEventListener('click', () => switchView(b.dataset.view));
  });
  $('btn-snap').addEventListener('click', takeSnapshot);
  $('btn-event').addEventListener('click', triggerEvent);
  $('btn-reload-events').addEventListener('click', () => { loadEvents(); loadClips(); });
  $('player-close').addEventListener('click', closePlayer);

  $('btn-add-contact').addEventListener('click', () => {
    const name = $('c-name').value.trim();
    const phone = $('c-phone').value.trim();
    if (!phone) { toast('请填电话号码'); return; }
    const next = S.contacts.concat([{ name: name || '紧急联系人', phone }]);
    $('c-name').value = '';
    $('c-phone').value = '';
    saveContacts(next);
  });

  $('btn-save-token').addEventListener('click', () => {
    S.token = $('token-input').value.trim();
    try { localStorage.setItem(LS_TOKEN, S.token); } catch (_) {}
    toast(S.token ? '令牌已保存，正在重连' : '已清空令牌');
    startStream();
    pollStatus();
  });

  // 回到前台时立刻刷新（手机省电会挂起流）
  document.addEventListener('visibilitychange', () => {
    if (!document.hidden) {
      pollStatus();
      if (!S.streamAbort || S.streamAbort.signal.aborted) startStream();
    }
  });

  startStream();
  pollStatus();
  S.statusTimer = setInterval(pollStatus, 2000);
  S.pollTimer = setInterval(() => {
    // 流断了但状态还在轮询：给一个保底重连
    if (!S.streamAbort || S.streamAbort.signal.aborted) {
      if (!S.streamTimer) startStream();
    }
  }, 5000);

  // 深链：#events / #settings 直接落到对应页签（默认实时）
  const want = (location.hash || '').replace('#', '');
  if (want === 'events' || want === 'settings') switchView(want);
}

document.addEventListener('DOMContentLoaded', boot);
