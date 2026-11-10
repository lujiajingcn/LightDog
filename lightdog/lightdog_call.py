#!/usr/bin/env python3
"""LightDog 外呼通道：自建 SIP（Asterisk AMI）+ 命令模板兜底。

选型已定（D2）：**自建 SIP**，Asterisk 跑在本机或家里的服务器上，机器人通过
AMI 让 Asterisk 外呼，家属的手机/软电话作为 SIP 分机接听。

为什么是 AMI，而不是「拼一条 `asterisk -rx "channel originate ..."` 命令」：

- AMI 能拿到 `OriginateResponse`，于是「接通」「无人接听」「忙」「拥塞」是四个
  不同的结果，而不是「命令退出码 0」。这三者对产品的含义完全不同：
  无人接听要立刻打下一个联系人，接通了要留出说话的时间，忙则说明人在通话中。
- `-rx` 每次都要新起一个 asterisk 客户端进程，`subprocess.run` 又会把调用方
  卡住；AMI 只开一条 TCP，且可以非阻塞地等事件。
- 走 shell 会有注入面：号码和姓名是手机端可编辑的外部输入。

## 四条硬约束（每条都对应一次真实或差点发生的故障）

1. **拨号绝不能同步跑在 tick 里。** 原实现是 `subprocess.run(..., timeout=15)`
   直接在 5Hz 的 `_tick` 里跑：一次外呼把状态机卡住 15 秒，这段时间里老人自己
   站起来了也不会进 RESOLVED，事件视频窗口跟着错位。现在拨号全在后台线程，
   tick 只 `poll()` 收结果。
2. **「命令返回 0」不等于「有人接听」。** 见上。
3. **ack 之后必须挂断正在响的那一通。** 只做到「不再拨下一个」的话，家属手机
   会一直响到超时，而界面写着「已确认，不再重复呼叫」。挂断要按频道名前缀找
   （`PJSIP/1001-0000000a` 里的流水号是 Asterisk 分配的，拨号时不知道）。
4. **失败要能区分「链路不通」和「没人接」。** AMI 连不上是设备故障，值得报错；
   没人接是正常业务，不该刷 error。

## 模块边界

本文件**不 import rclpy**：它只认 socket 和参数，因此宿主机上可以直接跑、
也可以被离线回归用假 AMI 服务器完整测（见 tests/test_call.py）。

命令行自检（真机现场用，不需要 ROS）：

    # 只验 AMI 登录
    python3 lightdog_call.py --check --secret "$LIGHTDOG_SIP_SECRET"
    # 真拨一个分机，响 45 秒、接通后最多 20 秒
    python3 lightdog_call.py --dial 1001 --ring 45 --max-call 20 \
        --secret "$LIGHTDOG_SIP_SECRET"
"""

import json
import queue
import shlex
import socket
import subprocess
import threading
import time

# ---------------------------------------------------------------- 呼叫结果

ANSWERED = 'answered'        # 接通了（有人拿起电话）
PLACED = 'placed'            # 命令模板发出去了，但是否接听不可知
NO_ANSWER = 'no_answer'      # 响铃超时，没人接
BUSY = 'busy'                # 对端忙（正在通话）
CONGESTION = 'congestion'    # 中继/分机不可达
OFFHOOK = 'offhook'          # 对端摘机但没应答（传真/语音信箱之类）
HANGUP = 'hangup'            # 未接通即挂断
FAILED = 'failed'            # AMI 报错 / 连接失败
TIMEOUT = 'timeout'          # 等 OriginateResponse 超时（Asterisk 没回事件）
CANCELLED = 'cancelled'      # 本地主动挂断（ack / 复位）

#: 给日志和手机端看的中文说明。键必须覆盖上面全部取值（一致性测试会查）。
OUTCOME_TEXT = {
    ANSWERED: '已接通',
    PLACED: '已发出外呼',
    NO_ANSWER: '无人接听',
    BUSY: '对方忙',
    CONGESTION: '线路不可达',
    OFFHOOK: '对方摘机未应答',
    HANGUP: '未接通即挂断',
    FAILED: '外呼失败',
    TIMEOUT: '等待应答超时',
    CANCELLED: '已取消',
}

#: Asterisk `OriginateResponse` 的 Reason 字段是 AST_CONTROL_* 子类型，
#: 不是「退出码」。0/4 表示成功接了，3 是最后一帧还是 RINGING（=响铃超时）。
_REASON_OUTCOME = {
    0: ANSWERED,
    4: ANSWERED,
    1: HANGUP,
    3: NO_ANSWER,
    5: BUSY,
    6: OFFHOOK,
    7: OFFHOOK,
    8: CONGESTION,
}


class AmiError(Exception):
    """AMI 链路层错误（连不上 / 被关闭 / 应答格式不认识）。"""


class AmiAuthError(AmiError):
    """用户名/密码不对，或 manager.conf 没允许这个来源 IP。"""


class AmiTimeout(AmiError):
    """在期限内没等到需要的消息。"""


# ---------------------------------------------------------------- AMI 客户端

class AmiClient:
    """最小可用的 Asterisk Manager Interface 客户端。

    只实现三件事：Login / Originate(Async) / Hangup（含 CoreShowChannels 找
    频道）。够用就好 —— AMI 是个很大的接口，实现得越多，出问题时越难判断
    是自己的 bug 还是 Asterisk 的行为。
    """

    def __init__(self, host, port=5038, username='lightdog', secret='',
                 timeout=5.0, log=None):
        self.host = host
        self.port = int(port)
        self.username = username
        self.secret = secret
        self.timeout = float(timeout)
        self.log = log or (lambda *a, **k: None)
        self._sock = None
        self._buf = b''
        self._seq = 0
        # 读 Action 响应时先到的 Event 不能扔：async Originate 的
        # OriginateResponse 完全可能在 Response 之前到达。暂存下来给
        # wait_event/channels 消费，否则那一通会白等到超时。
        self._events = []

    # -- 连接 ---------------------------------------------------------

    def connect(self):
        try:
            self._sock = socket.create_connection(
                (self.host, self.port), timeout=self.timeout)
        except OSError as e:
            raise AmiError(f'连不上 AMI {self.host}:{self.port}：{e}') from e
        self._buf = b''
        self._events = []
        # 不专门去读 banner（`Asterisk Call Manager/5.0.3`）：
        # 它是不带空行结尾的单行，单独读会引入「到底有没有 banner」的分支。
        # 解析器会跳过没有冒号的行，banner 自然被忽略。
        resp = self.action({'action': 'login', 'username': self.username,
                            'secret': self.secret, 'events': 'call,originate'})
        if str(resp.get('response', '')).lower() != 'success':
            msg = resp.get('message', '未知原因')
            self.close()
            raise AmiAuthError(f'AMI 登录被拒绝：{msg}')
        return self

    def close(self):
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
        self._sock = None

    def __enter__(self):
        return self.connect()

    def __exit__(self, *exc):
        self.close()
        return False

    # -- 收发 ---------------------------------------------------------

    def _fill(self, deadline):
        """收一批数据进缓冲；返回 False 表示只是这次超时，可以继续等。"""
        remain = deadline - time.monotonic()
        if remain <= 0:
            raise AmiTimeout('AMI 等待超时')
        if self._sock is None:
            raise AmiError('AMI 未连接')
        self._sock.settimeout(min(remain, 0.5))
        try:
            chunk = self._sock.recv(4096)
        except socket.timeout:
            return False
        except OSError as e:
            raise AmiError(f'AMI 读失败：{e}') from e
        if not chunk:
            # 对端关了连接。**不要**在这里重连：上层要根据这个失败决定
            # 「这一通算失败」，自动重连会把一次故障变成无限重试。
            raise AmiError('AMI 连接被对端关闭')
        self._buf += chunk
        return True

    def _read_message(self, deadline):
        """读一条以空行结尾的 AMI 消息，返回小写键的 dict。"""
        lines = []
        while True:
            while b'\n' in self._buf:
                raw, self._buf = self._buf.split(b'\n', 1)
                line = raw.decode('utf-8', 'replace').rstrip('\r')
                if not line:
                    if lines:
                        return _parse(lines)
                    continue        # 连续空行，跳过
                lines.append(line)
            if not self._fill(deadline):
                continue

    def _next_message(self, deadline):
        """先消费暂存的 Event，没有再上 socket 读。"""
        if self._events:
            return self._events.pop(0)
        return self._read_message(deadline)

    def _peek_deadline(self, timeout):
        return time.monotonic() + (self.timeout if timeout is None else float(timeout))

    def action(self, fields, timeout=None):
        """发一个 Action，读回它的 Response（途中的 Event 丢弃）。"""
        if self._sock is None:
            raise AmiError('AMI 未连接')
        # AMI 报文字段以 CRLF 分隔。号码 / 分机名是手机端可编辑的，混进一个
        # \r\n 就能在报文里插进第二个 Action（例如向任意号码 Originate）。
        # 上游已经过滤过联系人，这里再做一次纵深防御：谁绕过 HTTP 直接调
        # originate() 也注入不进来。
        for k, v in fields.items():
            if any(c in str(k) + str(v) for c in ('\r', '\n')):
                raise AmiError(f'AMI 字段含换行，拒绝发送：{k!r}')
        body = ''.join(f'{k}: {v}\r\n' for k, v in fields.items()) + '\r\n'
        try:
            self._sock.sendall(body.encode('utf-8'))
        except OSError as e:
            raise AmiError(f'AMI 写失败：{e}') from e
        return self._read_response(self._peek_deadline(timeout))

    def _read_response(self, deadline):
        """读到一条**Action 响应**为止，途中的 Event 直接丢掉。

        AMI 在同一条 TCP 上把「响应」和「事件」交错着发。刚登录、或者上一次
        外呼的 Hangup 事件恰好排在前面时，第一条消息就是个 Event。原实现把它
        当 Response 解析，`response` 字段为空 → 登录被判成
        「认证失败：未知原因」——密码明明是对的，日志却让人去查密码。

        这类误诊比直接报错难查得多：运维会去 manager.conf 里翻半天，
        而真正的问题是「读消息时没分辨类型」。

        只判 `'response' in msg` 还不够：`OriginateResponse` 这类 Event 自己就带
        `Response: Success/Failure` 头。Asterisk 把事件和 Action 响应同批送回来时，
        它会被误当成响应返回，随后 `wait_event('originateresponse')` 等一个已经被
        吃掉的事件直到超时。区分标志是 Event 一定有 `event` 字段，而 Action 的
        响应没有。被让过去的 Event **暂存**，不能丢：async Originate 的结果事件
        可能就排在它自己的 Action 响应前面。
        """
        while True:
            msg = self._read_message(deadline)
            if 'event' in msg:
                self._events.append(msg)
                continue
            if 'response' in msg:
                return msg

    def wait_event(self, event, action_id, timeout):
        """等一个指定事件；不相关的事件直接丢弃（AMI 会混着发很多）。"""
        deadline = self._peek_deadline(timeout)
        event = event.lower()
        while True:
            msg = self._next_message(deadline)
            if msg.get('event', '').lower() != event:
                continue
            got = msg.get('actionid', '')
            # 老版本 Asterisk 不带 ActionID，此时只能靠事件类型认。
            if action_id and got and got != action_id:
                continue
            return msg

    # -- 业务动作 -----------------------------------------------------

    def originate(self, channel, context, exten='s', ring_timeout=45.0,
                  event_margin=15.0):
        """异步外呼，等到 OriginateResponse 再返回 (outcome, reason, detail, 事件)。"""
        self._seq += 1
        action_id = f'lightdog-{int(time.time() * 1000)}-{self._seq}'
        resp = self.action({
            'action': 'originate',
            'channel': channel,
            'exten': exten,
            'context': context,
            'priority': '1',
            'timeout': str(int(max(1.0, ring_timeout) * 1000)),
            'async': 'true',
            'actionid': action_id,
        })
        if str(resp.get('response', '')).lower() != 'success':
            return FAILED, None, resp.get('message', 'Originate 被拒绝'), {}
        try:
            # Originate 排队后要等真正有结果才有事件；留一点余量，
            # 因为响铃超时是 Asterisk 侧计时，路上还有排队时间。
            ev = self.wait_event('originateresponse', action_id,
                                 ring_timeout + float(event_margin))
        except AmiTimeout:
            return TIMEOUT, None, '没等到 OriginateResponse', {}
        reason = _to_int(ev.get('reason'))
        # `Response: Success` 表示电话被接起来了；`Failure` + `Reason` 说明为什么
        # 没接。**不能**把 Failure 一律当成 FAILED —— 没人接是最常见的正常情况，
        # 记成「外呼失败」会让「链路坏了」和「家里没人」在日志里长得一模一样。
        if str(ev.get('response', '')).lower() == 'success':
            return ANSWERED, reason, '', ev
        return _REASON_OUTCOME.get(reason, FAILED), reason, ev.get('message', ''), ev

    def hangup(self, channel):
        resp = self.action({'action': 'hangup', 'channel': channel})
        return str(resp.get('response', '')).lower() == 'success'

    def channels(self, prefix):
        """按频道名前缀查活动频道。

        拨号时只写了 `PJSIP/1001`，Asterisk 会把频道叫成
        `PJSIP/1001-0000000a`。响亮名还没结束前拿不到完整名，所以 ack 要挂断
        时只能先列一遍再按前缀匹配。
        """
        resp = self.action({'action': 'coreshowchannels'})
        if str(resp.get('response', '')).lower() != 'success':
            return []
        found = []
        deadline = self._peek_deadline(self.timeout)
        while True:
            msg = self._next_message(deadline)
            ev = msg.get('event', '').lower()
            if ev == 'coreshowchannelscomplete':
                break
            if ev == 'coreshowchannel':
                ch = msg.get('channel', '')
                if ch.startswith(prefix):
                    found.append(ch)
        return found


def _parse(lines):
    out = {}
    for line in lines:
        if ':' not in line:
            continue                     # banner / 空行 / 畸形行
        k, v = line.split(':', 1)
        out[k.strip().lower()] = v.strip()
    return out


def _to_int(v):
    try:
        return int(str(v).strip())
    except (TypeError, ValueError):
        return None


# ---------------------------------------------------------------- 结果对象

class DialResult:
    """一次外呼的完整结果。字段名直接进 `/alert/state` 的 call_attempts。"""

    __slots__ = ('number', 'name', 'tag', 'outcome', 'reason', 'detail',
                 'channel', 'dial_wall', 'answer_wall', 'end_wall', 'hung_up_by')

    def __init__(self, number, name, tag='', outcome=FAILED, reason=None,
                 detail='', channel='', dial_wall=0.0, answer_wall=0.0,
                 end_wall=0.0, hung_up_by=''):
        self.number = number
        self.name = name
        self.tag = tag
        self.outcome = outcome
        self.reason = reason
        self.detail = detail
        self.channel = channel
        self.dial_wall = dial_wall or time.time()
        self.answer_wall = answer_wall
        self.end_wall = end_wall
        self.hung_up_by = hung_up_by

    @property
    def answered(self):
        return self.outcome == ANSWERED

    @property
    def placed(self):
        """这一通到底有没有真的发到网络上（区别于「链路就没通」）。

        `FAILED` / `TIMEOUT` / `CANCELLED` 都不算发出去过：前者是 AMI 或密码
        的问题，后者是被 ack 打断。手机端和历史记录的 `dialed` 字段用它，
        这样「AMI 密码错了」不会被记成「已经给三个联系人打过电话」。
        """
        return self.outcome not in (FAILED, TIMEOUT, CANCELLED)

    @property
    def talk_sec(self):
        if not self.answer_wall or not self.end_wall:
            return 0.0
        return max(0.0, self.end_wall - self.answer_wall)

    def text(self):
        t = OUTCOME_TEXT.get(self.outcome, self.outcome)
        if self.answered and self.talk_sec:
            t = f'{t} {self.talk_sec:.0f}s'
        if self.hung_up_by == 'ack':
            t = '已确认后挂断'
        if self.detail:
            t = f'{t}（{self.detail}）'
        return t

    def as_dict(self):
        return {
            'outcome': self.outcome,
            'outcome_text': self.text(),
            'reason': self.reason,
            'channel': self.channel,
            'dial_wall': round(self.dial_wall, 3),
            'end_wall': round(self.end_wall, 3) if self.end_wall else 0.0,
            'talk_sec': round(self.talk_sec, 1),
            'hung_up_by': self.hung_up_by,
        }


# ---------------------------------------------------------------- SIP 拨号器

class SipConfig:
    """参数容器。默认值与 p5_params.yaml 保持一致。"""

    __slots__ = ('host', 'port', 'username', 'secret', 'channel_template',
                 'context', 'exten', 'ring_timeout', 'max_call',
                 'connect_timeout', 'event_margin')

    def __init__(self, host='127.0.0.1', port=5038, username='lightdog',
                 secret='', channel_template='PJSIP/{number}',
                 context='alarm', exten='s', ring_timeout=45.0,
                 max_call=120.0, connect_timeout=5.0, event_margin=15.0):
        self.host = host
        self.port = int(port)
        self.username = username
        self.secret = secret
        self.channel_template = channel_template
        self.context = context
        self.exten = exten
        self.ring_timeout = float(ring_timeout)
        self.max_call = float(max_call)
        self.connect_timeout = float(connect_timeout)
        #: 等 OriginateResponse 时在 ring_timeout 之外多等多久。
        #: 单测里调到 0.3s，否则「Asterisk 不回事件」那个用例要跑 15 秒。
        self.event_margin = float(event_margin)

    def channel_of(self, number):
        return self.channel_template.format(number=number)


class _Job:
    __slots__ = ('number', 'name', 'tag', 'attempt', 'abort', 'why',
                 'thread', 'started')

    def __init__(self, number, name, tag='', attempt=1):
        self.number = number
        self.name = name
        self.tag = tag
        self.attempt = int(attempt)
        self.abort = threading.Event()
        self.why = ''
        self.thread = None
        self.started = time.time()


class SipDialer:
    """非阻塞拨号器：`dial()` 立刻返回，结果通过 `poll()` 取。

    一次只允许一通在途。不是偷懒：同时拨两个联系人会让「谁接通了」变得暧昧，
    而老年人家属接到电话的第一句话往往是「谁啊」，同时响铃只会互相打断。
    """

    def __init__(self, cfg, log=None):
        self.cfg = cfg
        self.log = log or (lambda *a, **k: None)
        self._q = queue.Queue()
        self._lock = threading.Lock()
        self._job = None
        self._closed = False
        self.stats = {'dialed': 0, 'answered': 0, 'failed': 0, 'cancelled': 0}

    # -- 对外 ---------------------------------------------------------

    @property
    def enabled(self):
        return bool(self.cfg.secret) and not self._closed

    @property
    def busy(self):
        with self._lock:
            return self._job is not None

    def describe(self):
        if not self.enabled:
            return 'SIP 未配置（缺 secret），仅记录'
        return f'SIP AMI {self.cfg.host}:{self.cfg.port} 上下文 {self.cfg.context}'

    def dial(self, number, name, tag='', attempt=1):
        """发起一通。返回 False 表示上一通还没结束（调用方下个周期再试）。"""
        number = str(number).strip()
        if not number or self._closed:
            return False
        with self._lock:
            if self._job is not None:
                return False
            job = _Job(number, name, tag, attempt)
            self._job = job
            self.stats['dialed'] += 1
            t = threading.Thread(target=self._run, args=(job,),
                                 name=f'lightdog-call-{number}', daemon=True)
            job.thread = t
            t.start()
        return True

    def poll(self):
        """取走已完成的结果（非阻塞）。"""
        out = []
        while True:
            try:
                out.append(self._q.get_nowait())
            except queue.Empty:
                return out

    def hangup(self, why='ack'):
        """请求挂断当前这一通。

        工作线程的连接正忙着等 `OriginateResponse`，不能复用（AMI 允许流水线
        发包，但应答会和事件交错，简单客户端分不清哪个 Response 属于哪个
        Action）。所以另开一条 AMI 连接去按前缀挂断：响铃中的频道被挂断后，
        Asterisk 会给原连接发一个 Reason=1 的 OriginateResponse，工作线程自然
        收尾。这样「确认后 1 秒内停止响铃」不依赖主线程的轮询节奏。
        """
        with self._lock:
            job = self._job
        if job is None:
            return False
        job.why = why
        job.abort.set()
        if job.thread is not None:
            threading.Thread(target=self._abort_call, args=(job,),
                             name=f'lightdog-call-abort-{job.number}',
                             daemon=True).start()
        return True

    def _abort_call(self, job):
        """独立的挂断连接。频道可能还没建好，所以重试几次再放弃。"""
        cfg = self.cfg
        prefix = cfg.channel_template.format(number=job.number)
        client = AmiClient(cfg.host, cfg.port, cfg.username, cfg.secret,
                           timeout=cfg.connect_timeout, log=self.log)
        try:
            client.connect()
            for _ in range(6):                    # 约 2.4s，覆盖建立频道的时间
                chans = client.channels(prefix)
                if chans:
                    for ch in chans:
                        client.hangup(ch)
                    return
                if job.thread is None or not job.thread.is_alive():
                    return
                time.sleep(0.4)
        except AmiError as e:
            self.log(f'[call] 挂断时 AMI 不可用：{e}')
        finally:
            client.close()

    def close(self, timeout=3.0):
        self._closed = True
        self.hangup('shutdown')
        with self._lock:
            job = self._job
        if job is not None and job.thread is not None:
            job.thread.join(timeout)
        return True

    # -- 工作线程 -----------------------------------------------------

    def _run(self, job):
        cfg = self.cfg
        result = DialResult(job.number, job.name, job.tag,
                            channel=cfg.channel_of(job.number))
        client = AmiClient(cfg.host, cfg.port, cfg.username, cfg.secret,
                           timeout=cfg.connect_timeout, log=self.log)
        try:
            client.connect()
            outcome, reason, detail, ev = client.originate(
                result.channel, cfg.context, cfg.exten, cfg.ring_timeout,
                cfg.event_margin)
            result.outcome = outcome
            result.reason = reason
            result.detail = detail
            if ev.get('channel'):
                result.channel = ev['channel']
            if outcome == ANSWERED:
                result.answer_wall = time.time()
                self._converse(client, job, result)
            elif job.abort.is_set() and outcome in (TIMEOUT, HANGUP, NO_ANSWER):
                # 我们主动放弃的：别把「用户已确认」记成「无人接听」
                result.outcome = CANCELLED
                result.hung_up_by = job.why
        except AmiAuthError as e:
            result.outcome = FAILED
            result.detail = str(e)
            self.log(f'[call] AMI 认证失败：{e}')
        except AmiError as e:
            result.outcome = FAILED
            result.detail = str(e)
            self.log(f'[call] AMI 错误：{e}')
        except Exception as e:                     # 不能让线程静默死掉
            result.outcome = FAILED
            result.detail = f'{type(e).__name__}: {e}'
            self.log(f'[call] 未预期异常：{e}')
        finally:
            if job.abort.is_set() and not result.hung_up_by:
                result.hung_up_by = job.why
            if result.answer_wall and not result.end_wall:
                result.end_wall = time.time()
            if result.outcome in (FAILED, TIMEOUT):
                self.stats['failed'] += 1
            elif result.outcome == ANSWERED:
                self.stats['answered'] += 1
            elif result.outcome == CANCELLED:
                self.stats['cancelled'] += 1
            try:
                client.close()
            except Exception:
                pass
            with self._lock:
                self._job = None
            self._q.put(result)

    def _converse(self, client, job, result):
        """接通之后：让家属听告警语音，直到超时或被 ack 挂断。"""
        # 计时必须用 monotonic：answer_wall 是 wall clock（要写进记录给手机端看），
        # 拿它和 monotonic 比较会得到一个「永远没到」的 deadline —— 测试里表现
        # 为一通电话占住通道不放，真机上表现为「打完第一个就再也不打第二个」。
        deadline = time.monotonic() + self.cfg.max_call
        while time.monotonic() < deadline:
            if job.abort.wait(0.2):        # Event.wait 自带退避，不用自旋 sleep
                break
        result.end_wall = time.time()
        if job.abort.is_set():
            result.hung_up_by = job.why
        else:
            result.hung_up_by = 'timeout'
        self._safe_hangup(client, result.channel)

    def _safe_hangup(self, client, channel):
        try:
            if client.hangup(channel):
                return
            # 频道名对不上时（Asterisk 已经自己挂了、或者名字被改写），
            # 按前缀再找一遍。找不到就算了 —— 电话自己也会超时。
            for ch in client.channels(channel.rsplit('-', 1)[0]):
                client.hangup(ch)
        except AmiError as e:
            self.log(f'[call] 挂断失败（不影响告警记录）：{e}')


# ---------------------------------------------------------------- 命令模板

class CommandDialer:
    """兜底通道：把 `call_command` 当模板执行（外部语音 API 的 CLI）。

    和 SIP 通道共用同一套接口，但**只能知道命令是否成功退出**，拿不到「是否
    接听」。所以 outcome 固定为 `placed`，界面文案是「已发出外呼」而不是
    「已接通」—— 不把不确定说成确定，是这个项目里反复出现的一条原则。
    """

    def __init__(self, template, log=None, timeout=15.0):
        self.template = str(template).strip()
        self.log = log or (lambda *a, **k: None)
        self.timeout = float(timeout)
        self._q = queue.Queue()
        self._lock = threading.Lock()
        self._job = None
        self.stats = {'dialed': 0, 'answered': 0, 'failed': 0, 'cancelled': 0}

    @property
    def enabled(self):
        return bool(self.template)

    @property
    def busy(self):
        with self._lock:
            return self._job is not None

    def describe(self):
        return f'命令模板 {self.template}' if self.template else '未配置外呼命令'

    def dial(self, number, name, tag='', attempt=1):
        if not self.template:
            return False
        with self._lock:
            if self._job is not None:
                return False
            job = _Job(str(number), name, tag, attempt)
            self._job = job
            self.stats['dialed'] += 1
            t = threading.Thread(target=self._run, args=(job,),
                                 name=f'lightdog-cmd-{number}', daemon=True)
            job.thread = t
            t.start()
        return True

    def poll(self):
        out = []
        while True:
            try:
                out.append(self._q.get_nowait())
            except queue.Empty:
                return out

    def hangup(self, why='ack'):
        # 命令模板是一次性的短命进程，没有「挂断」这个动作。如实返回 False，
        # 让上层知道这一通不会被取消（手机端别显示「已挂断」）。
        return False

    def close(self, timeout=3.0):
        with self._lock:
            job = self._job
        if job is not None and job.thread is not None:
            job.thread.join(timeout)
        return True

    def _run(self, job):
        result = DialResult(job.number, job.name, job.tag, outcome=FAILED)
        try:
            # 先 shlex 切模板、再逐 token 替换占位符。顺序反了（先 format 再
            # shlex）会让外部输入参与命令行结构本身：姓名含不成对引号时
            # shlex.split 直接抛 ValueError 被吞成 FAILED、这通电话根本打不出去；
            # 含空格/引号还会把一段姓名撑成多个 argv。反过来切分后替换，号码/
            # 姓名永远各自落在单个 argv 元素里，既不撑破边界也不被拆成额外参数。
            subs = {
                '{phone}': str(job.number),
                '{name}': str(job.name),
                '{attempt}': str(job.attempt),
            }
            argv = []
            for tok in shlex.split(self.template):
                for key, val in subs.items():
                    tok = tok.replace(key, val)
                argv.append(tok)
            if not argv:
                raise ValueError('call_command 为空，无法外呼')
            # 绝不 shell=True：号码/姓名是外部输入
            r = subprocess.run(argv, capture_output=True,
                               timeout=self.timeout)
            if r.returncode == 0:
                result.outcome = PLACED
            else:
                result.detail = (f'rc={r.returncode} '
                                 f'{r.stderr.decode(errors="replace")[:120]}')
        except subprocess.TimeoutExpired:
            result.detail = f'命令超过 {self.timeout:.0f}s 未返回'
        except Exception as e:
            result.detail = f'{type(e).__name__}: {e}'
        finally:
            result.end_wall = time.time()
            if result.outcome == PLACED:
                self.stats['answered'] += 1
            else:
                self.stats['failed'] += 1
                self.log(f'[call] 命令外呼失败：{result.detail}')
            with self._lock:
                self._job = None
            self._q.put(result)


def make_dialer(mode, call_command='', sip=None, log=None):
    """按模式造拨号器。mode 已由节点解析过（log / sip / command）。"""
    if mode == 'sip' and sip is not None:
        return SipDialer(sip, log=log)
    if mode == 'command' and call_command:
        return CommandDialer(call_command, log=log)
    return None


# ---------------------------------------------------------------- 命令行自检

def _main(argv=None):
    import argparse
    import os

    ap = argparse.ArgumentParser(description='LightDog SIP 外呼自检')
    ap.add_argument('--ami-host', default='127.0.0.1')
    ap.add_argument('--ami-port', type=int, default=5038)
    ap.add_argument('--user', default='lightdog')
    ap.add_argument('--secret', default=os.environ.get('LIGHTDOG_SIP_SECRET', ''))
    ap.add_argument('--context', default='alarm')
    ap.add_argument('--exten', default='s')
    ap.add_argument('--channel-template', default='PJSIP/{number}')
    ap.add_argument('--dial', default='', help='要拨的分机号/号码')
    ap.add_argument('--ring', type=float, default=45.0)
    ap.add_argument('--max-call', type=float, default=20.0)
    ap.add_argument('--check', action='store_true', help='只验 AMI 登录')
    args = ap.parse_args(argv)

    if not args.secret:
        print('缺少 --secret 或环境变量 LIGHTDOG_SIP_SECRET')
        return 2

    cfg = SipConfig(host=args.ami_host, port=args.ami_port,
                    username=args.user, secret=args.secret,
                    channel_template=args.channel_template,
                    context=args.context, exten=args.exten,
                    ring_timeout=args.ring, max_call=args.max_call)
    if args.check:
        try:
            AmiClient(cfg.host, cfg.port, cfg.username, cfg.secret).connect().close()
        except AmiError as e:
            print(f'FAIL: {e}')
            return 1
        print('OK: AMI 登录成功')
        return 0
    if not args.dial:
        print('要么 --check，要么 --dial <号码>')
        return 2

    d = SipDialer(cfg, log=lambda m: print(m, flush=True))
    d.dial(args.dial, args.dial)
    print(f'已发起外呼 {cfg.channel_of(args.dial)} ...', flush=True)
    result = None
    while result is None:
        for r in d.poll():
            result = r
        time.sleep(0.2)
    print(json.dumps(result.as_dict(), ensure_ascii=False, indent=2))
    return 0 if result.answered else 1


if __name__ == '__main__':
    import sys
    sys.exit(_main())
