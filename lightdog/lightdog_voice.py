#!/usr/bin/env python3
"""LightDog 语音链路（ASR 进 / TTS 出）

## 为什么先把「链路」做完，而不是等麦克风

`Readme.md` 的第一条功能是「通过麦克风能听到指令」。但麦克风、唤醒词、ASR 模型
全都依赖硬件和算力平台。平台已定（树莓派 5，**没有 CUDA**），所以引擎选择也定了：
唤醒词 openWakeWord、ASR whisper.cpp `base` int8 或 sherpa-onnx paraformer、
TTS Piper `zh_CN-huayan`。三者在 Pi 5 的 4 核 A76 上都是 CPU 实时，不需要加速卡；
代价是不能用需要 GPU 的方案（faster-whisper CUDA / 云端大模型），见
`docs/hardware_readiness.md` §8.5。

所以这里把**不依赖硬件的那一半**先做成契约：

    外部 ASR ──/voice/command──> lightdog_voice ──/mission/command──> 调度层
                                              └─/light/mode──────> 自动灯
                                              └─/safety/estop────> 急停闸

    告警状态机 ──/alert/voice──> lightdog_voice ──/voice/say────> 外部 TTS
                                               └─(可选) tts_command 直接调引擎

真机上唯一要做的是：**把 ASR 的输出灌进 `/voice/command`，让某个 TTS 引擎订阅
`/voice/say`**。本节点本身不需要改一行 —— 这正是把硬件依赖关在话题边界外的意义。

## `/voice/command` 的格式

两种都收，按能解析的先试：

    {"text": "把灯打开", "intent": "light_on", "confidence": 0.92}
    {"intent": "light_on"}
    "把灯打开"                       ← 裸文本也行（调试用 echo 手发）

`intent` 字段优先级最高：ASR/NLU 那边如果已经分类好了，我们就不要再用关键词
猜一遍。只有没给 intent（或给了不认识的 intent）时才走 `match_intent()`。

## 关键词为什么是「宁可漏、不可错」

一个听错的词如果被理解成「过来」，机器人就会朝说话的人开过去；如果被理解成
「急停」，倒还好。所以：

- 只做**包含匹配**，不做模糊/拼音/编辑距离 —— 那类匹配在远场噪声下错得离谱。
- 顺序上把**语义更强**的放前面：`急停` 先于 `停`，`别跟` 先于 `跟`。
  否则「别跟着我」会被理解成「跟着我」，这是这套逻辑里最危险的一类错。
- 匹配不到就**什么都不做**，只记一条 info。绝不猜。

## 说话（TTS）

`/alert/voice` 是 P5 已经发出来的「要机器人说的话」。本节点把它转发到
`/voice/say`，并且：

- `tts_command` 非空时，按模板起子进程直接合成
  （例：`espeak-ng -v zh "{text}"`、`aplay /tmp/x.wav` 由脚本包住）。
  模板里的 `{text}` 会做 shell 安全替换 —— 用 `shlex.split` 切分而不是 `shell=True`。
- 相同的一句话在 `tts_dedupe`(1.5s) 内不重复播（告警状态机 1Hz 刷新，否则会
  把同一句话念 20 遍）。
- 文本截断到 200 字符、去掉换行 —— 它最终会进子进程参数，长度必须封顶。

## 话题

    订阅  /voice/command   std_msgs/String   ASR/NLU 的输出（外部）
    订阅  /alert/voice     std_msgs/String   告警状态机要机器人说的话
    发布  /voice/say       std_msgs/String   给 TTS 引擎的文本（外部）
    发布  /mission/command std_msgs/String   语义化指令
    发布  /light/mode      std_msgs/String   auto | on | off
    发布  /safety/estop    std_msgs/Bool     急停闭锁
"""

import json
import shlex
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, String

from lightdog_qos import SAFETY_QOS


# 意图表：顺序即优先级，**不要随意重排**。
#
# 「停」相关的必须排在「跟」前面：「别跟着我」同时包含「跟」和「别跟」，
# 先匹配到 follow 的话，机器人会朝你走过来 —— 这是最容易出人命的一条歧义。
RULES = (
    # 顺序即优先级，**每一条都是踩过的坑**，不要凭「看起来更整齐」重排。
    #
    # 1) 解锁必须排在加锁**前面**：「解除急停」里含「急停」、「解除刹车」里含
    #    「刹车」，顺序反了的话，唯一的解锁口令会被理解成再加一次锁 ——
    #    运营者喊三遍都解不开。反过来不会误判：「急停」不含「解除急停」。
    ('clear_estop', ('解除急停', '取消急停', '解除刹车')),
    ('estop', ('急停', '紧急停止', '紧急刹车', '刹车', 'emergency stop')),
    ('stop_follow', ('别跟', '不要跟', '不要再跟', '不用跟', '不跟',
                     '别跟着', '停止跟随', 'stop following')),
    # 2) 否定式必须排在它要否定的词**前面**。
    #    「不要充电」含「充电」（dock 的关键词），dock 排前面的话，
    #    喊「别充电」反而会把车派去充电 —— 和「别跟着我」同一类歧义。
    ('stop_dock', ('不用充电', '不要充电', '别充电', '不用回充', '别回充',
                   '取消充电')),
    # 3) 停止巡逻要排在 stop 之前：「停止巡逻」含「停止」，stop 先匹配的话
    #    只是停掉跟随，巡游照跑（车还在走）。也不能排在 patrol_on 之后：
    #    「别巡逻」「不用巡逻」含「巡逻」，会被理解成**开始**巡逻。
    ('patrol_off', ('停止巡逻', '不用巡逻', '别巡逻', '取消巡逻')),
    ('stop', ('停下', '停止', '别动', '站住', '不要动', 'stop', 'halt')),
    ('follow', ('过来', '跟着我', '跟我走', '跟上', '来一下', 'come here', 'follow me')),
    # '出来' 故意不收：中文里「你出来一下」通常是「过来」而不是「离桩」，
    # 把一句招呼理解成离桩动作，代价是一台正在充电的车自己开走。
    ('undock', ('离桩', '离开充电', 'undock')),
    ('dock', ('充电', '回充', '回去充电', '回桩', '去充电', 'charge', 'dock')),
    ('light_on', ('开灯', '把灯打开', '灯打开', '亮一点', '灯亮', 'light on')),
    ('light_off', ('关灯', '把灯关掉', '灯关掉', '暗一点', '灯灭', 'light off')),
    ('light_auto', ('自动灯光', '灯自动', '灯光自动', 'auto light')),
    ('patrol_on', ('开始巡逻', '去巡逻', '巡逻', 'patrol')),
    ('status', ('状态', '报告情况', '现在怎么样', 'status')),
)

# 意图 -> 发给调度层的命令。/light/* 和 /safety/estop 不走调度层，单独处理。
MISSION_CMD = {
    'follow': 'follow',
    'stop_follow': 'follow_off',
    # 「停下」必须是**全停**，不能只停跟随：车正在巡游/回充时喊「停下」，
    # 只发 follow_off 的话它照旧往前走 —— 这是用户最不能接受的一种「不听话」。
    'stop': 'stop',
    'stop_dock': 'cancel',
    'dock': 'dock',
    'patrol_on': 'patrol_on',
    'patrol_off': 'patrol_off',
    'status': 'status',
}

LIGHT_MODE = {
    'light_on': 'on',
    'light_off': 'off',
    'light_auto': 'auto',
}

KNOWN_INTENTS = {name for name, _ in RULES} | {'none'}


def normalize(text):
    """统一成小写、去掉首尾空白。只做这一层，别做更多「聪明」的处理。"""
    return ' '.join(str(text or '').split()).strip().lower()


def match_intent(text):
    """按 RULES 顺序做包含匹配，返回意图名；匹配不到返回 None。

    刻意不做编辑距离/拼音/词向量：远场噪声下那类匹配会把「关灯」听成「开灯」，
    而这里的每一条意图都会让一台几十公斤的机器真的动起来或真的灭灯。
    """
    t = normalize(text)
    if not t:
        return None
    for name, keys in RULES:
        for k in keys:
            if k in t:
                return name
    return None


def parse_command(payload):
    """把 /voice/command 的 data 解析成 (intent, text, confidence)。

    - 优先信任 JSON 里的 intent（ASR/NLU 那边已经分好类）
    - 没有 intent 或 intent 不认识 → 用 text 走关键词
    - 全是垃圾 → (None, '', 0.0)
    """
    text_in = ''
    conf = 0.0
    raw = str(payload or '').strip()
    if not raw:
        return None, '', 0.0

    obj = None
    if raw.startswith('{'):
        try:
            obj = json.loads(raw)
        except ValueError:
            obj = None

    if isinstance(obj, dict):
        text_in = str(obj.get('text') or obj.get('query') or '')
        try:
            conf = float(obj.get('confidence', 0.0) or 0.0)
        except (TypeError, ValueError):
            conf = 0.0
        intent = normalize(obj.get('intent', ''))
        if intent in KNOWN_INTENTS and intent != 'none':
            return intent, text_in, conf
    else:
        text_in = raw

    intent = match_intent(text_in)
    return intent, text_in, conf


class VoiceBridge(Node):

    def __init__(self):
        super().__init__('lightdog_voice')

        self.declare_parameter('command_topic', '/voice/command')
        self.declare_parameter('say_topic', '/voice/say')
        self.declare_parameter('alert_voice_topic', '/alert/voice')
        self.declare_parameter('tts_command', '')      # 空 = 只发话题 + 记日志
        self.declare_parameter('tts_dedupe', 1.5)      # 同一句话多久内不重复
        self.declare_parameter('tts_max_len', 200)
        self.declare_parameter('tts_timeout', 10.0)
        self.declare_parameter('estop_topic', '/safety/estop')
        # 同一条指令多久内只执行一次。
        #
        # 两个来源都需要它：
        #  * 外部 ASR 会把同一句话的中间结果连发多帧（「把灯打」「把灯打开」）；
        #  * sim/system.sh 为了对抗丢包会连发 6 条（-r 10 -t 6），每一条都会
        #    触发一次完整动作 —— 日志里出现 6 行「语音：patrol_off」，
        #    配上真机 TTS 就是同一句话说六遍。
        self.declare_parameter('command_dedupe', 0.8)

        p = self.get_parameter
        self.tts_command = str(p('tts_command').value).strip()
        self.tts_dedupe = max(0.0, float(p('tts_dedupe').value))
        self.tts_max_len = max(16, int(p('tts_max_len').value))
        self.tts_timeout = max(1.0, float(p('tts_timeout').value))
        self.command_dedupe = max(0.0, float(p('command_dedupe').value))

        self._last_say = ''
        self._last_say_at = 0.0
        self._last_cmd = None
        self._last_cmd_at = 0.0
        self._estop = False
        self._warned_tts = False
        # TTS 子进程必须后台跑：say() 在单线程 spin 的订阅回调里，
        # subprocess.run 卡在音频设备上最多 10s，期间 /voice/command
        # 和 /safety/estop 全都不被处理。同时只准一个 TTS 在跑。
        self._tts_lock = threading.Lock()

        self.create_subscription(
            String, str(p('command_topic').value), self._on_command, 10)
        self.create_subscription(
            String, str(p('alert_voice_topic').value), self._on_alert_voice, 10)

        self.pub_say = self.create_publisher(String, str(p('say_topic').value), 10)
        self.pub_mission = self.create_publisher(String, '/mission/command', 10)
        self.pub_light = self.create_publisher(String, '/light/mode', 10)
        self.pub_estop = self.create_publisher(
            Bool, str(p('estop_topic').value), SAFETY_QOS)

        self.get_logger().info(
            f'LightDog voice ready: in={p("command_topic").value} '
            f'out={p("say_topic").value} '
            f'TTS={"外部命令" if self.tts_command else "仅话题+日志"} '
            f'（ASR 把文本发到 {p("command_topic").value} 即可驱动全部功能）')

    # ---------- 输入 ----------

    def _on_command(self, msg: String):
        intent, text, conf = parse_command(msg.data)
        if intent is None:
            if normalize(text):
                self.get_logger().info(
                    f'语音：没听懂（text={text!r}），不执行任何动作')
            return

        # 去重按**原文**而不是意图：连发/中间结果都是同一句话，
        # 而「开灯」紧接着「关灯」是两句不同的话，必须都执行。
        key = normalize(text) or intent
        now = time.monotonic()
        if (key == self._last_cmd
                and (now - self._last_cmd_at) < self.command_dedupe):
            return
        self._last_cmd = key
        self._last_cmd_at = now

        self.get_logger().info(
            f'语音：{intent}'
            f'（text={text!r}' + (f', conf={conf:.2f}' if conf else '') + '）')
        self._act(intent)

    def _on_alert_voice(self, msg: String):
        self.say(msg.data)

    # ---------- 执行 ----------

    def _act(self, intent):
        if intent == 'estop':
            self._estop = True
            self.pub_estop.publish(Bool(data=True))
            self.get_logger().warn('语音触发急停：轮子已锁，需显式「解除急停」')
            self.say('已急停。')
            return

        if intent == 'clear_estop':
            # 语音能解急停是有意的：跌倒场景下，喊一声比摸手机快。
            # 但这意味着「急停」不能只靠语音，物理按钮/网页按钮要同时存在。
            self._estop = False
            self.pub_estop.publish(Bool(data=False))
            self.get_logger().info('语音解除急停')
            self.say('急停已解除。')
            return

        if intent in LIGHT_MODE:
            mode = LIGHT_MODE[intent]
            self.pub_light.publish(String(data=mode))
            self.get_logger().info(f'灯光模式 -> {mode}')
            return

        if intent == 'undock':
            # 离桩交给 dock_manager 自己的 UndockRobot：它知道桩类型和超时。
            self.pub_mission.publish(String(data='undock'))
            return

        cmd = MISSION_CMD.get(intent)
        if cmd:
            self.pub_mission.publish(String(data=cmd))

    # ---------- TTS ----------

    def say(self, text):
        """把一句话送给 TTS。永远先发话题，再考虑本地命令。"""
        s = ' '.join(str(text or '').split())[:self.tts_max_len]
        if not s:
            return
        now = time.monotonic()
        # 告警状态机 1Hz 刷新，同一句话会被反复推过来。去重是必须的，
        # 否则「正在联系女儿」会被念二十遍。
        if s == self._last_say and (now - self._last_say_at) < self.tts_dedupe:
            return
        self._last_say = s
        self._last_say_at = now

        self.pub_say.publish(String(data=s))
        self.get_logger().info(f'[TTS] {s}')

        if not self.tts_command:
            return
        if not self._tts_lock.acquire(blocking=False):
            self.get_logger().warn('上一条 TTS 还没结束，跳过本次（不阻塞回调）')
            return

        def _run_tts():
            try:
                # 用 shlex 切分，绝不 shell=True：文本来自告警链路，虽然是我们
                # 自己拼的，但一旦将来接了外部输入，shell 注入就是一次提权。
                argv = [a.replace('{text}', s)
                        for a in shlex.split(self.tts_command)]
                subprocess.run(argv, capture_output=True,
                               timeout=self.tts_timeout)
            except (OSError, ValueError, subprocess.SubprocessError) as e:
                if not self._warned_tts:
                    self._warned_tts = True
                    self.get_logger().error(
                        f'tts_command 执行失败（{e}），后续只发 /voice/say 话题')
            finally:
                self._tts_lock.release()

        threading.Thread(target=_run_tts, daemon=True).start()


def main(args=None):
    rclpy.init(args=args)
    node = VoiceBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
