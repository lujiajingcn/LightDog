#!/usr/bin/env bash
#
# 生成告警外呼里播放的语音（Asterisk `Playback(lightdog/alarm)`）。
#
#   sudo ./deploy/sip/make_prompt.sh                    # 默认话术
#   sudo TEXT="检测到老人摔倒，请立刻回家查看。" ./deploy/sip/make_prompt.sh
#
# 为什么要在本机渲染而不是用 Asterisk 的 SayDigits/Flite：
#   - Flite 只有英文，说中文的告警等于没说；
#   - 现场可能没有外网，云 TTS 不可用。
# 所以走 Piper（CPU 上实时倍速，树莓派 5 完全够）。
#
# Asterisk 要的是 8kHz / 单声道 / 16bit PCM 的 wav。直接喂 22kHz 的模型输出，
# 现象是「能播但声音又慢又尖」，很容易误判成 TTS 坏了。
#
set -euo pipefail

OUT="${1:-/var/lib/asterisk/sounds/lightdog/alarm.wav}"
TEXT="${TEXT:-检测到有人摔倒，请尽快回家查看。如果接到本电话，请立即联系家人。}"

command -v ffmpeg >/dev/null 2>&1 || { echo "缺 ffmpeg：sudo apt install ffmpeg" >&2; exit 1; }

mkdir -p "$(dirname "$OUT")"
TMP="$(mktemp --suffix=.wav)"
trap 'rm -f "$TMP" "$TMP.8k.wav"' EXIT

if command -v piper >/dev/null 2>&1; then
  MODEL="${PIPER_MODEL:-}"
  if [ -z "$MODEL" ]; then
    # set -euo pipefail 下，find 遇到不存在的搜索根会返回 1，head -1 提前
    # 退出还会让 find 吃 SIGPIPE(141)。任一情况都会让这条赋值直接终止脚本，
    # 连下面「找不到模型」的提示都打不出来。用 || true 兜住整条管道。
    MODEL="$(find /usr/share/piper /opt/piper "$HOME" -name 'zh_CN-*.onnx' 2>/dev/null | head -1 || true)"
  fi
  [ -n "$MODEL" ] || {
    echo "找不到中文 Piper 模型（zh_CN-*.onnx）。" >&2
    echo "下载：https://huggingface.co/rhasspy/piper-voices/tree/main/zh/zh_CN" >&2
    echo "然后 PIPER_MODEL=/path/zh_CN-huayan-medium.onnx $0" >&2
    exit 1
  }
  # shellcheck disable=SC2086
  echo "$TEXT" | piper --model "$MODEL" --output_file "$TMP" >/dev/null
elif command -v espeak-ng >/dev/null 2>&1; then
  echo "没装 piper，退回 espeak-ng（音质较差，但能听清）" >&2
  espeak-ng -v cmn -s 150 -w "$TMP" "$TEXT"
else
  echo "piper 和 espeak-ng 都没有，装一个：sudo apt install espeak-ng" >&2
  exit 1
fi

ffmpeg -y -loglevel error -i "$TMP" -ar 8000 -ac 1 -c:a pcm_s16le "$TMP.8k.wav"
install -m 0644 "$TMP.8k.wav" "$OUT"
echo "已生成：$OUT"
echo "手动试一通（会真的打给 1001）："
echo "  asterisk -rx 'channel originate PJSIP/1001 extension s@alarm'"
