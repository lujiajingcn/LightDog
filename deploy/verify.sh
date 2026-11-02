#!/usr/bin/env bash
#
# LightDog 部署自检 —— 装完先跑这个，再考虑启动。
#
#   sudo ./deploy/verify.sh              # 全套（含 ROS 话题）
#   sudo ./deploy/verify.sh --no-ros     # 只查 unit 语法与 systemd 状态
#
# 检查分四层，任何一层报 FAIL 都会让退出码非零：
#   1. unit 层   —— 文件在不在、systemd-analyze 认不认、ExecStart 引用的路径存不存在
#   2. 进程层   —— 服务是否 active、有没有在重启循环里
#   3. 数据层   —— 关键话题有没有人发、HTTP 端口通不通、急停闭锁是什么状态
#   4. 外呼层   —— 配了 SIP 时 AMI 能不能登录、告警语音在不在（**不拨号**）
#
# 第 1 层不需要机器人跑起来，所以它能在「装完还没敢启动」时先抓拼写错误。
#
set -uo pipefail   # 故意不写 -e：一项失败要继续查完，不能停在第一个错

SELF_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NODES_CONF="$SELF_DIR/nodes.conf"
UNIT_DIR="/etc/systemd/system"
DATA_DIR="/data"
STATE_ESTOP="/var/lib/lightdog/estop"

CHECK_ROS=1
PORT=8000
FAIL=0
WARN=0

while [ $# -gt 0 ]; do
  case "$1" in
    --no-ros) CHECK_ROS=0; shift ;;
    --port)   PORT="${2:-8000}"; shift 2 ;;
    -h|--help) sed -n '2,16p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) printf '未知参数：%s\n' "$1" >&2; exit 2 ;;
  esac
done

ok()   { printf '  \033[32m✓\033[0m %s\n' "$*"; }
bad()  { printf '  \033[31m✗\033[0m %s\n' "$*"; FAIL=$((FAIL + 1)); }
warn() { printf '  \033[33m!\033[0m %s\n' "$*"; WARN=$((WARN + 1)); }
head1() { printf '\n\033[1m%s\033[0m\n' "$*"; }

# 从 p5_params.yaml 里取一个标量值。
#
# 不引 yaml 解析器：verify.sh 要在装完、还没 source workspace 的干净 shell 里跑，
# 依赖越少越不容易「检查脚本自己先崩」。但 `tr -d ' \r'` 这一招有个坑：
# 行尾注释会粘进值里 —— `sip_ami_host: 127.0.0.1   # 同机` 会变成
# `127.0.0.1#同机`，拿它去连 AMI 必然失败，而报出来的是「AMI 登录失败」。
# 所以先按 yaml 的规则去掉注释：`#` 前面必须是空白，否则密码里的 # 会被截断。
# 顺带注意：**不能**用 `tr -d ' '` 去空格 —— 那会把密码里的空格也吃掉
# （`p#ss word` → `p#ssword`），登录失败且完全看不出原因。只去首尾。
yaml_val() {
  sed -n "s/^[[:space:]]*$1:[[:space:]]*//p" "$2" | head -1 \
    | sed 's/[[:space:]]#.*$//' | tr -d "\"'\r" \
    | sed 's/^[[:space:]]*//; s/[[:space:]]*$//'
}

mapfile -t NAMES < <(grep -v '^[[:space:]]*#' "$NODES_CONF" \
                     | grep -v '^[[:space:]]*$' | cut -d'|' -f1)
mapfile -t ENABLED < <(grep -v '^[[:space:]]*#' "$NODES_CONF" \
                       | grep -v '^[[:space:]]*$' | awk -F'|' '$5=="yes"{print $1}')

# ---------------------------------------------------------------- 1. unit 层

head1 "1. unit 文件"

[ -f "$UNIT_DIR/lightdog.target" ] && ok "lightdog.target" || bad "lightdog.target 不存在"

MISSING=0
for n in "${NAMES[@]}"; do
  if [ -f "$UNIT_DIR/lightdog-$n.service" ]; then
    :
  else
    bad "lightdog-$n.service 不存在"; MISSING=$((MISSING + 1))
  fi
done
[ "$MISSING" -eq 0 ] && ok "${#NAMES[@]} 个节点 unit 全部就位"

if command -v systemd-analyze >/dev/null 2>&1; then
  # 这是唯一能在「还没启动」时抓到 ExecStart 写错、占位符没替换干净的检查
  ANALYZE="$(systemd-analyze verify "$UNIT_DIR"/lightdog-*.service \
             "$UNIT_DIR/lightdog.target" 2>&1 || true)"
  if [ -z "$ANALYZE" ]; then
    ok "systemd-analyze verify 通过"
  else
    warn "systemd-analyze 有输出（多数是 WantedBy 未 enable，可忽略）："
    printf '%s\n' "$ANALYZE" | sed 's/^/      /' | head -12
  fi
fi

# 占位符没替换干净是 install.sh 最容易犯的错，unit 里留着 @XXX@ 会静默跑偏
LEFTOVER="$(grep -l '@[A-Z_]\{3,\}@' "$UNIT_DIR"/lightdog-*.service 2>/dev/null || true)"
[ -z "$LEFTOVER" ] && ok "无未替换的占位符" || bad "占位符残留：$LEFTOVER"

# ExecStart 里的 workspace 与参数文件必须真实存在
WS_GUESS="$(sed -n 's/.*source \(.*\)\/install\/setup\.bash.*/\1/p' \
            "$UNIT_DIR/lightdog-mission.service" 2>/dev/null | head -1)"
if [ -n "$WS_GUESS" ]; then
  [ -f "$WS_GUESS/install/setup.bash" ] \
    && ok "workspace 可 source：$WS_GUESS" \
    || bad "unit 里写的 workspace 不存在：$WS_GUESS"
  MISSING_EXE=0
  while IFS='|' read -r n _desc exe params _rest; do
    [ -f "$WS_GUESS/install/lightdog/share/lightdog/config/$params" ] || {
      bad "参数文件缺失：$params（$n）"; MISSING_EXE=$((MISSING_EXE + 1)); }
  done < <(grep -v '^[[:space:]]*#' "$NODES_CONF" | grep -v '^[[:space:]]*$')
  [ "$MISSING_EXE" -eq 0 ] && ok "参数文件齐全"
else
  warn "读不出 workspace（unit 可能没装）"
fi

# ---------------------------------------------------------------- 2. 进程层

head1 "2. 服务状态"

if ! systemctl list-unit-files 'lightdog-*' >/dev/null 2>&1; then
  warn "systemctl 不可用（容器里？），跳过进程检查"
else
  for n in "${ENABLED[@]}"; do
    st="$(systemctl is-active "lightdog-$n.service" 2>/dev/null || true)"
    case "$st" in
      active)   ok "lightdog-$n  active" ;;
      inactive|failed|"")
        warn "lightdog-$n  ${st:-未启动}" ;;
      *)        warn "lightdog-$n  $st" ;;
    esac
  done

  # 反复重启是最难查的故障：服务是 active，但每隔 2 秒换一个 PID
  for n in "${ENABLED[@]}"; do
    nrestart="$(systemctl show -p NRestarts --value "lightdog-$n.service" 2>/dev/null || echo 0)"
    if [ "${nrestart:-0}" -gt 3 ]; then
      bad "lightdog-$n 已重启 ${nrestart} 次 —— 大概率在崩溃循环，看 journalctl -t lightdog-$n"
    fi
  done
fi

# ---------------------------------------------------------------- 3. 数据层

head1 "3. 运行时"

if [ -e "$STATE_ESTOP" ]; then
  warn "急停处于闭锁状态（$STATE_ESTOP 存在）—— 车不会动是刻意的。
      现场确认安全后执行：sudo lightdog-estop clear --yes
      （别只删文件：rm 只骗得过下次重启，正在跑的 twist_mux 仍锁着）"
else
  ok "急停未闭锁"
fi

for d in events clips snapshots; do
  [ -d "$DATA_DIR/$d" ] && ok "$DATA_DIR/$d" || bad "$DATA_DIR/$d 不存在（recorder 会写失败）"
done

if curl -s -m 3 --noproxy '*' -o /dev/null -w '%{http_code}' \
     "http://127.0.0.1:$PORT/api/v1/health" 2>/dev/null | grep -q '^200$'; then
  ok "HTTP :$PORT /api/v1/health -> 200"
else
  warn "HTTP :$PORT 无响应（服务没起，或端口被改过）"
fi

if [ "$CHECK_ROS" -eq 1 ]; then
  if ! command -v ros2 >/dev/null 2>&1; then
    warn "当前 shell 没有 ros2（先 source），跳过话题检查"
  else
    check_topic() {   # check_topic <话题> <说明>
      if timeout 6 ros2 topic echo "$1" --once --truncate-length 400 >/dev/null 2>&1; then
        ok "$1 有数据（$2）"
      else
        bad "$1 收不到（$2）"
      fi
    }
    check_topic /safety/heartbeat   "twist_mux 活着"
    check_topic /cmd_vel_muxed      "速度出口在发"
    check_topic /mission/state      "调度层活着"
    check_topic /alert/state        "告警状态机活着"
  fi
fi

# ---------------------------------------------------------------- 4. 外呼

head1 "4. 外呼通道（自建 SIP）"

PARAMS="$WS_GUESS/install/lightdog/share/lightdog/config/p5_params.yaml"
[ -f "$PARAMS" ] || PARAMS="$SELF_DIR/../lightdog/config/p5_params.yaml"

MODE="?"
SECRET=""
AMI_HOST=""
AMI_PORT=""
AMI_USER=""
if [ -f "$PARAMS" ]; then
  MODE="$(yaml_val call_mode "$PARAMS")"
  SECRET="$(yaml_val sip_ami_secret "$PARAMS")"
  # 这三个也要读：节点是按 yaml 里的 host/port/user 连的，自检若用 CLI 默认值，
  # 用户改过其中任何一个就会得到「AMI 登录失败」，而节点其实是好的。
  AMI_HOST="$(yaml_val sip_ami_host "$PARAMS")"
  AMI_PORT="$(yaml_val sip_ami_port "$PARAMS")"
  AMI_USER="$(yaml_val sip_ami_user "$PARAMS")"
fi

if [ -z "$SECRET" ]; then
  ok "未配置 sip_ami_secret（call_mode=$MODE → 只记录不拨号）"
else
  command -v asterisk >/dev/null 2>&1 \
    && ok "asterisk 已安装" || bad "没有 asterisk 命令（sudo apt install asterisk）"
  AST="$(systemctl is-active asterisk 2>/dev/null || true)"
  [ "$AST" = active ] && ok "asterisk active" || bad "asterisk 未运行（${AST:-unknown}）"

  CALL_PY="$SELF_DIR/../lightdog/lightdog_call.py"
  [ -f "$CALL_PY" ] || CALL_PY="$WS_GUESS/install/lightdog/lib/lightdog/lightdog_call.py"
  if [ -f "$CALL_PY" ]; then
    # --check 只登录、不拨号：自检脚本**不能**让家里人半夜接到测试电话。
    OUT="$(python3 "$CALL_PY" --check --secret "$SECRET" \
             --ami-host "${AMI_HOST:-127.0.0.1}" \
             --ami-port "${AMI_PORT:-5038}" \
             --user "${AMI_USER:-lightdog}" 2>&1)" \
      && ok "AMI 登录成功（${AMI_USER:-lightdog}@${AMI_HOST:-127.0.0.1}:${AMI_PORT:-5038}）" \
      || bad "AMI 登录失败（${AMI_USER:-lightdog}@${AMI_HOST:-127.0.0.1}:${AMI_PORT:-5038}）：$OUT"
  else
    warn "找不到 lightdog_call.py，跳过 AMI 登录检查"
  fi
  [ -f /var/lib/asterisk/sounds/lightdog/alarm.wav ] \
    && ok "告警语音已就绪（deploy/sip/make_prompt.sh）" \
    || warn "缺 /var/lib/asterisk/sounds/lightdog/alarm.wav —— 打通了也是静音"
fi

# ---------------------------------------------------------------- 结论

printf '\n'
if [ "$FAIL" -eq 0 ]; then
  printf '\033[32m通过\033[0m（%d 项警告）\n' "$WARN"
  exit 0
fi
printf '\033[31m%d 项失败\033[0m（%d 项警告）\n' "$FAIL" "$WARN"
printf '排查顺序：systemctl status lightdog.target → journalctl -t lightdog-<名字> -n 50\n'
exit 1
