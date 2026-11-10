#!/usr/bin/env bash
#
# LightDog 真机服务安装器 —— 把「仿真里手敲一串 bash」变成「开机自己起来」。
#
# 用法：
#   sudo ./deploy/install.sh                   # 探测 workspace，装全部真机节点
#   sudo ./deploy/install.sh --dry-run          # 只打印将生成的 unit，不落盘
#   sudo ./deploy/install.sh --start            # 装完直接拉起
#   sudo ./deploy/install.sh --nodes safety-mux,mission,alert
#   sudo ./deploy/install.sh --uninstall
#
# 为什么不是「一个 launch 一个 unit」：见 deploy/nodes.conf 顶部注释。
#
set -euo pipefail

SELF_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NODES_CONF="$SELF_DIR/nodes.conf"
UNIT_TPL="$SELF_DIR/lightdog.service.in"
TARGET_TPL="$SELF_DIR/lightdog.target.in"

UNIT_DIR="/etc/systemd/system"
ENV_DIR="/etc/lightdog"
JOURNAL_DIR="/etc/systemd/journald.conf.d"
DATA_DIR="/data"
REPO_SRC="$SELF_DIR/.."

ROS_DISTRO_ARG=""
WS_ARG=""
RUN_USER=""
ONLY_NODES=""
DO_START=0
DRY_RUN=0
JOURNAL_LIMIT="200M"
ACTION="install"

die()  { printf '\033[31m错误：%s\033[0m\n' "$*" >&2; exit 1; }
warn() { printf '\033[33m警告：%s\033[0m\n' "$*" >&2; }
info() { printf '  %s\n' "$*"; }
head1() { printf '\n\033[1m%s\033[0m\n' "$*"; }

usage() {
  sed -n '2,20p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
  exit 0
}

while [ $# -gt 0 ]; do
  case "$1" in
    --ros-distro) ROS_DISTRO_ARG="${2:-}"; shift 2 ;;
    --ws)         WS_ARG="${2:-}"; shift 2 ;;
    --user)       RUN_USER="${2:-}"; shift 2 ;;
    --nodes)      ONLY_NODES="${2:-}"; shift 2 ;;
    --journal-limit) JOURNAL_LIMIT="${2:-}"; shift 2 ;;
    --start)      DO_START=1; shift ;;
    --dry-run)    DRY_RUN=1; shift ;;
    --uninstall)  ACTION="uninstall"; shift ;;
    -h|--help)    usage ;;
    *) die "未知参数：$1（--help 看用法）" ;;
  esac
done

# ------------------------------------------------------------------ 环境探测

if [ "$DRY_RUN" -eq 0 ] && [ "$(id -u)" -ne 0 ]; then
  die "需要 root（要写 $UNIT_DIR 和 $ENV_DIR）。加 sudo 重跑，或先 --dry-run 预览"
fi

# sudo 下 $HOME 是 root 的，要回到真实用户的 home 才能找到 workspace
if [ -z "$RUN_USER" ]; then
  RUN_USER="${SUDO_USER:-$(id -un)}"
fi
# 查不到用户时 getent 返回 2，pipefail + set -e 会让这条赋值直接终止脚本，
# 下一行的友好报错永远打不出来。吞掉管道的非零退出码，交给下面的判空。
RUN_HOME="$(getent passwd "$RUN_USER" | cut -d: -f6 || true)"
[ -n "$RUN_HOME" ] || die "查不到用户 $RUN_USER 的 home"

if [ -z "$WS_ARG" ]; then
  # 按「install/setup.bash 且装了 lightdog 包」两个条件找，而不是猜目录名
  for c in "$RUN_HOME"/*/install/setup.bash "$RUN_HOME"/*/*/install/setup.bash; do
    [ -f "$c" ] || continue
    d="$(dirname "$(dirname "$c")")"
    if [ -d "$d/install/lightdog/lib/lightdog" ]; then WS_ARG="$d"; break; fi
  done
fi
[ -n "$WS_ARG" ] || die "没找到 ROS workspace。用 --ws /path/to/ws 指定"
[ -f "$WS_ARG/install/setup.bash" ] || die "$WS_ARG/install/setup.bash 不存在，那是源码目录不是 workspace"
[ -d "$WS_ARG/install/lightdog/lib/lightdog" ] \
  || die "$WS_ARG 里没有 lightdog 的 install 产物。先跑 colcon build --packages-select lightdog"

if [ -z "$ROS_DISTRO_ARG" ]; then
  for d in "${ROS_DISTRO:-}" jazzy humble iron rolling; do
    [ -n "$d" ] && [ -f "/opt/ros/$d/setup.bash" ] && { ROS_DISTRO_ARG="$d"; break; }
  done
fi
[ -n "$ROS_DISTRO_ARG" ] || die "找不到 ROS 发行版。用 --ros-distro jazzy 指定"
[ -f "/opt/ros/$ROS_DISTRO_ARG/setup.bash" ] || die "/opt/ros/$ROS_DISTRO_ARG/setup.bash 不存在"

PARAM_DIR="$WS_ARG/install/lightdog/share/lightdog/config"
[ -d "$PARAM_DIR" ] || die "$PARAM_DIR 不存在 —— 参数文件没装到 install space，重新 colcon build"

# ------------------------------------------------------------------ 卸载

if [ "$ACTION" = "uninstall" ]; then
  head1 "卸载 LightDog 服务"
  mapfile -t UNITS < <(grep -v '^[[:space:]]*#' "$NODES_CONF" \
                        | grep -v '^[[:space:]]*$' | cut -d'|' -f1)
  for n in "${UNITS[@]}"; do
    if [ -f "$UNIT_DIR/lightdog-$n.service" ]; then
      systemctl disable --now "lightdog-$n.service" >/dev/null 2>&1 || true
      rm -f "$UNIT_DIR/lightdog-$n.service"
      info "已删除 lightdog-$n.service"
    fi
  done
  systemctl disable lightdog.target >/dev/null 2>&1 || true
  rm -f "$UNIT_DIR/lightdog.target"
  rm -f /usr/local/bin/lightdog-estop
  rm -f /usr/local/lib/lightdog/lightdog-estop.py
  systemctl daemon-reload
  info "已删除 lightdog.target 与 lightdog-estop"
  printf '\n保留（要清自己删）：%s/env、/var/lib/lightdog、%s\n' "$ENV_DIR" "$DATA_DIR"
  exit 0
fi

# ------------------------------------------------------------------ 生成

esc() { printf '%s' "$1" | sed 's/[\\&|]/\\&/g'; }

render() {   # render <模板> <name> <desc> <exe> <params> <extra> <after> <restart> <stoptimeout>
  local tpl="$1" name="$2" desc="$3" exe="$4" params="$5" \
        extra="$6" after="$7" restart="$8" stoptimeout="$9"
  local after_units=""
  if [ -n "$after" ]; then
    for a in $(printf '%s' "$after" | tr ',' ' '); do
      after_units="$after_units lightdog-$a.service"
    done
  fi
  sed -e "s|@NAME@|$(esc "$name")|g" \
      -e "s|@DESC@|$(esc "$desc")|g" \
      -e "s|@EXE@|$(esc "$exe")|g" \
      -e "s|@PARAMS@|$(esc "$params")|g" \
      -e "s|@EXTRA@|$(esc "$extra")|g" \
      -e "s|@AFTER@|$(esc "$after_units")|g" \
      -e "s|@RESTART@|$(esc "$restart")|g" \
      -e "s|@STOP_TIMEOUT@|$(esc "$stoptimeout")|g" \
      -e "s|@WS@|$(esc "$WS_ARG")|g" \
      -e "s|@DISTRO@|$(esc "$ROS_DISTRO_ARG")|g" \
      -e "s|@RUN_USER@|$(esc "$RUN_USER")|g" \
      "$tpl"
}

is_selected() {
  if [ -z "$ONLY_NODES" ]; then return 0; fi
  case ",$ONLY_NODES," in *",$1,"*) return 0 ;; esac
  return 1
}

head1 "LightDog 服务安装"
info "workspace : $WS_ARG"
info "ROS       : $ROS_DISTRO_ARG"
info "运行用户  : $RUN_USER"
info "参数目录  : $PARAM_DIR"
[ -n "$ONLY_NODES" ] && info "仅安装    : $ONLY_NODES"
[ "$DRY_RUN" -eq 1 ] && info "模式      : dry-run（不落盘）"

ENABLED=()
SKIPPED=()

while IFS='|' read -r name desc exe params default extra after restart stoptimeout; do
  [ -n "$(printf '%s' "${name:-}" | tr -d '[:space:]')" ] || continue

  if ! is_selected "$name"; then SKIPPED+=("$name"); continue; fi
  [ -x "$WS_ARG/install/lightdog/lib/lightdog/$exe" ] \
    || die "可执行不存在：$WS_ARG/install/lightdog/lib/lightdog/$exe（$name）"
  [ -f "$PARAM_DIR/$params" ] || die "参数文件不存在：$PARAM_DIR/$params（$name）"

  unit="$(render "$UNIT_TPL" "$name" "$desc" "$exe" "$params" \
                  "$extra" "$after" "$restart" "$stoptimeout")"

  if [ "$DRY_RUN" -eq 1 ]; then
    printf '\n===== lightdog-%s.service =====\n%s\n' "$name" "$unit"
  else
    printf '%s\n' "$unit" > "$UNIT_DIR/lightdog-$name.service"
    chmod 644 "$UNIT_DIR/lightdog-$name.service"
    info "写入 lightdog-$name.service ($default)"
  fi

  # `no` 的只装不 enable —— 仿真专用节点（led / person_sim）真机不该自启
  [ "$default" = "yes" ] && ENABLED+=("lightdog-$name.service")
done < <(grep -v '^[[:space:]]*#' "$NODES_CONF" | grep -v '^[[:space:]]*$')

[ "${#ENABLED[@]}" -gt 0 ] || die "一个节点都没选中（--nodes 写错？）"

WANTS="${ENABLED[*]}"
target="$(sed -e "s|@WS@|$(esc "$WS_ARG")|g" -e "s|@WANTS@|$(esc "$WANTS")|g" "$TARGET_TPL")"

if [ "$DRY_RUN" -eq 1 ]; then
  printf '\n===== lightdog.target =====\n%s\n' "$target"
  printf '\n（dry-run 结束，什么都没写）\n'
  exit 0
fi

printf '%s\n' "$target" > "$UNIT_DIR/lightdog.target"
chmod 644 "$UNIT_DIR/lightdog.target"
info "写入 lightdog.target（${#ENABLED[@]} 个单元）"
if [ "${#SKIPPED[@]}" -gt 0 ]; then
  info "跳过：${SKIPPED[*]}"
fi

# ------------------------------------------------------------------ 环境文件

if [ ! -f "$ENV_DIR/env" ]; then
  mkdir -p "$ENV_DIR"
  cat > "$ENV_DIR/env" <<'EOF'
# LightDog 运行时环境（systemd 单元的 EnvironmentFile）
# 改完执行：sudo systemctl restart lightdog.target
#
# 多台车在同一个 WiFi 里时必须给不同值，否则会互相收到对方的话题
# ROS_DOMAIN_ID=7
#
# 默认 rmw_fastrtps_cpp。真机上 CycloneDDS 通常更省内存、发现更快
# RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
#
# 真机若把 TTS 换成别家，在这里改模板（节点参数 tts_command 也会读环境）
# LIGHTDOG_TTS=
EOF
  info "写入 $ENV_DIR/env"
else
  info "保留已有 $ENV_DIR/env"
fi

# ------------------------------------------------------------------ 数据目录

# recorder 会往 /data 下写 events / clips / snapshots，server 读 contacts.json。
# 这里只建目录和属主；已有内容一律不动。
mkdir -p "$DATA_DIR"
chown "$RUN_USER:$RUN_USER" "$DATA_DIR"
for d in events clips snapshots; do
  mkdir -p "$DATA_DIR/$d"
  chown "$RUN_USER:$RUN_USER" "$DATA_DIR/$d"
done
info "数据目录 $DATA_DIR 就绪（events/clips/snapshots）"

# ------------------------------------------------------------------ 急停控制台

# 真机上「解除急停」的唯一可用入口。没有它，`Restart=always` + 急停落盘
# 就等于「一旦按过急停，这辆车永久锁死」—— 因为清除状态文件只能骗过重启，
# 骗不过正在运行的 twist_mux。
mkdir -p /usr/local/lib/lightdog
install -m 644 "$SELF_DIR/lightdog_estop.py" \
        /usr/local/lib/lightdog/lightdog-estop.py
cat > /usr/local/bin/lightdog-estop <<WRAPPER
#!/bin/bash
# 由 deploy/install.sh 生成 —— 重新安装会覆盖，不要直接改
source "/opt/ros/$ROS_DISTRO_ARG/setup.bash" || exit 78
source "$WS_ARG/install/setup.bash" || exit 78
exec python3 /usr/local/lib/lightdog/lightdog-estop.py "\$@"
WRAPPER
chmod 755 /usr/local/bin/lightdog-estop
info "安装 /usr/local/bin/lightdog-estop"

# ------------------------------------------------------------------ journald 限额

# 32G 存储卡的机器人上，「journal 写满分区」比节点崩溃更常见，
# 而且它发生的时间点通常正是告警录像要落盘的时候。
if [ "$JOURNAL_LIMIT" != "0" ]; then
  mkdir -p "$JOURNAL_DIR"
  if [ ! -f "$JOURNAL_DIR/lightdog.conf" ]; then
    cat > "$JOURNAL_DIR/lightdog.conf" <<EOF
# 由 deploy/install.sh 生成。只为给 journal 设上限，
# 不改其它 journald 行为（没有 SystemKeepFree 等覆盖）。
[Journal]
SystemMaxUse=$JOURNAL_LIMIT
SystemMaxFileSize=50M
# 崩溃重启循环里的重复日志由单元自己的 LogRateLimit* 兜住
EOF
    info "journal 上限设为 $JOURNAL_LIMIT（$JOURNAL_DIR/lightdog.conf）"
    systemctl restart systemd-journald || warn "journald 重启失败，限额下次开机生效"
  else
    info "保留已有 $JOURNAL_DIR/lightdog.conf"
  fi
fi

# ------------------------------------------------------------------ 生效

systemctl daemon-reload
systemctl enable lightdog.target >/dev/null
info "已 enable lightdog.target（开机自启）"

if [ "$DO_START" -eq 1 ]; then
  head1 "启动服务"
  systemctl start lightdog.target
  sleep 3
  systemctl --no-pager --lines=0 status lightdog.target | head -20
  printf '\n看日志：sudo journalctl -t lightdog-safety-mux -f\n'
else
  head1 "下一步"
  info "1) 检查配置：sudo ./deploy/verify.sh"
  info "2) 启动服务：sudo systemctl start lightdog.target"
  info "3) 看状态  ：systemctl status lightdog.target"
  info "4) 看日志  ：sudo journalctl -t lightdog-mission -f"
  info "5) 停止全部：sudo systemctl stop lightdog.target"
fi
