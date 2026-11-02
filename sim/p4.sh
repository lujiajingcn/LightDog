#!/usr/bin/env bash
# LightDog P4 助手：视频存储 + 手机互联（宿主机 Git Bash 操作容器 sim）
#
#   bash sim/p4.sh start             # 拉起录像 + 后端（entrypoint 已起时先杀再起）
#   bash sim/p4.sh stop              # 停掉 P4 两个节点
#   bash sim/p4.sh status            # 节点 / 录像状态 / 文件统计 / HTTP 健康
#   bash sim/p4.sh urls              # 打印手机该访问的地址（含令牌）
#   bash sim/p4.sh event [type]      # 触发一次事件切片（默认 manual）
#   bash sim/p4.sh snap              # 存一张快照
#   bash sim/p4.sh files             # 列出 data 目录里的录像/事件/快照
#   bash sim/p4.sh probe             # 用 curl 把所有 HTTP 接口打一遍（自检）
#   bash sim/p4.sh watch             # 盯 /recorder/state
#   bash sim/p4.sh log               # 看 P4 节点日志
#   bash sim/p4.sh clean             # 清空 data 目录（录像/事件/快照/联系人）
#
# 典型验收流程：
#   bash sim/p4.sh urls                ← 拿到手机地址，浏览器打开能看到画面
#   bash sim/p4.sh event fall          ← 立刻触发一次"跌倒"事件
#   # 等 10 秒（后置窗口），事件列表里应出现一条带视频的记录
#   bash sim/p4.sh files               ← 确认 events/ 下真的有 video.mp4
#   bash sim/p4.sh probe               ← 所有接口 200/206
set -u

export MSYS_NO_PATHCONV=1

CONTAINER="${CONTAINER:-sim}"
DATA_DIR="${LIGHTDOG_DATA_DIR:-/data}"
PORT="${PORT:-8000}"

ENV_FILE="$(cd "$(dirname "$0")" && pwd)/.env"
env_get() { grep -E "^$1=" "$ENV_FILE" 2>/dev/null | cut -d= -f2- | tr -d '\r' | sed 's/[[:space:]]*$//'; }

TOKEN="${LIGHTDOG_TOKEN:-$(env_get LIGHTDOG_TOKEN)}"

DOCKER="${DOCKER:-}"
if [ -z "$DOCKER" ]; then
  if command -v docker >/dev/null 2>&1; then
    DOCKER="docker"
  elif [ -x "/c/Program Files/Docker/Docker/resources/bin/docker.exe" ]; then
    DOCKER="/c/Program Files/Docker/Docker/resources/bin/docker.exe"
  else
    echo "找不到 docker，请把 docker.exe 加入 PATH，或用 DOCKER=<路径> 指定" >&2
    exit 1
  fi
fi

ros() {
  "$DOCKER" exec "$CONTAINER" bash -lc \
    "source /opt/ros/jazzy/setup.bash >/dev/null 2>&1; \
     source /home/ros/linorobot2_ws/install/setup.bash >/dev/null 2>&1; \
     $1"
}

# 用 sim/pub_once.py 发一条：`--once` 的发布进程常在 DDS 发现完成前退出，
# 消息会丢，而输出被重定向掉了 → 现象是「命令成功返回、什么都没发生」。
pub() {  # pub <topic> <type> <yaml-body>；返回 1 = 没人订阅，这条命令没送到
  # 只发一条：pub_once.py 会让 publisher 多活 1 秒完成 ACK/重传。
  # 为什么不再连发 6 条，见 sim/system.sh 里 send() 上面那段说明。
  #
  # 必须检查 subscribers：没有订阅者时发布「成功」返回，消息直接进了黑洞，
  # 脚本却照样打印「已下发」。实测真出现过（DDS 发现还没收敛就发了）。
  local out
  out=$(ros "timeout 25 python3 /sim/pub_once.py $1 $2 '$3'")
  echo "$out"
  if echo "$out" | grep -q 'subscribers=0'; then
    echo "!! $1 没有订阅者，这条命令没人收（对应节点没起来？）" >&2
    return 1
  fi
}

need_container() {
  if ! "$DOCKER" ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
    echo "容器 $CONTAINER 没在运行。先执行：" >&2
    echo "  cd sim && docker compose --env-file .env up -d" >&2
    exit 1
  fi
}

# 这个容器里 `pkill -f` 不生效（P3 踩过），必须按 PID 杀。
# awk 排除执行者自己的 PID：这条命令自己的命令行里就含 `lightdog_p4.launch.py`，
# 会被同一个 grep 匹配到，可能先把自己杀掉、后面的节点一个都不处理。
kill_p4() {
  # 停止分两段，顺序不能反：
  #   1) 先对整个进程组发 SIGTERM。ros2 launch 装着信号处理，会依次关掉子节点，
  #      rclpy 正常 shutdown 时会向 DDS 发「我走了」。直接 kill -9 不会发这条，
  #      对端只能等租约超时 —— 这段时间 `ros2 node list` 会一直展示**幽灵节点**
  #      （实测杀完立刻 list 看到两个 lightdog_mission，8 秒后自己变回一个）。
  #      幽灵让人根本无法判断「到底有没有起重复实例」。
  #   2) 再对剩下的顽固进程补 SIGKILL。
  # 另外两点也是为了不误伤自己：
  #   * awk -v me=$$ 排除执行这条命令的 shell（它的命令行里含有模式原文）；
  #   * 模式里用 \. 而不是裸点，让「模式原文」本身不构成一次匹配。
  ros "f() { ps -eo pid,args --no-headers | grep -E 'lightdog_(recorder|server)\\.py|lightdog_p4\\.launch\\.py' | grep -v grep | awk -v me=\$\$ '\$1 != me {print \$1}'; }; \
       pids=\$(f); if [ -n \"\$pids\" ]; then kill -TERM \$pids 2>/dev/null; sleep 2; fi; \
       pids=\$(f); if [ -n \"\$pids\" ]; then kill -9 \$pids 2>/dev/null; fi; sleep 1; true"
}

# 本机局域网 IPv4 列表。
#
# 不能"取第一个 IPv4"：这台机器上 ipconfig 排在前面的全是 WSL / Hyper-V / Docker
# 的虚拟网卡（172.21.x.1、192.168.134.1 …），手机根本连不上。所以全列出来，
# 让用户挑那个属于 WiFi/以太网的 —— 比猜错一个地址强。
#
# 取地址用 awk '{print $NF}' 而不是 sed 去切冒号：中文 Windows 的 ipconfig 输出是
# **GBK** 编码（"地址"三个字节不是合法 UTF-8），GNU sed 在 UTF-8 locale 下会直接
# 放弃替换，整条命令静默返回原行（踩过）。awk 按空白切字段不受编码影响。
host_ips() {
  ipconfig 2>/dev/null | tr -d '\r' \
    | grep -a "IPv4" \
    | awk '{print $NF}' \
    | grep -aE '^[0-9]+(\.[0-9]+){3}$' \
    | grep -av '^127\.' \
    | awk '!seen[$0]++'
}

host_ip() { host_ips | head -1; }

# 明显是虚拟网卡网段的先剔掉（Docker bridge / WSL），剩下的第一个当"首选"
#
# 再加一条经验规则：虚拟网卡（Hyper-V Default Switch / WSL / VMware）几乎都是
# 网段的 .1，而真实 WiFi/以太网地址极少以 .1 结尾 —— 实测这台机器上
# 172.21.112.1 / 192.168.134.1 / 192.168.245.1 全是虚拟的，
# 真正的 WiFi 是 192.168.131.182。
host_ip_preferred() {
  local all pref
  all=$(host_ips)
  pref=$(echo "$all" | grep -avE '^172\.(1[7-9]|2[0-9]|3[01])\.' | grep -avE '\.1$' | head -1)
  [ -z "$pref" ] && pref=$(echo "$all" | grep -avE '^172\.(1[7-9]|2[0-9]|3[01])\.' | head -1)
  [ -z "$pref" ] && pref=$(echo "$all" | head -1)
  [ -z "$pref" ] && pref="127.0.0.1"
  echo "$pref"
}

q() { if [ -n "$TOKEN" ]; then echo "?token=$TOKEN"; fi; }

# --noproxy '*' 是必须的：这台机器的 shell 环境里带着指向 WorkBuddy 沙箱代理的
# HTTP_PROXY/HTTPS_PROXY，curl 会把它套到 127.0.0.1 上，结果永远是 code=000。
#
# 输出丢弃用 NUL 而不是 /dev/null：本脚本 export 了 MSYS_NO_PATHCONV=1，
# Git Bash 不会把 /dev/null 转成 NUL 传过去，curl.exe 就会去写字面路径
# `/dev/null`（写失败、但不报错），于是 %{size_download} 恒为 0。
# 注意 CURL_OPTS 里**不能**带 -o NUL：curl_api 的调用方要读响应体。
CURL_OPTS=(-s -m 10 --noproxy '*')

curl_api() {
  # $1=path  $2...=额外 curl 参数
  local path="$1"; shift || true
  if [ -n "$TOKEN" ]; then
    curl "${CURL_OPTS[@]}" -H "X-Auth-Token: $TOKEN" "$@" "http://127.0.0.1:$PORT$path"
  else
    curl "${CURL_OPTS[@]}" "$@" "http://127.0.0.1:$PORT$path"
  fi
}

cmd="${1:-status}"

case "$cmd" in

  start)
    need_container
    kill_p4
    ros "mkdir -p $DATA_DIR"
    token_arg=""
    if [ -n "$TOKEN" ]; then token_arg="auth_token:=$TOKEN"; fi
    ros "nohup ros2 launch lightdog lightdog_p4.launch.py data_dir:=$DATA_DIR $token_arg \
         > /tmp/p4.log 2>&1 & echo started"
    echo "P4 已拉起（日志 /tmp/p4.log）。等 8 秒再查状态："
    sleep 8
    ros "ros2 node list 2>/dev/null | grep -E 'lightdog_(recorder|server)' || echo '  (还没出现在 node list)'"
    echo
    echo "手机访问：$(host_ip_preferred):$PORT   （完整列表见 bash sim/p4.sh urls）"
    ;;

  stop)
    need_container
    kill_p4
    echo "P4 节点已停"
    ;;

  clean)
    need_container
    echo "将清空 $DATA_DIR 下的 clips/ events/ snapshots/ contacts.json"
    ros "rm -rf $DATA_DIR/clips $DATA_DIR/events $DATA_DIR/snapshots $DATA_DIR/clips.jsonl $DATA_DIR/contacts.json; mkdir -p $DATA_DIR/clips $DATA_DIR/events $DATA_DIR/snapshots; echo done"
    echo "已清空（正在运行的 recorder 会在下一次写盘时自动重建目录）"
    ;;

  event)
    need_container
    type="${2:-manual}"
    case "$type" in
      fall) label="检测到跌倒" ;;
      manual) label="手动标记" ;;
      *) label="${3:-$type}" ;;
    esac
    # 走 HTTP：这条路径同时验证「后端 -> /recorder/event -> recorder 切片」整条链
    out=$(curl_api "/api/v1/events" -X POST -H 'Content-Type: application/json' \
          -d "{\"type\":\"$type\",\"label\":\"$label\",\"pre\":10,\"post\":10}" -w '\n%{http_code}')
    code=$(echo "$out" | tail -1)
    body=$(echo "$out" | sed '$d')
    if [ "$code" = "202" ]; then
      echo "已触发事件 $type。后置窗口 10 秒，之后事件列表里会出现带视频的记录。"
      echo "  查看：bash sim/p4.sh files"
    else
      echo "触发失败（HTTP $code）：$body" >&2
      echo "后端没起？ bash sim/p4.sh start" >&2
      exit 1
    fi
    ;;

  snap)
    need_container
    # 必须确认送达：没送到的话下面 ls -lt 照样能列出**上一张**快照，
    # 屏幕上一片"成功"，而这一张根本没生成。
    pub /recorder/snapshot std_msgs/msg/Empty '{}' || exit 1
    sleep 2
    ros "ls -lt $DATA_DIR/snapshots 2>/dev/null | head -3"
    ;;

  urls)
    need_container
    echo "=== 手机访问（手机要和电脑在同一个 WiFi）==="
    ips=$(host_ips)
    pref=$(host_ip_preferred)
    if [ -z "$ips" ]; then
      echo "  取不到本机 IPv4，请手工看 ipconfig"
      ips="127.0.0.1"
      pref="127.0.0.1"
    fi
    print_url() {
      local ip="$1" tag="$2"
      if [ -n "$TOKEN" ]; then
        echo "  http://$ip:$PORT/?token=$TOKEN$tag"
      else
        echo "  http://$ip:$PORT/$tag"
      fi
    }
    print_url "$pref" "   ← 首选"
    for ip in $ips; do
      [ "$ip" = "$pref" ] && continue
      print_url "$ip" ""
    done
    echo "  （多网卡很常见，哪个能打开就用哪个）"
    echo
    if [ -n "$TOKEN" ]; then
      echo "  实时流： /api/v1/stream.mjpg?token=$TOKEN"
      echo "  单帧：   /api/v1/snapshot.jpg?token=$TOKEN"
    else
      echo "  实时流： /api/v1/stream.mjpg"
      echo "  单帧：   /api/v1/snapshot.jpg"
      echo
      echo "  注意：未设令牌 —— 同一网络内谁都能看画面。真机务必设置 LIGHTDOG_TOKEN。"
    fi
    echo
    echo "=== 本机自检 ==="
    code=$(curl -s -m 5 --noproxy '*' -o NUL -w '%{http_code}' "http://127.0.0.1:$PORT/api/v1/health" 2>/dev/null)
    if [ "$code" = "200" ]; then
      echo "  health: HTTP 200"
    else
      echo "  连不上（HTTP ${code:-000}）—— 容器没起？或者后端节点挂了：bash sim/p4.sh log"
    fi
    ;;

  files)
    need_container
    ros "python3 - <<'PY'
import os, json, time
D = '$DATA_DIR'
def show(p, label):
    if not os.path.isdir(p):
        print(f'{label}: (无)'); return
    items = []
    for n in sorted(os.listdir(p)):
        fp = os.path.join(p, n)
        if os.path.isfile(fp):
            items.append((n, os.path.getsize(fp)))
    print(f'{label}: {len(items)} 项')
    for n, s in items[-8:]:
        print(f'    {n:34s} {s/1024:8.1f} KB')
    if len(items) > 8:
        print(f'    ... 还有 {len(items)-8} 项')

show(os.path.join(D, 'clips'), 'clips（环形分段）')
show(os.path.join(D, 'snapshots'), 'snapshots（快照）')

ed = os.path.join(D, 'events')
names = sorted(os.listdir(ed)) if os.path.isdir(ed) else []
print(f'events: {len(names)} 条')
for n in names[-5:]:
    mp = os.path.join(ed, n, 'meta.json')
    try:
        m = json.load(open(mp, encoding='utf-8'))
    except Exception:
        m = {}
    vp = os.path.join(ed, n, 'video.mp4')
    vs = os.path.getsize(vp)/1024 if os.path.exists(vp) else 0
    print(f\"    {n:38s} {m.get('frames',0):4d} 帧 {vs:8.1f} KB  {m.get('label','')}\")

cp = os.path.join(D, 'contacts.json')
if os.path.exists(cp):
    try:
        c = json.load(open(cp, encoding='utf-8'))
        c = c.get('contacts', c) if isinstance(c, dict) else c
        print(f'contacts: {len(c)} 个')
    except Exception:
        print('contacts: 解析失败')
else:
    print('contacts: (无)')
PY"
    ;;

  probe)
    need_container
    ip="127.0.0.1"
    echo "=== HTTP 自检 @ $ip:$PORT ==="
    probe() {
      local path="$1" label="$2" extra="${3:-}"
      local out code ctype
      size=""                       # 流分支不产出 size，不清空会串上一行的值
      if [ "$path" = "/api/v1/stream.mjpg" ]; then
        # 流不会自己结束，取 2 秒的头部
        out=$(curl -s -m 2 --noproxy '*' -o NUL -D - -H "X-Auth-Token: $TOKEN" "http://$ip:$PORT$path" 2>/dev/null | head -20)
        code=$(echo "$out" | head -1 | awk '{print $2}')
        ctype=$(echo "$out" | grep -i '^content-type' | tr -d '\r' | sed 's/.*: //')
      else
        out=$(curl -s -m 10 --noproxy '*' -o NUL -w '%{http_code}\n%{content_type}\n%{size_download}' \
              -H "X-Auth-Token: $TOKEN" $extra "http://$ip:$PORT$path" 2>/dev/null)
        code=$(echo "$out" | sed -n 1p)
        ctype=$(echo "$out" | sed -n 2p)
        size=$(echo "$out" | sed -n 3p)
      fi
      printf '  %-34s %-4s %-42s %s\n' "$path" "${code:-ERR}" "${ctype:0:42}" "${size:+${size}B}"
    }
    probe "/api/v1/health" ""
    probe "/api/v1/status" ""
    probe "/api/v1/snapshot.jpg" ""
    probe "/api/v1/stream.mjpg" ""
    probe "/api/v1/events" ""
    probe "/api/v1/clips" ""
    probe "/api/v1/contacts" ""
    probe "/api/v1/snapshots" ""
    probe "/" ""
    probe "/style.css" ""
    probe "/app.js" ""
    echo
    echo "=== Range 支持（iOS 播放录像的前提，必须是 206）==="
    # 用容器里的 python 找素材：宿主 Git Bash 没有 python，别在这儿踩空
    ev=$(ros "python3 -c \"
import os, glob
for d in sorted(glob.glob('$DATA_DIR/events/*/'), reverse=True):
    if os.path.exists(os.path.join(d, 'video.mp4')):
        print(os.path.basename(d.rstrip('/')))
        break
\"" 2>/dev/null | tr -d '\r')
    clip=$(ros "ls -1 $DATA_DIR/clips/*.mp4 2>/dev/null | tail -1" 2>/dev/null | tr -d '\r' | xargs -r basename)

    if [ -z "$ev" ] && [ -z "$clip" ]; then
      echo "  还没有录像素材。先 bash sim/p4.sh event fall，等 12 秒再试。"
    else
      for url in ${ev:+/api/v1/events/$ev/video} ${clip:+/api/v1/clips/$clip}; do
        out=$(curl -s -m 10 --noproxy '*' -o NUL -D - -H "Range: bytes=0-1023" -H "X-Auth-Token: $TOKEN" \
              "http://$ip:$PORT$url" 2>/dev/null)
        code=$(echo "$out" | head -1 | awk '{print $2}')
        cr=$(echo "$out" | grep -i '^content-range' | tr -d '\r' | sed 's/.*: //')
        printf '  %-46s %-4s %s\n' "$url" "$code" "$cr"
      done
    fi
    ;;

  watch)
    need_container
    echo "实时监控 /recorder/state（Ctrl+C 退出）："
    ros "timeout 60 ros2 topic echo /recorder/state std_msgs/msg/String --truncate-length 999999 2>/dev/null" \
      | grep --line-buffered data | sed 's/^data: //' \
      | while IFS= read -r line; do echo "  $line" | cut -c1-160; done
    ;;

  log)
    need_container
    ros "tail -60 /tmp/p4.log 2>&1"
    ;;

  status)
    need_container
    echo "=== 容器 ==="
    "$DOCKER" ps --format '{{.Names}} | {{.Status}} | {{.Ports}}' | grep "^$CONTAINER"
    echo
    echo "=== P4 节点 ==="
    ros "ros2 node list 2>/dev/null | grep -E 'lightdog_(recorder|server)' || echo '  (未启动)'"
    echo
    echo "=== /recorder/state ==="
    ros "timeout 6 ros2 topic echo /recorder/state std_msgs/msg/String --once --truncate-length 999999 2>/dev/null | grep data || echo '  (读不到)'"
    echo "=== HTTP ==="
    printf '  health  '
    curl_api "/api/v1/health" -m 5 -w '  [HTTP %{http_code}]\n' 2>/dev/null || echo "  (连不上，端口 $PORT 没映射或后端没起)"
    echo
    echo "=== data 目录 ==="
    ros "du -sh $DATA_DIR 2>/dev/null; \
         echo -n '  clips: '; ls $DATA_DIR/clips/*.mp4 2>/dev/null | wc -l; \
         echo -n '  events: '; ls -d $DATA_DIR/events/*/ 2>/dev/null | wc -l; \
         echo -n '  snapshots: '; ls $DATA_DIR/snapshots/*.jpg 2>/dev/null | wc -l"
    echo
    echo "=== 手机地址 ==="
    echo "  http://$(host_ip_preferred):$PORT/$(q)"
    echo "  （其余网卡地址见 bash sim/p4.sh urls）"
    ;;

  *)
    sed -n '2,21p' "$0" | sed 's/^# \{0,1\}//'
    exit 1
    ;;
esac
