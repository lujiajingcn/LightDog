#!/usr/bin/env bash
# LightDog P1 效果查看助手（在宿主机 Git Bash 里跑，操作容器 sim）
#
#   bash sim/p1.sh led 0.8          # 主灯亮度 0~1
#   bash sim/p1.sh led off          # 主灯全灭
#   bash sim/p1.sh indicator 0.5    # 底部指示灯
#   bash sim/p1.sh battery          # 读电池状态（等不到数据会提示）
#   bash sim/p1.sh sweep            # 亮度 0→1 分段扫一遍，肉眼看变化
#   bash sim/p1.sh status           # 容器/节点/话题/GUI 概览
#
# 浏览器看画面：http://127.0.0.1:6080/vnc.html
set -u

export MSYS_NO_PATHCONV=1

CONTAINER="${CONTAINER:-sim}"

# --- 找 docker：Git Bash 的 PATH 里常常没有 ---
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

# 容器内统一先 source 两个工作空间
ros() {
  "$DOCKER" exec "$CONTAINER" bash -lc \
    "source /opt/ros/jazzy/setup.bash >/dev/null 2>&1; \
     source /home/ros/linorobot2_ws/install/setup.bash >/dev/null 2>&1; \
     $1"
}

# 发一条指令。
#
# 为什么不用 `ros2 topic pub --once`：它的发布者常常在 DDS 发现完成之前就退出，
# 消息直接丢；而输出被重定向掉了，于是现象是「命令成功返回、灯没变」。
# 曾经改成 `-r 10 -t 6` 连发 6 条来绕过，但 `-t/--times` 是**消息条数**不是秒，
# 本机实际只跑 2~3Hz、要 3~6 秒才发完，一条命令会被执行好几次。
# 现在统一用 sim/pub_once.py：先等订阅者、只发一条、多活 1 秒完成 ACK/重传。
pub() {  # pub <topic> <type> <yaml-body>
  ros "timeout 25 python3 /sim/pub_once.py $1 $2 '$3'"
}

# 某个话题有几个订阅者。
#
# 这里不能靠「读回自己发的值」确认：`/led/brightness` 只有订阅者（lightdog_led），
# 没有任何节点会把它再发出来，echo 永远读不到 —— 于是「读不到」既可能是没发成功，
# 也可能是正常。查订阅者数量是这条链路上唯一诚实的确认手段：0 个订阅者
# 说明发出去也没人收（节点没起 / 话题名写错），这一点必须报出来。
topic_subs() {
  ros "timeout 8 ros2 topic info $1 2>/dev/null | sed -n 's/^Subscription count: //p' | head -1"
}

need_container() {
  if ! "$DOCKER" ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
    echo "容器 $CONTAINER 没在运行。先执行：" >&2
    echo "  cd sim && docker compose --env-file .env up -d" >&2
    exit 1
  fi
}

set_brightness() {
  local v="$1" n
  n=$(topic_subs /led/brightness)
  if [ "$n" = "0" ]; then
    echo "!! /led/brightness 没有任何订阅者，发了也不会亮" >&2
    echo "   检查 lightdog_led 是否在跑：bash sim/p1.sh status" >&2
    return 1
  fi
  # 预检过了也未必发得出去（发现还没收敛时会被判 0 订阅者），所以还要看返回值，
  # 否则会打出「!! 没有订阅者」+「亮度 -> 0.8」两条自相矛盾的行。
  pub /led/brightness std_msgs/msg/Float32 "{data: $v}" || return 1
  echo "主灯亮度 -> $v（订阅者 ${n:-?} 个）"
}

cmd="${1:-status}"

case "$cmd" in
  led|brightness)
    need_container
    v="${2:-}"
    if [ -z "$v" ]; then echo "用法: bash sim/p1.sh led 0.8 | off" >&2; exit 1; fi
    case "$v" in
      off) v=0.0 ;;
      on|full|max) v=1.0 ;;
    esac
    set_brightness "$v"
    ;;

  indicator)
    need_container
    v="${2:-}"
    if [ -z "$v" ]; then echo "用法: bash sim/p1.sh indicator 0.5" >&2; exit 1; fi
    n=$(topic_subs /led/indicator)
    if [ "$n" = "0" ]; then
      echo "!! /led/indicator 没有任何订阅者，发了也不会亮" >&2
      echo "   检查 lightdog_led 是否在跑：bash sim/p1.sh status" >&2
      exit 1
    fi
    pub /led/indicator std_msgs/msg/Float32 "{data: $v}" || exit 1
    echo "指示灯亮度 -> $v（订阅者 ${n:-?} 个）"
    ;;

  battery)
    need_container
    echo "读 /battery_state（bridge 建链慢，最多等 60 秒）..."
    out=$(ros "timeout 60 ros2 topic echo /battery_state --once" 2>/dev/null)
    if [ -z "$out" ]; then
      echo "没读到数据。多半是 gz 侧电池发布者还没被 bridge 发现，等 2~3 分钟再试。" >&2
      exit 1
    fi
    echo "$out" | grep -E 'voltage|current|charge|capacity:|percentage|power_supply_status'
    ;;

  sweep)
    need_container
    echo "亮度扫描（每档停 1.5 秒，看浏览器里的车身灯条）"
    for v in 0.0 0.2 0.4 0.6 0.8 1.0; do
      set_brightness "$v"
      sleep 1.5
    done
    echo "扫描结束，保持在 1.0。复位：bash sim/p1.sh led off"
    ;;

  status)
    need_container
    echo "=== 容器 ==="
    "$DOCKER" ps --format '{{.Names}} | {{.Status}} | {{.Ports}}' | grep "^$CONTAINER"
    echo
    echo "=== 关键节点 ==="
    ros "ros2 node list 2>/dev/null | grep -E 'lightdog_led|lightdog_bridge|ros_gz_bridge|slam_toolbox|rviz2'"
    echo
    echo "=== 关键话题 ==="
    ros "ros2 topic list 2>/dev/null | grep -E 'led|battery'"
    echo
    echo "=== GUI 窗口进程 ==="
    if ros "ps -eo cmd | grep -q '[g]z sim -g'"; then echo "运行中"; else echo "未运行（SHOW_GUI=0 或已退出）"; fi
    echo
    echo "=== 生效的车型 ==="
    ros "ros2 param get /lightdog_led led_visuals 2>&1 | tail -1"
    echo
    echo "浏览器打开 http://127.0.0.1:6080/vnc.html"
    ;;

  *)
    sed -n '2,14p' "$0" | sed 's/^# \{0,1\}//'
    exit 1
    ;;
esac
