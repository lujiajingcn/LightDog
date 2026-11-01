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

need_container() {
  if ! "$DOCKER" ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
    echo "容器 $CONTAINER 没在运行。先执行：" >&2
    echo "  cd sim && docker compose --env-file .env up -d" >&2
    exit 1
  fi
}

set_brightness() {
  local v="$1"
  ros "ros2 topic pub --once /led/brightness std_msgs/msg/Float32 '{data: $v}'" >/dev/null 2>&1
  echo "主灯亮度 -> $v"
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
    ros "ros2 topic pub --once /led/indicator std_msgs/msg/Float32 '{data: $v}'" >/dev/null 2>&1
    echo "指示灯亮度 -> $v"
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
