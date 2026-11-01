#!/usr/bin/env bash
# LightDog P3 助手：自动回充（宿主机 Git Bash 操作容器 sim）
#
#   bash sim/p3.sh start             # 拉起 P3 两个节点（aruco 检测 + 低电状态机）
#   bash sim/p3.sh spawn             # 在场景里生成充电底座（带重试）
#   bash sim/p3.sh stop              # 停掉 P3 节点
#   bash sim/p3.sh status            # 状态机/电量/marker/位姿 概览
#   bash sim/p3.sh dock              # 手动发起回充（不等低电）
#   bash sim/p3.sh undock            # 手动离桩
#   bash sim/p3.sh cancel            # 取消回充
#   bash sim/p3.sh battery 0.15      # 注入模拟电量比（触发低电分支）
#   bash sim/p3.sh battery off       # 取消注入，回到真实电池
#   bash sim/p3.sh pose              # 机器人 + 底座在 gz 里的真实位姿
#   bash sim/p3.sh log               # 看 P3 节点日志
#   bash sim/p3.sh watch             # 盯状态机 + 仲裁源 + 速度
#
# 典型验收流程：
#   bash sim/p3.sh spawn && bash sim/p3.sh start
#   bash sim/p3.sh battery 0.15        ← 3 秒后应自动进入 DOCKING
#   bash sim/p3.sh watch               ← 看 DOCKING -> DOCKED -> UNDOCKING -> IDLE
set -u

export MSYS_NO_PATHCONV=1

CONTAINER="${CONTAINER:-sim}"
WORLD="${WORLD:-turtlebot3_world}"

ENV_FILE="$(cd "$(dirname "$0")" && pwd)/.env"
env_get() { grep -E "^$1=" "$ENV_FILE" 2>/dev/null | cut -d= -f2 | tr -d '\r '; }

DOCK_X="${DOCK_X:-$(env_get DOCK_X)}"; DOCK_X="${DOCK_X:--2.3}"
DOCK_Y="${DOCK_Y:-$(env_get DOCK_Y)}"; DOCK_Y="${DOCK_Y:-0.5}"
DOCK_YAW="${DOCK_YAW:-$(env_get DOCK_YAW)}"; DOCK_YAW="${DOCK_YAW:-0.0}"

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

need_container() {
  if ! "$DOCKER" ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
    echo "容器 $CONTAINER 没在运行。先执行：" >&2
    echo "  cd sim && docker compose --env-file .env up -d" >&2
    exit 1
  fi
}

# 这个容器里 `pkill -f` 不生效（试过，进程纹丝不动），必须按 PID 杀。
kill_p3() {
  # 不含 twist_mux：它是常驻基础设施，由 entrypoint 起
  ros "for p in \$(ps -eo pid,args --no-headers | grep -E 'lightdog_(aruco_dock|dock_manager)|lightdog_p3.launch.py' | grep -v grep | awk '{print \$1}'); do kill -9 \$p 2>/dev/null; done; sleep 1; true"
}

cmd="${1:-status}"

case "$cmd" in

  start)
    need_container
    kill_p3
    ros "nohup ros2 launch lightdog lightdog_p3.launch.py > /tmp/p3.log 2>&1 & echo started"
    sleep 10
    echo "P3 节点已拉起，日志 /tmp/p3.log（容器内）。当前节点："
    ros "ros2 node list 2>/dev/null | grep -E 'lightdog_(aruco|dock)' || echo '  (还没出现在 node list，稍等几秒)'"
    echo
    echo "=== /dock_robot action 是否可用 ==="
    ros "timeout 10 ros2 action list 2>/dev/null | grep -E 'dock_robot|undock_robot' || echo '  (没有 docking_server，检查 navigation_launch 日志)'"
    ;;

  spawn)
    need_container
    SDF=/home/ros/linorobot2_ws/install/lightdog/share/lightdog/models/lightdog_dock/model.sdf
    if ! ros "test -f $SDF"; then
      echo "找不到 $SDF，先重新 colcon build（或确认 lightdog/models 已安装）" >&2
      exit 1
    fi
    ros "gz service -s /world/$WORLD/remove --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean \
         --timeout 1000 --req 'name: \"lightdog_dock\", type: MODEL' >/dev/null 2>&1; true"
    ok=0
    for i in 1 2 3 4 5; do
      echo "[$i/5] ros_gz_sim create dock @ ($DOCK_X, $DOCK_Y, yaw $DOCK_YAW) ..."
      out=$(ros "ros2 run ros_gz_sim create -world $WORLD -file $SDF -name lightdog_dock \
                 -x $DOCK_X -y $DOCK_Y -z 0.0 -Y $DOCK_YAW" 2>&1)
      echo "$out" | sed 's/^/    /'
      if echo "$out" | grep -qi "Entity creation successful\|created entity\|Entity created"; then ok=1; break; fi
      sleep 4
    done
    if [ "$ok" != "1" ]; then
      echo "生成失败。多半是 gz server 还没就绪，等 20 秒再试一次。" >&2
      exit 1
    fi
    echo "底座已生成。注意：桩体低于激光平面，/scan 里看不到它是正常的。"
    ;;

  stop)
    need_container
    ros "ros2 topic pub --once /dock/command std_msgs/msg/String '{data: cancel}' >/dev/null 2>&1; true"
    ros "ros2 topic pub --once /cmd_vel_dock geometry_msgs/msg/Twist '{}' >/dev/null 2>&1; true"
    kill_p3
    echo "P3 节点已停"
    ;;

  dock)
    need_container
    ros "ros2 topic pub --once /dock/command std_msgs/msg/String '{data: dock}' >/dev/null 2>&1"
    echo "已发 dock 命令。用 bash sim/p3.sh watch 看进度"
    ;;

  undock)
    need_container
    ros "ros2 topic pub --once /dock/command std_msgs/msg/String '{data: undock}' >/dev/null 2>&1"
    echo "已发 undock 命令"
    ;;

  cancel)
    need_container
    ros "ros2 topic pub --once /dock/command std_msgs/msg/String '{data: cancel}' >/dev/null 2>&1"
    echo "已发 cancel 命令"
    ;;

  battery)
    need_container
    v="${2:-}"
    if [ -z "$v" ]; then echo "用法: bash sim/p3.sh battery 0.15 | off" >&2; exit 1; fi
    if [ "$v" = "off" ]; then
      # 发一个过期值：dock_manager 的 sim_battery_timeout（5s）过后会自动回落到真实电池
      echo "注入值会在 5 秒后自动失效（sim_battery_timeout），无需额外操作"
      exit 0
    fi
    # 每 2 秒重发一次，让注入值一直新鲜（超时窗口 5s）
    ros "for i in \$(seq 1 30); do ros2 topic pub --once /dock/sim_battery_ratio std_msgs/msg/Float32 '{data: $v}' >/dev/null 2>&1; sleep 2; done & echo '注入 $v（后台持续 60s）'"
    ;;

  pose)
    need_container
    echo "=== 机器人 / 底座 在 gz 世界里的位姿 ==="
    ros "source /opt/ros/jazzy/setup.bash >/dev/null 2>&1
         timeout 12 gz topic -e -t /world/$WORLD/pose/info -n 1 2>/dev/null > /tmp/pose_p3.txt
         python3 - <<'PY'
import re, math
s = open('/tmp/pose_p3.txt', errors='replace').read()
for blk in s.split('name: \"')[1:]:
    name = blk.split('\"')[0]
    if not any(k in name for k in ('lightdog_dock', 'linorobot2', 'base_footprint')):
        continue
    m = re.search(r'position \\{(.*?)\\}', blk, re.S)
    o = re.search(r'orientation \\{(.*?)\\}', blk, re.S)
    vals = {}
    for k in 'xyz':
        mm = re.search(k + r': ([0-9.eE+-]+)', m.group(1)) if m else None
        vals[k] = round(float(mm.group(1)), 3) if mm else None
    yaw = None
    if o:
        d = {}
        for k in ('x','y','z','w'):
            mm = re.search(k + r': ([0-9.eE+-]+)', o.group(1))
            d[k] = float(mm.group(1)) if mm else 0.0
        yaw = round(math.degrees(2*math.atan2(d['z'], d['w'])), 1)
    print(f'  {name[:44]:46s} {vals}  yaw={yaw}')
PY"
    ;;

  watch)
    need_container
    echo "实时监控（Ctrl+C 退出）：state | battery_ratio | marker_visible | mux | cmd_vel"
    ros "bash -c '
      (timeout 70 ros2 topic echo /dock/state std_msgs/msg/String 2>/dev/null | grep data | sed \"s/^/  state  /\" ) &
      (timeout 70 ros2 topic echo /dock/battery_ratio std_msgs/msg/Float32 2>/dev/null | grep data | sed \"s/^/  ratio  /\" ) &
      (timeout 70 ros2 topic echo /cmd_vel_mux_state std_msgs/msg/String 2>/dev/null | grep data | sed \"s/^/  mux    /\" ) &
      (timeout 70 ros2 topic echo /cmd_vel geometry_msgs/msg/Twist 2>/dev/null | grep -E \"^  x: |^  z: \" | paste - - -d\" |\" | sed \"s/^/  vel    /\" ) &
      wait'"
    ;;

  log)
    need_container
    ros "tail -60 /tmp/p3.log 2>&1"
    ;;

  status)
    need_container
    echo "=== 容器 ==="
    "$DOCKER" ps --format '{{.Names}} | {{.Status}}' | grep "^$CONTAINER"
    echo
    echo "=== P3 节点 ==="
    ros "ros2 node list 2>/dev/null | grep -E 'lightdog_(aruco_dock|dock_manager)' || echo '  (未启动)'"
    echo
    echo "=== 状态机 ==="
    ros "timeout 8 ros2 topic echo /dock/state std_msgs/msg/String --once 2>/dev/null | grep data || echo '  (读不到)'"
    echo "=== 电量比 ==="
    ros "timeout 8 ros2 topic echo /dock/battery_ratio std_msgs/msg/Float32 --once 2>/dev/null | grep data || echo '  (读不到)'"
    echo "=== marker 可见性 ==="
    ros "timeout 8 ros2 topic echo /dock/marker_visible std_msgs/msg/Bool --once 2>/dev/null | grep data || echo '  (读不到)'"
    echo "=== 检测到的 dock 停靠位姿（base_footprint 系）==="
    # 注意缩进：ros2 topic echo 的嵌套字段是 4 空格起（pose.position.x）
    ros "timeout 8 ros2 topic echo /detected_dock_pose geometry_msgs/msg/PoseStamped --once 2>/dev/null | grep -E 'frame_id|^    x:|^    y:|^    z:' || echo '  (读不到，marker 可能不在视野内)'"
    echo "=== action server ==="
    ros "timeout 8 ros2 action list 2>/dev/null | grep -E 'dock_robot|undock_robot' || echo '  (没有)'"
    echo "=== /cmd_vel 仲裁 ==="
    ros "timeout 10 ros2 topic echo /cmd_vel_mux_state std_msgs/msg/String --once 2>/dev/null | grep data || echo '  (读不到)'"
    echo
    echo "=== /battery_state（真实）==="
    ros "timeout 8 ros2 topic echo /battery_state sensor_msgs/msg/BatteryState --once 2>/dev/null | grep -E 'voltage|current|charge' || echo '  (读不到，gz 电池桥接要几分钟)'"
    ;;

  *)
    sed -n '2,22p' "$0" | sed 's/^# \{0,1\}//'
    exit 1
    ;;
esac
