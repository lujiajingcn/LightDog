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

# 用 sim/pub_once.py 发一条：`--once` 的发布进程常在 DDS 发现完成前退出，
# 消息会丢，而输出被重定向掉了 → 现象是「命令成功返回、车没动」。
# 这三个命令本身都有 busy 保护（dock_manager 的 start_docking / start_undocking
# 都是 `if self.busy: return`），就算真的重复送达也没有副作用。
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

# 这个容器里 `pkill -f` 不生效（试过，进程纹丝不动），必须按 PID 杀。
kill_p3() {
  # 不含 twist_mux：它是常驻基础设施，由 entrypoint 起
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
  ros "f() { ps -eo pid,args --no-headers | grep -E 'lightdog_(aruco_dock|dock_manager)\\.py|lightdog_p3\\.launch\\.py' | grep -v grep | awk -v me=\$\$ '\$1 != me {print \$1}'; }; \
       pids=\$(f); if [ -n \"\$pids\" ]; then kill -TERM \$pids 2>/dev/null; sleep 2; fi; \
       pids=\$(f); if [ -n \"\$pids\" ]; then kill -9 \$pids 2>/dev/null; fi; sleep 1; true"
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
    # 先取消（让 docking_server 松开方向盘），再杀节点，最后补一条零速
    #
    # cancel 发不出去**不该拦住"停"这个动作本身**：节点照杀，只提醒一句。
    pub /dock/command std_msgs/msg/String '{data: cancel}' 2>/dev/null \
      || echo "  (cancel 没送达 —— P3 节点本来就没起？仍继续执行停止)"
    kill_p3
    # 这一条只是把 mux 里残留的最后指令刷成零速。节点刚被杀掉，
    # /cmd_vel_dock 上 0 个订阅者是**预期状态**，不算失败，所以连告警都吞掉。
    pub /cmd_vel_dock geometry_msgs/msg/Twist '{}' 2>/dev/null || true
    echo "P3 节点已停"
    ;;

  # 下面三条是**命令**，必须确认送达才报成功。
  # pub 在没人订阅时返回 1（消息直接进了黑洞），原来忽略返回值 →
  # 屏幕上先打「!! 没有订阅者」再打「已发 dock 命令」，自相矛盾，
  # 而操作者只会看最后一行。
  dock)
    need_container
    pub /dock/command std_msgs/msg/String '{data: dock}' || exit 1
    echo "已发 dock 命令。用 bash sim/p3.sh watch 看进度"
    ;;

  undock)
    need_container
    pub /dock/command std_msgs/msg/String '{data: undock}' || exit 1
    echo "已发 undock 命令"
    ;;

  cancel)
    need_container
    pub /dock/command std_msgs/msg/String '{data: cancel}' || exit 1
    echo "已发 cancel 命令"
    ;;

  battery)
    need_container
    v="${2:-}"
    if [ -z "$v" ]; then echo "用法: bash sim/p3.sh battery 0.15 | off" >&2; exit 1; fi
    if [ "$v" = "off" ]; then
      # 原来只打印一句话就退出：后台的 `ros2 topic pub` 还在持续发新鲜值，
      # 要等最多 65s 才停，dock_manager 根本不会回落到真实电池。
      # 模式里写 [r]os2 是为了不让 pkill 匹配到承载这条命令的 shell 自己。
      ros "pkill -f '[r]os2 topic pub -r 1 -t 60 /dock/sim_battery_ratio' >/dev/null 2>&1 || true"
      echo "已停止电量注入，sim_battery_timeout（5s）后回落到真实电池"
      exit 0
    fi
    # -t 60 数的是**消息条数**，不是秒数。这台机器 `-r 1` 实际只能跑到 2~3Hz，
    # 所以 60 条往往 20~30 秒就发完了；好在窗口只有 sim_battery_timeout(5s)，
    # 只要注入期间持续有新鲜值就够，停发 5 秒后才回落到真实电池。
    # nohup 不能省：docker exec 的 shell 一退出，后台子进程会被回收，
    # 表现为「注入成功、2 秒后电量直接跳回真实值」。
    ros "nohup timeout 65 ros2 topic pub -r 1 -t 60 /dock/sim_battery_ratio std_msgs/msg/Float32 '{data: $v}' >/dev/null 2>&1 & echo '注入 $v（后台，60 条消息 ≈ 20~30s，停发 5s 后失效）'"
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
