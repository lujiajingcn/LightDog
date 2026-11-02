#!/usr/bin/env bash
# LightDog P2 助手：人体检测 + 跟随 + 光照联动（宿主机 Git Bash 操作容器 sim）
#
#   bash sim/p2.sh start          # 启动 P2 四个节点（detector/light/follower/person_sim）
#   bash sim/p2.sh reset          # 机器人复位到 .env 里的出生点（先停 P2 节点）
#   bash sim/p2.sh spawn          # 在场景里生成「人」（带重试）
#   bash sim/p2.sh stop           # 停掉 P2 节点，机器人停住
#   bash sim/p2.sh status         # 节点/话题/人位姿/亮度/速度 概览
#   bash sim/p2.sh watch          # 实时盯 present / position / cmd_vel（验收用）
#   bash sim/p2.sh det            # 读一次 /person/position
#   bash sim/p2.sh pose           # 读「人」和机器人在世界里的实际位姿
#   bash sim/p2.sh brightness 0.8 # 手动接管亮度（先 stop，否则会被 auto 覆盖）
#   bash sim/p2.sh log            # 看 P2 节点日志尾巴
#
# 典型验收流程：
#   bash sim/p2.sh start && bash sim/p2.sh spawn
#   浏览器 http://127.0.0.1:6080/vnc.html  ← 看到人和机器人
#   bash sim/p2.sh watch                ← 终端里看数据是否跟着人动
set -u

export MSYS_NO_PATHCONV=1

CONTAINER="${CONTAINER:-sim}"
WORLD="${WORLD:-turtlebot3_world}"

ENV_FILE="$(cd "$(dirname "$0")" && pwd)/.env"
env_get() { grep -E "^$1=" "$ENV_FILE" 2>/dev/null | cut -d= -f2 | tr -d '\r '; }

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

# 连发 6 条而不是 `--once`：单发时发布进程常在 DDS 发现完成前退出，消息会丢，
# 而输出被重定向掉了 → 现象是「命令成功返回、车没动」。见 sim/system.sh 的说明。
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

# 注意：这个容器里 `pkill -f` 不生效（试过，进程纹丝不动），必须按 PID 杀。
# 用 ps 拿 PID 再 kill -9。
kill_p2() {
  # 注意不含 twist_mux：它是常驻基础设施（entrypoint 起的），停 P2 不能把它带走，
  # 否则 Nav2 的输入也断了。
  #
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
  ros "f() { ps -eo pid,args --no-headers | grep -E 'lightdog_(person_detector|follower|light_auto|person_sim)\\.py|lightdog_p2\\.launch\\.py' | grep -v grep | awk -v me=\$\$ '\$1 != me {print \$1}'; }; \
       pids=\$(f); if [ -n \"\$pids\" ]; then kill -TERM \$pids 2>/dev/null; sleep 2; fi; \
       pids=\$(f); if [ -n \"\$pids\" ]; then kill -9 \$pids 2>/dev/null; fi; sleep 1; true"
}

# 关于 /cmd_vel 的仲裁（历史教训，别回退）
#
# P2 最初是「kill 掉 Nav2 运动节点让 follower 独占 /cmd_vel」，因为导航空闲时
# command_timeout / collision_monitor / docking_server 会持续往 /cmd_vel 灌零速，
# 把 follower 的 -0.15 淹掉（实测机器人 6 秒只挪 5cm）。代价是丢了碰撞保护。
#
# 现在改成标准做法：follower 只发 /cmd_vel_follow，由 lightdog_twist_mux 仲裁
# （跟随 prio 20 > Nav2 prio 10），输出 /cmd_vel_muxed 再进 collision_monitor，
# 最终仍由 collision_monitor 独占 /cmd_vel。所以这里**不再需要杀 Nav2**。
show_mux() {
  # 发现 DDS 有延迟，刚起完节点时 5s 内很可能读不到，给足 12s。
  ros "source /opt/ros/jazzy/setup.bash >/dev/null 2>&1
       echo -n '  仲裁当前生效源: '
       timeout 12 ros2 topic echo /cmd_vel_mux_state std_msgs/msg/String --once 2>/dev/null | grep data || echo '(读不到：跟随节点未起时是 idle，多等几秒再 status)'
       echo '  /cmd_vel 发布者:'
       timeout 10 ros2 topic info -v /cmd_vel 2>/dev/null | grep -A2 'Node name' | grep -E 'Node name' | sed 's/^/    /'"
}

cmd="${1:-status}"

case "$cmd" in

  start)
    need_container
    kill_p2
    ros "nohup ros2 launch lightdog lightdog_p2.launch.py > /tmp/p2.log 2>&1 & echo started"
    # 10s 而不是 6s：DDS 发现慢，太早读 node list 会是空的，看着像启动失败
    sleep 10
    echo "P2 节点已拉起，日志 /tmp/p2.log（容器内）。当前节点："
    ros "ros2 node list 2>/dev/null | grep -E 'lightdog_(person|follower|light|twist)' || echo '  (还没出现在 node list，稍等几秒)'"
    echo
    show_mux
    ;;

  reset)
    need_container
    # 只用 set_pose 搬机器人是不可靠的：给 z=0 会把轮子直接压进地面，
    # 之后 gz 侧明明收到 x:0.2（验证过），机器人却一动不动；给 z=0.15 让它
    # 从空中落下也一样卡住（base_footprint 停在 -0.0138 缓慢上浮）。
    # 干净的办法是把容器重启一次，机器人回到 spawn 的初始状态。
    echo "重启仿真容器，把机器人放回出生点 ..."
    ( cd "$(dirname "$0")" && "$DOCKER" compose --env-file .env restart 2>&1 | tail -2 )
    echo "等约 40s 仿真栈就绪（docker logs -f sim 看到 'All services started'）"
    echo "然后：bash sim/p2.sh start && bash sim/p2.sh spawn"
    ;;

  spawn)
    need_container
    # 人的出生点要和 p2_params.yaml 里 person_sim 的 waypoints 起点一致，
    # 且落在 y=0.5 那条无柱走廊上（见 sim/.env 的说明）。
    SPAWN_HX="${SPAWN_HX:--0.4}"
    SPAWN_HY="${SPAWN_HY:-0.5}"
    SDF=/home/ros/linorobot2_ws/install/lightdog/share/lightdog/models/person/model.sdf
    if ! ros "test -f $SDF"; then
      echo "找不到 $SDF，先重新 colcon build（或确认 lightdog/models 已安装）" >&2
      exit 1
    fi
    # 已有同名实体先删掉，避免重复
    ros "gz service -s /world/$WORLD/remove --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean \
         --timeout 1000 --req 'name: \"person\", type: MODEL' >/dev/null 2>&1; true"
    ok=0
    for i in 1 2 3 4 5; do
      echo "[$i/5] ros_gz_sim create ..."
      out=$(ros "ros2 run ros_gz_sim create -world $WORLD -file $SDF -name person -x $SPAWN_HX -y $SPAWN_HY -z 0.0" 2>&1)
      echo "$out" | sed 's/^/    /'
      # gz Harmonic 下实际输出是 "Entity creation successful."，早期版本是
      # "Entity created"，两个都认，否则会把成功当失败连开 5 次。
      if echo "$out" | grep -qi "Entity creation successful\|created entity\|Entity created"; then ok=1; break; fi
      sleep 4
    done
    if [ "$ok" != "1" ]; then
      echo "生成失败。多半是 gz server 还没就绪，等 20 秒再试一次。" >&2
      exit 1
    fi
    echo "人已生成（初始位置 $SPAWN_HX, $SPAWN_HY）"
    ;;

  stop)
    need_container
    # 顺序很重要：**先杀节点，再发零速**。
    # 反过来的话，跟随节点还活着，它下一拍就用 P 控制重算的速度覆盖掉这条零速
    # （实测「stop 之后车还在往前挪半秒」）。
    kill_p2
    pub /cmd_vel_follow geometry_msgs/msg/Twist '{}'
    pub /cmd_vel geometry_msgs/msg/Twist '{}'
    echo "P2 节点已停，机器人已停。亮度保持最后状态（bash sim/p1.sh led off 可复位）"
    ;;

  log)
    need_container
    ros "tail -40 /tmp/p2.log 2>&1"
    ;;

  det)
    need_container
    echo "读 /person/position（最多等 15 秒）..."
    ros "timeout 15 ros2 topic echo /person/position --once 2>&1 | grep -E 'frame_id|^  x:|^  y:|^  z:'"
    ;;

  pose)
    need_container
    echo "=== world 里各实体的位置 ==="
    ros "source /opt/ros/jazzy/setup.bash >/dev/null 2>&1
         timeout 12 gz topic -e -t /world/$WORLD/pose/info -n 1 2>/dev/null > /tmp/pose_p2.txt
         python3 - <<'PY'
import re
s = open('/tmp/pose_p2.txt', errors='replace').read()
for blk in s.split('name: \"')[1:]:
    name = blk.split('\"')[0]
    if not any(k in name for k in ('person', 'linorobot2', 'base_footprint')):
        continue
    m = re.search(r'position \\{(.*?)\\}', blk, re.S)
    p = {}
    if m:
        for k in 'xyz':
            mm = re.search(k + r': ([0-9.eE+-]+)', m.group(1))
            p[k] = round(float(mm.group(1)), 3) if mm else None
    print(f'  {name[:52]:54s} {p}')
PY"
    ;;

  watch)
    need_container
    echo "实时监控（Ctrl+C 退出）：present | position(fwd/left) | cmd_vel(v/w)"
    ros "timeout 90 ros2 topic echo /person/present std_msgs/msg/Bool 2>/dev/null | head -3" >/dev/null 2>&1
    ros "bash -c '
      (timeout 60 ros2 topic echo /person/position geometry_msgs/msg/PointStamped 2>/dev/null | grep -E \"^  x:|^  y:|^  z:\" | paste - - - -d\" \" ) &
      (timeout 60 ros2 topic echo /cmd_vel geometry_msgs/msg/Twist 2>/dev/null | grep -E \"^  x: |^  z: \" | paste - - -d\" \" ) &
      wait'"
    ;;

  brightness)
    need_container
    v="${2:-}"
    if [ -z "$v" ]; then echo "用法: bash sim/p2.sh brightness 0.8 | off" >&2; exit 1; fi
    case "$v" in off) v=0.0 ;; on|full|max) v=1.0 ;; esac
    pub /led/brightness std_msgs/msg/Float32 "{data: $v}"
    echo "主灯亮度 -> $v（注意：light_auto 还在跑的话会被它覆盖，先 bash sim/p2.sh stop）"
    ;;

  status)
    need_container
    echo "=== 容器 ==="
    "$DOCKER" ps --format '{{.Names}} | {{.Status}}' | grep "^$CONTAINER"
    echo
    echo "=== P2 节点 ==="
    ros "ros2 node list 2>/dev/null | grep -E 'lightdog_(person_detector|person_sim|follower|light_auto|twist_mux)' || echo '  (未启动)'"
    echo
    echo "=== /cmd_vel 仲裁 ==="
    show_mux
    echo
    echo "=== P2 话题 ==="
    ros "ros2 topic list 2>/dev/null | grep -E 'person|led/brightness|cmd_vel'"
    echo
    echo "=== 相机是否出图 ==="
    ros "timeout 8 ros2 topic hz /camera/color/image_raw 2>&1 | head -2"
    echo
    echo "=== 当前亮度 ==="
    ros "timeout 6 ros2 topic echo /led/brightness --once 2>&1 | head -2"
    echo
    echo "=== 人的位姿 ==="
    ros "source /opt/ros/jazzy/setup.bash >/dev/null 2>&1
         timeout 10 gz service -s /world/$WORLD/pose/info --reqtype gz.msgs.Empty --reptype gz.msgs.Pose_V --timeout 2000 --req '' 2>/dev/null | grep -A6 'name: \"person\"' | head -8 || echo '  (人还没生成)'"
    ;;

  *)
    sed -n '2,20p' "$0" | sed 's/^# \{0,1\}//'
    exit 1
    ;;
esac
