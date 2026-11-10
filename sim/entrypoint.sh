#!/bin/bash
# LightDog 仿真容器入口。
# 容器基于 linorobot2:gazebo 镜像（上游 ROS 2 Jazzy + Gazebo Harmonic 已装好），
# 只额外挂载我们的 lightdog 包，进来后现场构建，再拉起仿真。
set -e

source "/opt/ros/${ROS_DISTRO}/setup.bash"
source /home/ros/linorobot2_ws/install/setup.bash

WORLD="${WORLD:-turtlebot3_world}"
SPAWN_X="${SPAWN_X:-0.5}"
SPAWN_Y="${SPAWN_Y:-0.0}"

# 各阶段 PID 先全部初始化：cleanup 可能在任何一个节点起来之前就被触发
# （`docker stop` 立刻发 SIGTERM），而脚本开了 `set -u`，未定义变量会让
# 清理函数自己先崩掉，反而留下一堆孤儿进程。
XVFB_PID="" VNC_PID="" NOVNC_PID="" MUX_PID="" SIM_PID="" SLAM_PID=""
GUI_PID="" P4_PID="" P5_PID="" SYS_PID=""

# 递归杀整棵进程树。
#
# 为什么不能只 `kill $PID`：`ros2 launch` 只是调度器，真正的节点是它的
# 子进程。杀掉 launch 之后子进程会被 init 收养继续活着 —— 实测表现为
# 「重启后有两个 lightdog_mission 同时抢 /follow/enable」，排查了很久。
kill_tree() {
  local pid="$1" sig="${2:-TERM}" kids k
  if [ -z "$pid" ]; then
    return 0
  fi
  kids=$(ps -o pid= --ppid "$pid" 2>/dev/null || true)
  for k in $kids; do
    kill_tree "$k" "$sig"
  done
  kill -"$sig" "$pid" 2>/dev/null || true
  return 0
}

CLEANED=0
cleanup() {
  # 注意：这里一律用 if 而不是 `[ ... ] && foo`。
  # 脚本开了 `set -e`，而 `[ 条件 ] && cmd` 在条件为假时整条命令返回 1，
  # 会被当成"清理函数失败"直接掀掉整个脚本 —— 后面的清理一步都不跑。
  if [ "$CLEANED" = "1" ]; then
    return 0
  fi
  CLEANED=1
  echo "=== Shutting down ==="
  local p
  for p in "$GUI_PID" "$P4_PID" "$P5_PID" "$SYS_PID" "$SIM_PID" "$SLAM_PID" "$MUX_PID"; do
    if [ -n "$p" ]; then
      kill_tree "$p" TERM
    fi
  done
  sleep 1
  for p in "$GUI_PID" "$P4_PID" "$P5_PID" "$SYS_PID" "$SIM_PID" "$SLAM_PID" "$MUX_PID"; do
    if [ -n "$p" ]; then
      kill_tree "$p" KILL
    fi
  done
  for p in "$XVFB_PID" "$VNC_PID" "$NOVNC_PID"; do
    if [ -n "$p" ]; then
      kill -TERM "$p" 2>/dev/null || true
    fi
  done
  return 0
}
# TERM/INT 覆盖 `docker stop` / Ctrl+C；EXIT 兜住「某个 launch 自己挂了」
# 之后的正常退出路径。CLEANED 保证只跑一次。
# I-004：STOPPING 只在收到终止信号时置位（EXIT 那次 cleanup 不置），用来区分
# 「人为 stop（正常退出 0）」和「被监控进程自己挂了（异常，退非零让编排重启）」。
STOPPING=0
trap 'STOPPING=1; cleanup' TERM INT
trap cleanup EXIT

# 1) 上游唯一必需的补丁（幂等；失败则直接退出，不带着坏配置继续）
echo "=== Patching upstream ==="
python3 /patch_upstream.py

# 2) 构建 lightdog 包（源码从宿主机挂载，--symlink-install 让 urdf/launch 改动即时生效）
echo "=== Building lightdog package ==="
cd /home/ros/linorobot2_ws
colcon build --symlink-install --packages-select lightdog
source install/setup.bash

# 让 gz 能按 model:// 解析我们自己的模型（充电桩的 ArUco 贴图靠这个找）
# 不加这行，model.sdf 里的 model://lightdog_dock/... 会在 spawn 时报
# "Unable to find material"，marker 就是个纯白/纯黑方块，回充直接瞎。
export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH}:/home/ros/linorobot2_ws/install/lightdog/share/lightdog/models"
echo "=== GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH ==="

# 3) 虚拟显示 + VNC
export DISPLAY=:200
DISP_NUM=${DISPLAY#:}
rm -f "/tmp/.X${DISP_NUM}-lock" "/tmp/.X11-unix/X${DISP_NUM}" 2>/dev/null || true
mkdir -p /tmp/.X11-unix
Xvfb :200 -screen 0 1920x1080x24 -ac +extension GLX +render -noreset &
XVFB_PID=$!
sleep 2
x11vnc -display :200 -forever -shared -rfbport 5900 -nopw > /tmp/x11vnc.log 2>&1 &
VNC_PID=$!
websockify --web=/usr/share/novnc/ 6080 localhost:5900 > /tmp/websockify.log 2>&1 &
NOVNC_PID=$!
echo "=== Xvfb :200 (PID $XVFB_PID) / x11vnc :5900 / noVNC :6080 ==="

# 4) /cmd_vel 多源仲裁（常驻，且必须在仿真栈之前就位）
#
#    collision_monitor 的输入已由 patch_upstream.py 指到本节点的输出，
#    所以它必须在仿真栈之前就位，否则 Nav2 一条速度都发不出去（统一被
#    source_timeout 兜成零）。P2 的跟随 / P3 的回充只是往它里面多接一个源。
echo "=== Starting twist mux ==="
nohup ros2 run lightdog lightdog_twist_mux.py --ros-args \
    --params-file /home/ros/linorobot2_ws/install/lightdog/share/lightdog/config/p2_params.yaml \
    > /tmp/twist_mux.log 2>&1 &
MUX_PID=$!
echo "  twist_mux PID $MUX_PID (log /tmp/twist_mux.log)"

# 5) LightDog 仿真栈（上游 gazebo.launch.py + 我们的 URDF 覆盖层 + 桥接 + LED 节点）
echo "=== Starting simulation ==="
ros2 launch lightdog lightdog_sim.launch.py \
    gui:=false \
    world_name:="${WORLD}" \
    spawn_x:="${SPAWN_X}" \
    spawn_y:="${SPAWN_Y}" \
    spawn_yaw:="${SPAWN_YAW:-0.0}" &
SIM_PID=$!

sleep 10

# 6) SLAM + RViz
echo "=== Starting SLAM + RViz ==="
ros2 launch linorobot2_navigation slam.launch.py sim:=true rviz:=true &
SLAM_PID=$!

sleep 8

# 6.5) P4：录像 + 手机互联后端
#
#      放在 SLAM 之后、Gazebo GUI 之前，是因为这两个节点不依赖任何东西 ——
#      相机没出图时 recorder 只是空转，server 的 /status 照样回 200。早点起，
#      GUI 那 30 秒的等待时间里录像已经开始了。
# 存储卡根目录。P4 的录像和 P5 的 contacts.json 都落在这里，所以在两个
# ENABLE_* 判断之前就定下来（否则 ENABLE_P4=0 时它会是个空串，
# contacts_path 就变成 `:=/contacts.json`，参数看着合法但指向根目录）。
DATA_DIR="${LIGHTDOG_DATA_DIR:-/data}"
mkdir -p "$DATA_DIR"
if [ "${ENABLE_P4:-1}" = "1" ]; then
  # 注意：空的 `auth_token:=` 会被 ros2 launch 判成 malformed argument 直接退出，
  # 所以这里的 P4_ARGS 必须按需拼接，不能写成固定字符串。
  P4_ARGS="data_dir:=$DATA_DIR"
  if [ -n "${LIGHTDOG_TOKEN:-}" ]; then
    P4_ARGS="$P4_ARGS auth_token:=$LIGHTDOG_TOKEN"
  fi
  echo "=== Starting P4 recorder + server (data=$DATA_DIR) ==="
  nohup ros2 launch lightdog lightdog_p4.launch.py $P4_ARGS \
      > /tmp/p4.log 2>&1 &
  P4_PID=$!
  echo "  P4 PID $P4_PID (log /tmp/p4.log)"
fi

# 6.6) P5：跌倒检测 + 告警状态机
#
#      必须在 P4 之后起：告警时要往 /recorder/event 发切片请求，recorder 不在
#      的话告警照样能发（只是没有视频）。反过来 P4 完全不依赖 P5，两个阶段
#      谁先谁后都不会崩，这也是当初把它们拆成两个 launch 的原因。
#
#      默认开：没有 "人" 的时候检测器只是持续报 absent，代价可以忽略，
#      但手机端 /status 里的 alert 字段一直是 NORMAL —— 看起来才像个成品。
if [ "${ENABLE_P5:-1}" = "1" ]; then
  echo "=== Starting P5 fall detector + alert manager ==="
  nohup ros2 launch lightdog lightdog_p5.launch.py \
      contacts_path:="$DATA_DIR/contacts.json" \
      > /tmp/p5.log 2>&1 &
  P5_PID=$!
  echo "  P5 PID $P5_PID (log /tmp/p5.log)"
fi

# 6.7) 系统层：任务调度 + 语音链路
#
#      mission 决定「现在该谁说话」：有人 -> 跟随；低电 -> 先导航到桩前再回充；
#      告警 -> 按住轮子。它不产生速度，但没它的话跟随会一直压着 Nav2
#      （P2 §7.8），而且「从远处自己走回桩」根本没人负责（P3 边界之外）。
#
#      放在最后起：它一起来就会发 /follow/enable，而 P2 的跟随节点如果还没起，
#      这条消息只是没人收 —— 反正跟随节点没收到过时按自己的 enabled 工作。
if [ "${ENABLE_SYSTEM:-1}" = "1" ]; then
  echo "=== Starting system layer (mission + voice) ==="
  nohup ros2 launch lightdog lightdog_system.launch.py \
      > /tmp/system.log 2>&1 &
  SYS_PID=$!
  echo "  system PID $SYS_PID (log /tmp/system.log)"
fi


# 7) Gazebo GUI（默认开）。两件事决定了这段必须放在 SLAM 之后：
#    - 容器里 gz 只跑服务端，不拉 GUI 的话浏览器里只有 RViz，看不到车身和 LED
#    - Xvfb 下没有窗口管理器，窗口靠创建顺序叠放 → 后建的在上层，
#      所以 GUI 必须在 RViz 之后再起，打开 noVNC 才是 3D 视图在最前面
BASE="${LINOROBOT2_BASE:-4wd}"
if [ "${SHOW_GUI:-1}" = "1" ]; then
  echo "=== Starting Gazebo GUI (SHOW_GUI=1) ==="
  gz sim -g --gui-config /gui.config > /tmp/gzgui.log 2>&1 &
  GUI_PID=$!
  # 相机初始位姿由 gui.config 给；机器人出生后改用 follow 跟随，跟着它走
  (
    for _ in $(seq 1 60); do
      gz service -l 2>/dev/null | grep -qx '/gui/follow' && break
      sleep 1
    done
    gz service -s /gui/follow --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean \
      --timeout 5000 --req "data: \"linorobot2_${BASE}\"" >/dev/null 2>&1 || true
    gz service -s /gui/follow/offset --reqtype gz.msgs.Vector3d --reptype gz.msgs.Boolean \
      --timeout 5000 --req 'x: -1.2, y: -0.9, z: 0.8' >/dev/null 2>&1 || true
    echo "[gui] camera locked on linorobot2_${BASE}"
  ) &
fi

echo "=== All services started ==="
echo "  VNC:    localhost:5900"
echo "  noVNC:  http://localhost:6080/vnc.html"
echo "  Phone:  http://<本机局域网IP>:8000/    (P4 手机互联)"
echo "  Sim:    PID $SIM_PID"
echo "  GUI:    PID ${GUI_PID:-未启动}"
echo "  SLAM:   PID $SLAM_PID"
echo "  Mux:    PID $MUX_PID (log /tmp/twist_mux.log)"
echo "  P4:     PID ${P4_PID:-未启动} (log /tmp/p4.log)"
echo "  P5:     PID ${P5_PID:-未启动} (log /tmp/p5.log)"
echo "  System: PID ${SYS_PID:-未启动} (log /tmp/system.log)"

echo "  LED log: /tmp/lightdog_led.log (via ros2 launch)"
echo ""

# I-004：等到「有事发生」。监控集合里必须包含 twist_mux —— 它是 /cmd_mux 的
# 安全仲裁者，掉了就没有人兜零，比 sim/slam 挂掉更该惊动编排。`wait -n` 返回
# **先退出的那个子进程的退出码**；用 `|| rc=$?` 兜住 `set -e`，别在这一行就掀脚本。
rc=0
# I-004 只监控了 mux/sim/slam。P4/P5/system 崩了的话，容器还 healthy、
# wait -n 也等不到任何东西 —— 表现是 :8000 打不开，而编排层毫无察觉。
# 把已启用的 launch 一起纳入监控（未启用的变量为空，不能传进 wait）。
MONITORED=("$MUX_PID" "$SIM_PID" "$SLAM_PID")
for pid in "${P4_PID:-}" "${P5_PID:-}" "${SYS_PID:-}"; do
  if [ -n "$pid" ]; then
    MONITORED+=("$pid")
  fi
done
wait -n "${MONITORED[@]}" || rc=$?

# 收尾交给 trap EXIT 里的 cleanup（递归杀进程树），这里只决定退出码。
if [ "$STOPPING" = "1" ]; then
  echo "Received stop signal — shutting down cleanly."
  exit 0
fi

# 不是人为 stop，就是被监控进程自己挂了（mux / sim / slam 之一）。
# 以前这里 `|| true; exit 0`：崩溃被当成正常退出码 0，`docker restart`/systemd
# 的 on-failure 策略根本不会触发，运维端只看到「容器已退出(0)」= 静默死亡。
# 现在退非零，如实报「出事了」，配合 compose 的 restart 策略才会自动拉回。
echo "A monitored process exited unexpectedly (rc=$rc) — signaling failure."
exit 1
