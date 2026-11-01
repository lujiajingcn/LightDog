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

# 1) 上游唯一必需的补丁（幂等；失败则直接退出，不带着坏配置继续）
echo "=== Patching upstream ==="
python3 /patch_upstream.py

# 2) 构建 lightdog 包（源码从宿主机挂载，--symlink-install 让 urdf/launch 改动即时生效）
echo "=== Building lightdog package ==="
cd /home/ros/linorobot2_ws
colcon build --symlink-install --packages-select lightdog
source install/setup.bash

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

# 4) /cmd_vel 多源仲裁（常驻）。
#    collision_monitor 的输入已由 patch_upstream.py 指到本节点的输出，
#    所以它必须在仿真栈之前就位，否则 Nav2 一条速度都发不出去（统一被
#    source_timeout 兜成零）。P2 的跟随节点只是往它里面多接一个源。
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
    spawn_yaw:=0.0 &
SIM_PID=$!

sleep 10

# 6) SLAM + RViz
echo "=== Starting SLAM + RViz ==="
ros2 launch linorobot2_navigation slam.launch.py sim:=true rviz:=true &
SLAM_PID=$!

sleep 8

# 7) Gazebo GUI（默认开）。两件事决定了这段必须放在 SLAM 之后：
#    - 容器里 gz 只跑服务端，不拉 GUI 的话浏览器里只有 RViz，看不到车身和 LED
#    - Xvfb 下没有窗口管理器，窗口靠创建顺序叠放 → 后建的在上层，
#      所以 GUI 必须在 RViz 之后再起，打开 noVNC 才是 3D 视图在最前面
BASE="${LINOROBOT2_BASE:-4wd}"
GUI_PID=""
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
echo "  Sim:    PID $SIM_PID"
echo "  GUI:    PID ${GUI_PID:-未启动}"
echo "  SLAM:   PID $SLAM_PID"
echo "  Mux:    PID $MUX_PID (log /tmp/twist_mux.log)"
echo "  LED log: /tmp/lightdog_led.log (via ros2 launch)"
echo ""

wait -n $SIM_PID $SLAM_PID
echo "A process exited, shutting down..."
kill $XVFB_PID $VNC_PID $NOVNC_PID $SIM_PID $SLAM_PID $MUX_PID ${GUI_PID:-} 2>/dev/null || true
exit 0
