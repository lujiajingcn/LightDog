#!/bin/bash
# LightDog 仿真容器入口。
# 容器基于 linorobot2:gazebo 镜像（上游 ROS 2 Jazzy + Gazebo Harmonic 已装好），
# 只额外挂载我们的 lightdog 包，进来后现场构建，再拉起仿真。
set -e

source "/opt/ros/${ROS_DISTRO}/setup.bash"
source /home/ros/linorobot2_ws/install/setup.bash

WORLD="${WORLD:-turtlebot3_world}"
SPAWN_X="${SPAWN_X:-0.5}"

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

# 4) LightDog 仿真栈（上游 gazebo.launch.py + 我们的 URDF 覆盖层 + 桥接 + LED 节点）
echo "=== Starting simulation ==="
ros2 launch lightdog lightdog_sim.launch.py \
    gui:=false \
    world_name:="${WORLD}" \
    spawn_x:="${SPAWN_X}" \
    spawn_y:=0.0 \
    spawn_yaw:=0.0 &
SIM_PID=$!

sleep 10

# 5) SLAM + RViz
echo "=== Starting SLAM + RViz ==="
ros2 launch linorobot2_navigation slam.launch.py sim:=true rviz:=true &
SLAM_PID=$!

echo "=== All services started ==="
echo "  VNC:    localhost:5900"
echo "  noVNC:  http://localhost:6080/vnc.html"
echo "  Sim:    PID $SIM_PID"
echo "  SLAM:   PID $SLAM_PID"
echo "  LED log: /tmp/lightdog_led.log (via ros2 launch)"
echo ""

wait -n $SIM_PID $SLAM_PID
echo "A process exited, shutting down..."
kill $XVFB_PID $VNC_PID $NOVNC_PID $SIM_PID $SLAM_PID 2>/dev/null || true
exit 0
