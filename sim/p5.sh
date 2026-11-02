#!/usr/bin/env bash
# LightDog P5 助手：跌倒检测 + 告警链路（宿主机 Git Bash 操作容器 sim）
#
#   bash sim/p5.sh start             # 拉起跌倒检测 + 告警状态机
#   bash sim/p5.sh stop              # 停掉 P5 两个节点
#   bash sim/p5.sh status            # 节点 / 身位 / 离地高度 / 告警状态 / 告警记录
#   bash sim/p5.sh watch             # 盯 /fall/state、/fall/z_p95、/alert/state
#   bash sim/p5.sh z                 # 只看 /fall/z_p95（判据的物理量）
#   bash sim/p5.sh spawn             # 生成"人"（复用 P2 的模型与生成路径）
#   bash sim/p5.sh stand             # 让人站在机器人正前方（站立基准）
#   bash sim/p5.sh fall              # 让人躺倒（roll 90°）—— 触发跌倒检测
#   bash sim/p5.sh test              # 直接触发一次告警（跳过等待，验证链路）
#   bash sim/p5.sh ack               # 确认已知晓（停止重复呼叫）
#   bash sim/p5.sh reset             # 复位到 NORMAL
#   bash sim/p5.sh debug             # 存一张 /fall/debug_image（带姿态标注）
#   bash sim/p5.sh probe             # 用 curl 把告警 HTTP 接口打一遍（自检）
#   bash sim/p5.sh log               # 看 P5 节点日志
#
# 典型验收流程：
#   bash sim/p5.sh spawn && bash sim/p5.sh stand   # 人站着 -> /fall/state=standing
#   bash sim/p5.sh fall                            # 人躺下
#   bash sim/p5.sh watch                           # 看状态爬台阶：
#     NORMAL -(躺倒)-> SUSPECTED -(5s)-> CONFIRMING -(15s)-> ALARM
#   bash sim/p5.sh status                          # ALARM 里应出现 event_id
#   bash sim/p4.sh files                           # events/ 里多了一条 fall 视频
#   bash sim/p5.sh ack                             # -> ACKED
#   bash sim/p5.sh stand                           # 起身 3 秒后 -> RESOLVED -> NORMAL
set -u

export MSYS_NO_PATHCONV=1

CONTAINER="${CONTAINER:-sim}"
WORLD="${WORLD:-turtlebot3_world}"
PORT="${PORT:-8000}"

# 摆人用的几何参数。
#
# DIST 是「橙色躯干中心」到相机的水平距离。为什么不能随便给：
# 相机光心离地 0.3745m、垂直半视场角约 23.4°，2.0m 处屏幕上能看到的最低高度
# 约 -0.49m —— 人躺下后整体压在 0~0.36m，离太近会直接掉出画面下沿（1.5m 时
# 只余 0.28m 余量，1.2m 就看不见了）。2.0m 既留足余量，橙色躯干也还有
# 约 100x140px，远高于检测器的 min_points=200。
DIST="${P5_DIST:-2.0}"
LIE_Z="${P5_LIE_Z:-0.18}"      # 躺倒时身体中轴离地高度（≈躯干半径，半埋才像躺着）
STAND_Z="${P5_STAND_Z:-0.0}"

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

# 把一段脚本从宿主机 stdin 灌进容器执行（自动先 source ROS 环境）。
#
# 为什么不用 `ros "..."` 拼多行脚本：里面只要出现一个双引号，就会把
# ros() 那层 `bash -lc "..."` 的引号结构打断，报错还发生在容器里，极难定位。
# 走 stdin 之后，脚本内容一个字符都不用转义。
ros_script() {
  {
    echo 'source /opt/ros/jazzy/setup.bash >/dev/null 2>&1'
    echo 'source /home/ros/linorobot2_ws/install/setup.bash >/dev/null 2>&1'
    cat
  } | "$DOCKER" exec -i "$CONTAINER" bash -s
}

# 用 sim/pub_once.py 发一条：`--once` 的发布进程常在 DDS 发现完成前退出，
# 消息会丢，而输出被重定向掉了 → 现象是「命令成功返回、什么都没发生」。
pub() {  # pub <topic> <type> <yaml-body>；返回 1 = 没人订阅，这条命令没送到
  # 只发一条：pub_once.py 会让 publisher 多活 1 秒完成 ACK/重传。
  # 为什么不再连发 6 条，见 sim/system.sh 里 send() 上面那段说明。
  #
  # 必须检查 subscribers：没有订阅者时发布「成功」返回，消息直接进了黑洞，
  # 脚本却照样打印「已下发 ack」—— 实测真出现过（DDS 发现还没收敛就发了）。
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

# 这个容器里 `pkill -f` 不生效，必须按 PID 杀（P3 踩过）
kill_p5() {
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
  ros "f() { ps -eo pid,args --no-headers | grep -E 'lightdog_(fall_detector|alert_manager)\\.py|lightdog_p5\\.launch\\.py' | grep -v grep | awk -v me=\$\$ '\$1 != me {print \$1}'; }; \
       pids=\$(f); if [ -n \"\$pids\" ]; then kill -TERM \$pids 2>/dev/null; sleep 2; fi; \
       pids=\$(f); if [ -n \"\$pids\" ]; then kill -9 \$pids 2>/dev/null; fi; sleep 1; true"
}

# --noproxy '*' 是必须的：本机 shell 环境里带着指向 WorkBuddy 沙箱代理的
# HTTP_PROXY/HTTPS_PROXY，curl 会把它套到 127.0.0.1 上，结果永远是 code=000。
# 输出丢弃用 NUL 而不是 /dev/null（本脚本 export 了 MSYS_NO_PATHCONV=1）。
CURL_OPTS=(-s -m 10 --noproxy '*')

curl_api() {
  local path="$1"; shift || true
  if [ -n "$TOKEN" ]; then
    curl "${CURL_OPTS[@]}" -H "X-Auth-Token: $TOKEN" "$@" "http://127.0.0.1:$PORT$path"
  else
    curl "${CURL_OPTS[@]}" "$@" "http://127.0.0.1:$PORT$path"
  fi
}

# 摆人。
#
# 几何运算全放容器内的 python 里做：要先读一次机器人当前位姿（x, y, yaw）
# 再算人的落点，还要按 roll 90° 换算四元数。在 shell 里做这件事只会得到
# 一堆难以调试的引号地狱。
#
# 关键点：`person` 是 static 模型，不参与动力学，所以可以用 set_pose 直接搬
# （P2 里搬机器人的时候踩过坑 —— 机器人不是 static，set_pose 会被物理引擎顶回去）。
#
# 旋转中心在**模型原点＝脚底**，绕 X 轴 +90° 后：
#     local (x, y, z) -> world (x, -z, y)
# 即整条身体沿世界 -y 横躺，而身体中轴的高度 = position.z（因为中轴点的 local y = 0）。
# 于是「让橙色躯干中心落在目标点」就等价于把原点沿 +y 平移 TORSO_C 米。
apply_pose() {
  local mode="$1"
  ros_script <<EOS
export MODE='$mode' DIST='$DIST' LIE_Z='$LIE_Z' STAND_Z='$STAND_Z' WORLD='$WORLD'
python3 - <<'PY'
import math, os, re, subprocess, sys, time

world = os.environ['WORLD']
mode = os.environ['MODE']
dist = float(os.environ['DIST'])
lie_z = float(os.environ['LIE_Z'])
stand_z = float(os.environ['STAND_Z'])

# 人模型分段高度（models/person/model.sdf）：腿 0~0.72 / 躯干 0.72~1.47 / 头到 1.69
TORSO_C = 1.095          # 躯干中心在模型系里的 z


def num(block, key):
    m = re.search(key + r': ([0-9.eE+-]+)', block)
    return float(m.group(1)) if m else 0.0


def read_pose():
    """返回 {name: (x, y, z)}，读 /world/<world>/pose/info。"""
    raw = subprocess.run(['gz', 'topic', '-e', '-t', '/world/%s/pose/info' % world, '-n', '1'],
                         capture_output=True, timeout=25).stdout.decode('utf-8', 'replace')
    out = {}
    for blk in raw.split('name: "')[1:]:
        name = blk.split('"')[0]
        mp = re.search(r'position \{(.*?)\}', blk, re.S)
        if mp:
            out[name] = (num(mp.group(1), 'x'), num(mp.group(1), 'y'), num(mp.group(1), 'z'))
    return out


# P2 的 lightdog_person_sim 会以 5Hz 沿既定路径搬运同一个 person 实体。
# 它一在跑，下面的 set_pose 立刻被覆盖 —— 而服务照样回 true，于是表现为
# 「摆位成功、/fall/state 却一直 absent」。这里先让它让路（参数回调）。
def node_alive(name):
    try:
        r = subprocess.run(['ros2', 'node', 'list'], capture_output=True, timeout=20)
        return name in r.stdout.decode('utf-8', 'replace')
    except Exception:
        return False


if node_alive('/lightdog_person_sim'):
    try:
        r = subprocess.run(['ros2', 'param', 'set', '/lightdog_person_sim', 'enabled', 'false'],
                           capture_output=True, timeout=20)
        ok = b'Set parameter successful' in r.stdout
        print('  person_sim -> %s' % ('已暂停' if ok else '暂停失败，位姿可能被覆盖'))
        if not ok:
            print('    (stdout: %s)' % r.stdout.decode('utf-8', 'replace').strip()[:120])
    except Exception as e:
        print('  person_sim 暂停异常：%s' % e)
    time.sleep(0.5)

try:
    raw = subprocess.run(['gz', 'topic', '-e', '-t', '/world/%s/pose/info' % world, '-n', '1'],
                         capture_output=True, timeout=25).stdout.decode('utf-8', 'replace')
except Exception as e:
    print('读 pose/info 失败：%s' % e, file=sys.stderr)
    sys.exit(1)

rx = ry = ryaw = None
for blk in raw.split('name: "')[1:]:
    name = blk.split('"')[0]
    if not name.startswith('linorobot2_'):
        continue
    mp = re.search(r'position \{(.*?)\}', blk, re.S)
    mq = re.search(r'orientation \{(.*?)\}', blk, re.S)
    if not mp:
        continue
    rx, ry = num(mp.group(1), 'x'), num(mp.group(1), 'y')
    yaw = 0.0
    if mq:
        q = mq.group(1)
        qx, qy, qz, qw = (num(q, k) for k in 'xyzw')
        yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    ryaw = yaw
    break

if rx is None:
    print('没在 /world/%s 里找到 linorobot2_* 实体' % world, file=sys.stderr)
    sys.exit(1)

cx = rx + math.cos(ryaw) * dist
cy = ry + math.sin(ryaw) * dist

if mode == 'fall':
    # 躺倒：绕自身 +X 轴 90°（四元数 x=w=sin45=cos45），身体沿 -y 横躺，
    # 原点沿 +y 挪 TORSO_C，让橙色躯干正好压在目标点上。
    px, py, pz = cx, cy + TORSO_C, lie_z
    ox, oy, oz, ow = 0.70710678, 0.0, 0.0, 0.70710678
else:
    # 站立：模型原点就是脚底，直接放到目标点
    px, py, pz = cx, cy, stand_z
    ox, oy, oz, ow = 0.0, 0.0, 0.0, 1.0

print('  robot  x=%.3f y=%.3f yaw=%.1f deg' % (rx, ry, math.degrees(ryaw)))
print('  person x=%.3f y=%.3f z=%.3f  roll=%s' %
      (px, py, pz, '90deg' if mode == 'fall' else '0deg'))

req = ('name: "person", position: {x: %.4f, y: %.4f, z: %.4f}, '
       'orientation: {x: %.6f, y: %.6f, z: %.6f, w: %.6f}' %
       (px, py, pz, ox, oy, oz, ow))

r = None
out = ''
# 每个进程的第一次 gz service 调用常因服务发现没完成而超时（实测必现一次），
# 重试即可，不要当失败。
for attempt in range(1, 4):
    r = subprocess.run(['gz', 'service', '-s', '/world/%s/set_pose' % world,
                        '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
                        '--timeout', '3000', '--req', req],
                       capture_output=True, timeout=25)
    out = (r.stdout + r.stderr).decode('utf-8', 'replace').strip()
    print('  set_pose[%d] -> %s' % (attempt, out or 'no output'))
    if 'true' in out.lower():
        break
    time.sleep(0.5)
if 'true' not in out.lower():
    sys.exit(1)

# 回读校验：set_pose 返回 true 只说明服务收下了请求，不保证实体真的动了
# （被其它节点覆盖、实体名对不上都会这样）。差得离谱就当失败。
time.sleep(0.6)
pos = read_pose().get('person')
if pos is None:
    print('  回读失败：世界里的 person 实体不见了', file=sys.stderr)
    sys.exit(1)
err = math.hypot(pos[0] - px, pos[1] - py)
print('  回读    x=%.3f y=%.3f z=%.3f  偏差 %.3fm' % (pos[0], pos[1], pos[2], err))
if err > 0.15:
    print('  位姿没生效（被其它节点覆盖？检查 lightdog_person_sim 是否仍在跑）',
          file=sys.stderr)
    sys.exit(1)

# 坐标对了不等于「检测器看得见」：人可能正好被墙/柱子挡住，或者落在视场外。
# 以前就是这样静默失败的 —— set_pose 回 true、身位打印得漂漂亮亮，
# /fall/state 却一直是 absent。这里直接把检测器的话读回来。
if node_alive('/lightdog_fall_detector'):
    want = 'lying' if mode == 'fall' else 'standing'
    seen = None
    deadline = time.time() + 10.0
    while time.time() < deadline:
        try:
            r = subprocess.run(['ros2', 'topic', 'echo', '/fall/state', '--once',
                                '--qos-reliability', 'best_effort'],
                               capture_output=True, timeout=6)
            txt = r.stdout.decode('utf-8', 'replace')
            m = re.search(r'data:\s*(\w+)', txt)
            if m:
                seen = m.group(1)
                if seen == want:
                    break
        except Exception:
            pass
        time.sleep(0.5)
    print('  检测器  /fall/state=%s（期望 %s）' % (seen or '?', want))
    if seen != want:
        print('  检测器看不到人：多半是机器人被挡住或人落在视场外。', file=sys.stderr)
        print('  机器人当前 yaw 指向的位置可能贴着墙/柱子。干净的做法是', file=sys.stderr)
        # 注意：上面是 <<EOS（不带引号）的 heredoc，反引号和美元符都会被 shell 展开，
        # 所以这里不能写命令的引号形式，否则会把 p2.sh 真的执行一遍。
        print('  bash sim/p2.sh reset 回出生点后重新走一遍。', file=sys.stderr)
        sys.exit(1)
PY
EOS
}

show_pose() {
  # 这里必须是 <<EOS（不带引号）：`${WORLD:-...}` 写在带引号的 heredoc 里时，
  # 展开发生在**容器内**的 bash，而 docker exec 不继承宿主环境变量 —— 于是
  # 无论 sim/.env 里 WORLD 改成什么，这里永远读 turtlebot3_world，
  # status 就会对着一个不存在的世界报「什么都还没生成」。
  # 正文里没有 $ 和反引号（apply_pose 用的也是这种写法），可以安全展开。
  ros_script <<EOS
export WORLD='$WORLD'
python3 - <<'PY'
import math, os, re, subprocess

world = os.environ['WORLD']
try:
    raw = subprocess.run(['gz', 'topic', '-e', '-t', '/world/%s/pose/info' % world, '-n', '1'],
                         capture_output=True, timeout=25).stdout.decode('utf-8', 'replace')
except Exception:
    print('  (读不到 pose/info)')
    raise SystemExit(0)


def num(block, key):
    m = re.search(key + r': ([0-9.eE+-]+)', block)
    return float(m.group(1)) if m else 0.0


found = False
for blk in raw.split('name: "')[1:]:
    name = blk.split('"')[0]
    if name != 'person' and not name.startswith('linorobot2_'):
        continue
    found = True
    mp = re.search(r'position \{(.*?)\}', blk, re.S)
    mq = re.search(r'orientation \{(.*?)\}', blk, re.S)
    pos = '(?)'
    if mp:
        p = mp.group(1)
        pos = 'x=%.3f y=%.3f z=%.3f' % (num(p, 'x'), num(p, 'y'), num(p, 'z'))
    roll = 0.0
    if mq:
        q = mq.group(1)
        qx, qy, qz, qw = (num(q, k) for k in 'xyzw')
        # roll = atan2(2(wx+yz), 1-2(x^2+y^2))
        roll = math.degrees(math.atan2(2.0 * (qw * qx + qy * qz),
                                       1.0 - 2.0 * (qx * qx + qy * qy)))
    print('  %-20s %-30s roll=%+.0f' % (name[:20], pos, roll))

if not found:
    print('  (世界里的东西都还没生成出来)')
PY
EOS
}

one_shot() {
  # 只取一条消息。/fall/* 是 BEST_EFFORT，偶尔会等满超时，所以给足 15s。
  #
  # --truncate-length 必须有：Jazzy 的 `ros2 topic echo` 默认把**字符串/数组**
  # 截到 128 个字符，后面补 `...`。`/alert/state` 是几百字符的 JSON，
  # 不放开就只能看到 `{"state": "ALARM", "age": 12.3, "contact_...` 半截。
  ros "timeout 15 ros2 topic echo $1 ${2:-} --once --truncate-length 999999 2>&1 | head -3"
}

cmd="${1:-status}"

case "$cmd" in

  start)
    need_container
    kill_p5
    ros "nohup ros2 launch lightdog lightdog_p5.launch.py > /tmp/p5.log 2>&1 & echo started"
    echo "P5 已拉起（日志 /tmp/p5.log）。等 8 秒再查状态："
    sleep 8
    ros "ros2 node list 2>/dev/null | grep -E 'lightdog_(fall_detector|alert_manager)' || echo '  (还没出现在 node list)'"
    echo
    echo "下一步：bash sim/p5.sh spawn && bash sim/p5.sh stand"
    ;;

  stop)
    need_container
    kill_p5
    echo "P5 节点已停。手机端 /status 里的 alert 会变空（后端保留最后一条）"
    ;;

  spawn)
    need_container
    # 复用 P2 的生成逻辑：同一个模型、同一条 spawn 路径，不搞两套人
    WORLD="$WORLD" SPAWN_HX="${SPAWN_HX:-0.5}" SPAWN_HY="${SPAWN_HY:-0.5}" \
      bash "$(cd "$(dirname "$0")" && pwd)/p2.sh" spawn
    echo
    echo "接着用 bash sim/p5.sh stand 把人摆到机器人正前方"
    ;;

  stand)
    need_container
    echo "=== 让人站起来 ==="
    apply_pose stand || { echo "摆位失败" >&2; exit 1; }
    sleep 2
    echo "=== /fall/state ==="
    one_shot /fall/state std_msgs/msg/String
    ;;

  fall)
    need_container
    echo "=== 让人躺倒（roll 90deg）==="
    apply_pose fall || { echo "摆位失败" >&2; exit 1; }
    echo
    echo "已放倒。用 bash sim/p5.sh watch 看状态机爬台阶："
    echo "  NORMAL -(躺倒)-> SUSPECTED -(5s)-> CONFIRMING -(15s)-> ALARM"
    ;;

  z)
    need_container
    echo "=== /fall/z_p95（人体点云离地高度 95 分位，米）==="
    ros "timeout 15 ros2 topic echo /fall/z_p95 --once 2>&1 | head -2" | sed 's/^/  /'
    echo "  判据：<=0.50 躺倒 / >=0.68 站立 / 之间 unknown"
    ;;

  watch)
    need_container
    echo "实时监控（Ctrl+C 退出）：fall | z95 | alert"
    # 注意 alert 那一路：--truncate-length 不能省。默认 128 字符会把 JSON 截断成
    # 半截，下面 json.loads 直接抛异常、被 except 吞掉 -> [alert] 一行都不显示，
    # 看起来像「告警状态没在发」，其实是显示端的锅。
    ros_script <<'EOS'
( timeout 150 ros2 topic echo /fall/state std_msgs/msg/String 2>/dev/null \
    | grep -E '^data:' | sed 's/^data: /[fall ] /' ) &
( timeout 150 ros2 topic echo /fall/z_p95 std_msgs/msg/Float32 2>/dev/null \
    | grep -E '^data:' | sed 's/^data: /[z95  ] /' ) &
( timeout 150 ros2 topic echo /alert/state std_msgs/msg/String --truncate-length 999999 2>/dev/null \
    | grep -E '^data:' | sed 's/^data: //' \
    | python3 -u -c '
import json, sys, time
prev = None
for line in sys.stdin:
    try:
        a = json.loads(line)
    except Exception:
        continue
    key = (a.get("state"), a.get("rounds"), len(a.get("call_attempts") or []),
           a.get("event_id"))
    if key == prev:
        continue
    prev = key
    print("[alert] %-10s age=%5.1fs 联系人=%s 呼叫=%s/%s 视频=%s" % (
        a.get("state"), a.get("age", 0), a.get("contact_count", 0),
        a.get("rounds", 0), a.get("max_rounds", 0), a.get("event_id") or "--"))
' ) &
wait
EOS
    ;;

  test)
    need_container
    pub /alert/command std_msgs/msg/String '{data: test}' || exit 1
    echo "已下发 test（跳过两级确认，直接进 ALARM）。2 秒后状态："
    sleep 2
    one_shot /alert/state std_msgs/msg/String
    ;;

  ack)
    need_container
    pub /alert/command std_msgs/msg/String '{data: ack}' || exit 1
    echo "已下发 ack（确认已知晓，停止重复呼叫）"
    sleep 2
    one_shot /alert/state std_msgs/msg/String
    ;;

  reset)
    need_container
    pub /alert/command std_msgs/msg/String '{data: reset}' || exit 1
    echo "已下发 reset（回到 NORMAL）"
    sleep 2
    one_shot /alert/state std_msgs/msg/String
    ;;

  debug)
    need_container
    # 调试图落到宿主的 data 目录（= 容器的 /data = 存储卡），方便直接打开看
    OUT="${P5_DEBUG_OUT:-/data/fall_debug.png}"
    echo "抓一张 /fall/debug_image（最多等 25 秒）..."
    ros_script <<EOS
export OUT='$OUT'
python3 - <<'PY'
import os
import sys
import time

import numpy as np
import rclpy
import cv2
from sensor_msgs.msg import Image

# 不能用 ros2 topic echo 抓这张图：它对 uint8[] 的 YAML 输出是**一个字节一行**
# （640x480x3 = 921600 行），20 秒都打不完，文件里只剩前几百个字节。
# 直接用 rclpy 订阅一次更省事，也快。
# 注意：这段 python 在**未加引号的 heredoc** 里，所以注释里也不能出现反引号
# 和 $ —— 它们会被宿主 shell 当成命令替换先执行掉（踩过一次）。
out = os.environ['OUT']
got = {}

rclpy.init()
node = rclpy.create_node('p5_debug_grab')


def cb(msg):
    h, w = msg.height, msg.width
    a = np.frombuffer(msg.data, dtype=np.uint8)[:h * w * 3].reshape(h, w, 3)
    got['img'] = a.copy()


node.create_subscription(Image, '/fall/debug_image', cb, 2)
t0 = time.time()
while 'img' not in got and time.time() - t0 < 25.0:
    rclpy.spin_once(node, timeout_sec=0.2)

node.destroy_node()
rclpy.shutdown()

if 'img' not in got:
    print('没收到 /fall/debug_image —— 检测器没在跑？先 bash sim/p5.sh start')
    sys.exit(1)

cv2.imwrite(out, got['img'])
print('已保存 %s（%dx%d, %d 字节）' %
      (out, got['img'].shape[1], got['img'].shape[0], os.path.getsize(out)))
PY
EOS
    # 别写死 fall_debug.png：P5_DEBUG_OUT 一改，这句话就在骗人。
    # 容器里 /data 就是宿主机的 <repo>/data（compose 挂的卷）；写在别处宿主看不到。
    case "$OUT" in
      /data/*) echo "宿主机路径：$(cd "$(dirname "$0")/.." && pwd)/data/${OUT#/data/}" ;;
      *)      echo "容器内路径：$OUT（不在 /data 下，宿主机看不到，用 docker cp 取）" ;;
    esac
    ;;

  probe)
    need_container
    echo "=== 告警 HTTP 接口自检 @ 127.0.0.1:$PORT ==="
    for p in /api/v1/alarms /api/v1/status; do
      # -o NUL 而不是 /dev/null：脚本 export 了 MSYS_NO_PATHCONV=1
      out=$(curl_api "$p" -o NUL -w '%{http_code} %{content_type} %{size_download}')
      printf '  %-22s %s\n' "$p" "$out"
    done
    # 非法动作必须 400 —— 顺便验证路由前缀没有把 /alarms/xxx 当通配吞掉
    out=$(curl_api /api/v1/alarms/bogus -o NUL -X POST -w '%{http_code} %{size_download}')
    printf '  %-22s %s   (期望 400)\n' "POST /alarms/bogus" "$out"
    echo
    echo "=== 告警状态机（截断）==="
    curl_api /api/v1/alarms | head -c 360
    echo
    ;;

  log)
    need_container
    ros "tail -60 /tmp/p5.log 2>&1"
    ;;

  status)
    need_container
    echo "=== 容器 ==="
    "$DOCKER" ps --format '{{.Names}} | {{.Status}}' | grep "^$CONTAINER"
    echo
    echo "=== P5 节点 ==="
    ros "ros2 node list 2>/dev/null | grep -E 'lightdog_(fall_detector|alert_manager)' || echo '  (未启动)'"
    echo
    echo "=== 世界里的身位 ==="
    show_pose
    echo
    echo "=== 姿态判据 ==="
    ros "timeout 10 ros2 topic echo /fall/state --once 2>&1 | head -2" | sed 's/^/  /'
    ros "timeout 10 ros2 topic echo /fall/z_p95 --once 2>&1 | head -2" | sed 's/^/  /'
    echo
    echo "=== 告警状态机 ==="
    ros "timeout 10 ros2 topic echo /alert/state --once --truncate-length 999999 2>&1 | head -2"
    echo
    echo "=== 告警记录（HTTP /api/v1/alarms）==="
    body=$(curl_api /api/v1/alarms)
    if [ -z "$body" ]; then
      echo "  (后端没响应，先 bash sim/p4.sh start)"
    else
      # 先把响应塞进容器（ros_script 的 stdin 已经被脚本本身占用，不能再灌数据）
      printf '%s' "$body" | "$DOCKER" exec -i "$CONTAINER" bash -c 'cat > /tmp/p5_alarms.json'
      ros_script <<'EOS'
python3 - <<'PY'
import json, time

try:
    d = json.load(open('/tmp/p5_alarms.json'))
except Exception as e:
    print('  解析 /api/v1/alarms 失败：%s' % e)
    raise SystemExit(0)

cur = d.get('current') or {}
print('  当前：%s  联系人 %s 个  呼叫 %s/%s 轮  视频 %s' % (
    cur.get('state') or '--', cur.get('contact_count', 0),
    cur.get('rounds', 0), cur.get('max_rounds', 0), cur.get('event_id') or '--'))

rows = d.get('alarms') or []
if not rows:
    print('  (还没有告警记录)')
for a in rows[:6]:
    t = time.strftime('%m-%d %H:%M:%S', time.localtime(a.get('alarm_wall') or 0))
    print('  %s  %-24s %-9s 呼叫%d次  视频 %s' % (
        t, a.get('alarm_id', ''), a.get('state', ''),
        len(a.get('call_attempts') or []), a.get('event_id') or '--'))
PY
EOS
    fi
    ;;

  *)
    sed -n '2,26p' "$0" | sed 's/^# \{0,1\}//'
    exit 1
    ;;
esac
