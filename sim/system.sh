#!/usr/bin/env bash
# LightDog 系统层助手：任务调度 + 语音链路 + 安全闸（宿主机 Git Bash 操作容器 sim）
#
#   bash sim/system.sh start             # 拉起 mission + voice
#   bash sim/system.sh stop              # 停掉两个节点
#   bash sim/system.sh status            # 模式 / 跟随放行 / 急停 / 语音话题
#   bash sim/system.sh follow            # 命令：开始跟随
#   bash sim/system.sh follow_off        # 命令：停止跟随
#   bash sim/system.sh dock              # 命令：回充（有 Nav2 就先导航到桩前）
#   bash sim/system.sh patrol_on         # 命令：巡游（需 system_params.yaml 有路点）
#   bash sim/system.sh estop             # 急停（闭锁）
#   bash sim/system.sh clear_estop       # 解除急停
#   bash sim/system.sh say 该吃药了       # 走一遍 TTS 链路
#   bash sim/system.sh listen 把灯打开    # 模拟 ASR：往 /voice/command 发一条文本
#   bash sim/system.sh light on|off|auto # 手动灯光
#   bash sim/system.sh log               # 看日志
#
# 典型验收流程（不依赖硬件）：
#   bash sim/system.sh start
#   bash sim/system.sh status                    ← mode=IDLE，follow=false
#   bash sim/p2.sh start && bash sim/p2.sh spawn  ← 有人 -> mode=FOLLOW
#   bash sim/system.sh estop                     ← /cmd_vel_mux_state=estop，车立刻停
#   bash sim/system.sh clear_estop
#   bash sim/system.sh listen 把灯打开            ← /led/brightness 升到 0.85
set -u

export MSYS_NO_PATHCONV=1

CONTAINER="${CONTAINER:-sim}"

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

kill_system() {
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
  ros "f() { ps -eo pid,args --no-headers | grep -E 'lightdog_(mission|voice)\\.py|lightdog_system\\.launch\\.py' | grep -v grep | awk -v me=\$\$ '\$1 != me {print \$1}'; }; \
       pids=\$(f); if [ -n \"\$pids\" ]; then kill -TERM \$pids 2>/dev/null; sleep 2; fi; \
       pids=\$(f); if [ -n \"\$pids\" ]; then kill -9 \$pids 2>/dev/null; fi; sleep 1; true"
}

# 发命令：一条消息 + 发布者多活一会儿，而不是连发。
#
# 这里踩过两个坑，改之前务必看完：
#
# 1) `ros2 topic pub --once` 会丢消息：短命进程在 DDS 发现完成前就退出了，
#    RELIABLE 的 writer 还没等到 ACK 就被析构，未确认的样本直接没了。
#    实测急停发不进去，等 1 秒看状态还是 idle。
#
# 2) 「连发 6 条」是更糟的补法：这台机器上 `-r 10` 实际只能跑到 2~3Hz
#    （CLI 启动 + CPU 被仿真占满），6 条要 3~6 秒才发完。收端看到的是
#    **同一条指令被执行 6 次** —— 语音重复播报、dock 被连点、
#    日志里 6 行一模一样。去重窗口根本盖不住 6 秒。
#
# 正确做法：`sim/pub_once.py` 只 publish 一次，但让 DataWriter 多活 1 秒，
# 由 RELIABLE 自己完成重传。收端恰好收到 1 条，且能报出订阅者数量 ——
# 「发了但没人收」这种静默失败从此可查。
#
# 3) 回显里别用 `<` `>` 当箭头。`echo sent: /light/mode <- off` 会被 bash 解析成
#    「stdin 重定向自文件 `-`」，报 `bash: -: No such file or directory` ——
#    整条命令直接失效，而且报错离真正原因隔着好几层（docker exec 里）。
PUB_HOLD="${PUB_HOLD:-1.0}"

pub_raw() {  # pub_raw <topic> <type> <yaml-body> [hold] [durable]
  local hold="${4:-$PUB_HOLD}" dur="${5:-0}"
  ros "timeout 25 python3 /sim/pub_once.py $1 $2 '$3' $hold $dur"
}

send() {  # send <topic> <type> <text>
  # 单引号会破坏容器那层的引号结构，直接剔除（中文指令里也不该有）
  local text="${3//\'/}" out
  out=$(pub_raw "$1" "$2" "{data: \"$text\"}")
  if echo "$out" | grep -q 'subscribers=0'; then
    echo "!! $1 当前没有订阅者，命令没人收（节点没起？话题名写错？）" >&2
    return 1
  fi
  if echo "$out" | grep -q '^sent '; then
    echo "sent $1 = $text"
    return 0
  fi
  echo "!! 发送失败：$out" >&2
  return 1
}

# 读一个 String 话题的当前值，去掉 "data: " 前缀。
# 注意 sed 用**单引号**：整个表达式是在 ros() 的双引号里展开的，
# 单引号能原样活着，双引号会需要层层转义。
state_read() {  # state_read <topic>
  ros "timeout 8 ros2 topic echo $1 std_msgs/msg/String --once 2>/dev/null | sed -n 's/^data: //p' | head -1"
}

# 亮度是 Float32，单独一个读取口（用来确认灯光/语音链路真的生效）
level_read() {
  ros "timeout 8 ros2 topic echo /led/brightness std_msgs/msg/Float32 --once 2>/dev/null | sed -n 's/^data: //p' | head -1"
}

# 调度层在不在（急停的确认要按「有几个消费者」来定标准，不能硬编码）
mission_running() {
  ros "timeout 8 ros2 node list 2>/dev/null | grep -q lightdog_mission"
}

cmd="${1:-status}"

case "$cmd" in

  start)
    need_container
    kill_system
    # 再确认一次没有残留：kill 没清干净时宁可报错，也不要起第二份。
    # 两个 mission 同时跑会各自发 /follow/enable 与 /mission/hold_motion，
    # 跟随器收到的最后一条谁也说不好，排查起来像"信号随机丢失"。
    left=$(ros "ps -eo pid,args --no-headers | grep -E 'lightdog_(mission|voice)\.py' | grep -v grep | wc -l")
    if [ "${left:-0}" != "0" ]; then
      echo "!! 还有 $left 个系统层进程没杀掉，先手工检查：" >&2
      ros "ps -eo pid,etime,args --no-headers | grep -E 'lightdog_(mission|voice)\.py' | grep -v grep | sed 's/--ros-args.*//'" >&2
      exit 1
    fi
    ros "nohup ros2 launch lightdog lightdog_system.launch.py > /tmp/system.log 2>&1 & echo started"
    sleep 8
    echo "系统层已拉起，日志 /tmp/system.log（容器内）。当前节点："
    ros "ros2 node list 2>/dev/null | grep -E 'lightdog_(mission|voice)' || echo '  (还没出现，稍等几秒)'"
    ;;

  stop)
    need_container
    kill_system
    echo "系统层已停（跟随/回充节点不受影响）"
    ;;

  status)
    need_container
    echo "=== 节点 ==="
    ros "ros2 node list 2>/dev/null | grep -E 'lightdog_(mission|voice)' || echo '  (未启动)'"
    echo
    echo "=== 调度状态 ==="
    ros "timeout 8 ros2 topic echo /mission/state std_msgs/msg/String --once 2>/dev/null | grep data || echo '  (读不到)'"
    echo "=== 模式（纯字符串）==="
    ros "timeout 8 ros2 topic echo /mission/mode std_msgs/msg/String --once 2>/dev/null | grep data || echo '  (读不到)'"
    echo "=== 跟随放行 ==="
    ros "timeout 8 ros2 topic echo /follow/enable std_msgs/msg/Bool --once 2>/dev/null | grep data || echo '  (读不到)'"
    echo "=== 安全闸 ==="
    ros "timeout 8 ros2 topic echo /safety/estop std_msgs/msg/Bool --once 2>/dev/null | grep data || echo '  (还没有消息 = 未触发)'"
    echo "=== 仲裁源（estop/hold 会在这里显示）==="
    ros "timeout 8 ros2 topic echo /cmd_vel_mux_state std_msgs/msg/String --once 2>/dev/null | grep data || echo '  (读不到)'"
    echo "=== 亮度 ==="
    b="$(level_read)"
    echo "  ${b:-（读不到）}"
    ;;

  follow)      send /mission/command std_msgs/msg/String follow ;;
  follow_off)  send /mission/command std_msgs/msg/String follow_off ;;
  auto)        send /mission/command std_msgs/msg/String auto ;;
  manual)      send /mission/command std_msgs/msg/String manual ;;
  dock)        send /mission/command std_msgs/msg/String dock ;;
  undock)      send /mission/command std_msgs/msg/String undock ;;
  patrol_on)   send /mission/command std_msgs/msg/String patrol_on ;;
  patrol_off)  send /mission/command std_msgs/msg/String patrol_off ;;

  estop)
    # 急停是安全关键路径：**确认不了就当失败**，绝不打印「已闭锁」了事。
    #
    # 为什么要循环重发：`ros2 topic pub` 是短命进程，RELIABLE 的样本还没被
    # 所有订阅者 ACK 它就析构了。丢包是**按订阅者**发生的 —— 实测出现过
    # mux 收到了、mission 没收到（一个锁上、一个还自以为能动），所以确认
    # 必须两个消费者都点名，只看 mux 会骗过自己。
    ok=0
    for i in 1 2 3 4 5 6; do
      # durable=1：急停是「状态」不是「事件」，用 TRANSIENT_LOCAL 让之后
      # 才启动的节点也能继承"已经急停"这个既成事实（实测 kill 掉 mission
      # 再拉起，0.8s 内就进 ESTOP，不需要任何人重发）。
      pub_raw /safety/estop std_msgs/msg/Bool '{data: true}' 1.5 1 >/dev/null 2>&1
      sleep 0.8
      m="$(state_read /cmd_vel_mux_state)"
      s="$(state_read /mission/mode)"
      [ -n "$s" ] || s="(未运行)"
      if [ "$m" = "estop" ] && { ! mission_running || [ "$s" = "ESTOP" ]; }; then
        echo "急停已闭锁（第 $i 轮确认：mux=estop, mission=$s）"
        ok=1
        break
      fi
      echo "  第 $i 轮未确认：mux=${m:-?} mission=$s，重发"
    done
    if [ "$ok" != "1" ]; then
      echo "!! 急停未确认。先确认 lightdog_twist_mux 在跑：" >&2
      echo "   ros2 node list | grep twist_mux" >&2
      exit 1
    fi
    ;;

  clear_estop)
    ok=0
    for i in 1 2 3 4 5 6; do
      pub_raw /safety/estop std_msgs/msg/Bool '{data: false}' 1.5 1 >/dev/null 2>&1
      sleep 0.8
      m="$(state_read /cmd_vel_mux_state)"
      s="$(state_read /mission/mode)"
      [ -n "$s" ] || s="(未运行)"
      # 解除后 mux 可能是 hold（调度层还在按住，最多 3s 后自己过期），
      # 只要不是 estop 就算解开 —— 但 mission 必须离开 ESTOP。
      if [ -n "$m" ] && [ "$m" != "estop" ] && { ! mission_running || [ "$s" != "ESTOP" ]; }; then
        echo "急停已解除（第 $i 轮确认：mux=$m, mission=$s）"
        ok=1
        break
      fi
      echo "  第 $i 轮未确认：mux=${m:-?} mission=$s，重发"
    done
    if [ "$ok" != "1" ]; then
      echo "!! 解除未确认：仍有一方处于急停" >&2
      exit 1
    fi
    ;;

  say)
    shift
    text="${*:-测试语音}"
    send /alert/voice std_msgs/msg/String "$text"
    ;;

  listen)
    shift
    text="${*:-把灯打开}"
    send /voice/command std_msgs/msg/String "$text"
    ;;

  light)
    # on/off 是「闭锁式覆盖」，发一条就够，但前面说过短突发可能丢，所以
    # 仍然读回亮度确认 —— 这比打印「sent」有用得多。
    mode="${2:-auto}"
    want=1
    case "$mode" in
      on)  want=1 ;;
      off) want=0 ;;
      auto) want=-1 ;;
      *) echo "用法: system.sh light on|off|auto" >&2; exit 1 ;;
    esac
    ok=0
    for i in 1 2 3; do
      send /light/mode std_msgs/msg/String "$mode" >/dev/null 2>&1
      if [ "$want" = "-1" ]; then ok=1; break; fi
      sleep 3                      # 淡入淡出 fade_time 默认 2s，等它走完
      b="$(level_read)"
      echo "  /led/brightness = ${b:-（读不到）}"
      if awk -v b="$b" -v w="$want" \
           'BEGIN{ exit !((w==1 && b>=0.8) || (w==0 && b<=0.05)) }'; then
        echo "灯光 $mode 已确认（第 $i 轮）"
        ok=1
        break
      fi
    done
    if [ "$ok" != "1" ]; then
      echo "!! 灯光 $mode 未确认，检查 lightdog_light_auto 是否 enabled" >&2
      exit 1
    fi
    ;;

  log)
    need_container
    ros "tail -60 /tmp/system.log 2>&1"
    ;;

  *)
    sed -n '2,20p' "$0" | sed 's/^# \{0,1\}//'
    exit 1
    ;;
esac
