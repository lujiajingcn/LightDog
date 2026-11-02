# 真机部署：把节点变成 systemd 服务

仿真里靠 `bash sim/xxx.sh start` 手动拉起一串节点，这在真车上不够用 ——
没人插电之后去敲命令，进程崩了没人重启，日志涨到把存储卡写满。

这个目录解决的就这一件事：**开机自启 + 崩溃重启 + 日志有界**。

仿真继续用 `sim/*.sh`，两者互不干扰（这套脚本检测的是 `/opt/ros` 与 workspace，
不会碰容器）。

---

## 快速开始

```bash
cd ~/linorobot2_ws/src/lightdog

sudo ./deploy/install.sh --dry-run     # 先看会生成什么，不落盘
sudo ./deploy/install.sh               # 安装（探测 workspace / ROS 发行版）
sudo ./deploy/verify.sh                # 自检：unit 语法、路径、进程、话题
sudo systemctl start lightdog.target   # 拉起全部
systemctl status lightdog.target       # 一眼看谁没起来
```

装完就有了 `lightdog-estop` 这个命令（见下面「急停」）。

---

## 为什么是「一个节点一个 unit」

`Restart=` 是 **unit 级**的。如果按 launch 分组：

- `alert_manager`（告警状态机）和 `fall_detector`（CV 推理）在同一个 launch 里。
  检测器最容易崩（内存、模型加载），它一崩就整组重启 —— **正在拨的求救电话被静默清零**，
  而日志里只会看到「服务重启成功」。
- 反过来，`lightdog_p4.launch.py` 里 recorder 和 HTTP 服务绑在一起，
  HTTP 端口被占也会把正在录像的 recorder 一起停掉。

所以粒度是单个节点。代价是 unit 数量多，但生成逻辑集中在 `nodes.conf`，
加节点只改一行。

`nodes.conf` 里 `default=no` 的两个（`led` / `person-sim`）**只装不启用** ——
它们只对 Gazebo 有意义，真机上跑了反而让人误以为灯在受控
（`docs/hardware_readiness.md` §二）。

---

## 日常操作

```bash
# 起停全部
sudo systemctl start  lightdog.target
sudo systemctl stop   lightdog.target
sudo systemctl restart lightdog.target

# 单个节点
sudo systemctl restart lightdog-mission
systemctl status lightdog-safety-mux

# 日志（按节点过滤，装的时候就设好了 SyslogIdentifier）
sudo journalctl -t lightdog-mission -f
sudo journalctl -t lightdog-alert -n 100 --no-pager
sudo journalctl -u lightdog.target -b          # 本次开机全部

# 改了参数
sudo systemctl restart lightdog.target
# 参数文件在 install space，改源码里的 yaml 之后要先 colcon build
```

### 急停

```bash
lightdog-estop                 # 看当前锁存 + mux 实际生效的闸
sudo lightdog-estop set        # 立刻刹车（不需要 sudo，除非权限受限）
sudo lightdog-estop clear --yes
```

`clear` 必须带 `--yes` —— 少一个字母就把车解锁，这种事不该靠手气。

这个命令做三件事，任一步不成立就退非零：**发 → 回读锁存 → 回读 mux 实际闸**。
不是 `ros2 topic pub --once` 那种发完就走的做法（短突发会丢，而且按订阅者丢，
会出现「一个锁上、一个还自以为能动」，见 `docs/firmware_protocol.md` §三.3）。

---

## 急停为什么能跨重启

`twist_mux` 配的是 `Restart=always`：崩了 2 秒后自己回来。
而急停闭锁原本只存在内存里 —— 重启 = `estop=False` = **按着急停的车又能动了**。

所以 `safety-mux` 这个 unit 额外传了：

```
-p estop_state_path:=/var/lib/lightdog/estop
```

闭锁一置位就原子落盘（写临时文件 + `fsync` + `rename`），解除时删文件，
启动时先读回再建任何订阅。日志长这样：

```
[ERROR] [lightdog_twist_mux]: 从 /var/lib/lightdog/estop 恢复了急停闭锁 ——
        本节点重启过，车仍然是锁住的。确认现场安全后再显式解除
```

状态文件内容读不懂时**按已急停处理**：这个方向的错误是「车不动，日志让你去删文件」，
反方向是「以为锁着其实能动」。安全件只接受前一种。

想手工清掉：确认现场安全后 `sudo lightdog-estop clear --yes`
（它会同时清掉文件、让运行中的 mux 真解锁；`rm` 只能骗过下次重启，骗不过现在）。

---

## 环境变量

`/etc/lightdog/env`（`install.sh` 只在不存在时创建）：

```bash
# 多台车在同一个 WiFi 里必须给不同值，否则会互相收到对方的话题
ROS_DOMAIN_ID=7
# 真机上 CycloneDDS 通常更省内存、发现更快
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

改完 `sudo systemctl restart lightdog.target`。

`use_sim_time` **不用在这里改** —— unit 里统一带了 `-p use_sim_time:=false`
（实测命令行 `-p` 会覆盖 `--params-file` 里的值），所以源码里的 yaml 保持仿真默认即可，
仿真与真机共用同一份参数文件。

---

## 这套东西不做什么

| 不会做的事 | 为什么 / 该找谁 |
|---|---|
| 装 ROS、装依赖、`colcon build` | 它假设 workspace 已经构建好（检查 `install/lightdog/lib/lightdog`） |
| 改 `_params.yaml` 里的值 | 那些是标定参数，见 `docs/hardware_readiness.md` §一 |
| 替代 MCU 看门狗 | 上位机进程死了，只有固件断 PWM 能救，见 `docs/firmware_protocol.md` §二 |
| 管 WiFi、配静态 IP | 用 `nmcli` / `nmtui` |
| 自动升级 | 没做。真机上自动升级 = 自动引入新 bug |

`network-online.target` 只在 `After=` 里等 —— 没启 `NetworkManager-wait-online`
时不阻塞（target 不存在就跳过），所以别指望它保证网络已就绪。HTTP 服务起不来会
被 `Restart=always` 一直重试，日志里能看见。

---

## 卸载

```bash
sudo ./deploy/install.sh --uninstall
```

删 unit、target、`/usr/local/bin/lightdog-estop`。
**保留** `/etc/lightdog/env`、`/var/lib/lightdog`（里面有急停状态）、`/data`（录像）——
这些是你的数据，脚本不替你决定要不要删。
