# 上游依赖：linorobot2

本仓库**不包含**上游 [linorobot/linorobot2](https://github.com/linorobot/linorobot2) 的源码。
上游只作为底座使用，我们通过 ROS 2 的覆盖层（overlay）机制叠加自己的东西。

| 项 | 值 |
|---|---|
| 上游仓库 | `linorobot/linorobot2` |
| 分支 | `jazzy` |
| 锁定提交 | `f6b0bd0`（见 `upstream.lock`） |
| 运行时依赖 | 已构建的 Docker 镜像 `linorobot2:gazebo` |
| 我们对上游的改动 | 1 处，且由启动脚本幂等修补 |

## 仓库结构

```
lightdog/                 我们的 ROS 2 包（入库）
  urdf/lightdog_{2wd,4wd}.urdf.xacro   覆盖层：上游机座 + LED + 电池
  urdf/{led,battery}.urdf.xacro        我们新增的宏
  launch/lightdog_sim.launch.py        include 上游 gazebo.launch.py 并覆盖 urdf
  lightdog_led.py                      LED 亮度节点
  config/led_visuals.yaml              节点参数
sim/                      容器编排（入库）
  docker-compose.yaml                 基于 linorobot2:gazebo 镜像
  entrypoint.sh                       修补上游 → 构建我们的包 → 起仿真 + SLAM
  patch_upstream.py                   唯一的那个上游补丁
scripts/fetch_upstream.sh 按需拉上游源码（入库）
upstream.lock             版本锁（入库）
linorobot2/               拉下来的上游源码（gitignore，不入库）
```

## 三层叠加关系

1. **镜像层** `linorobot2:gazebo`：ROS 2 Jazzy + Gazebo Harmonic + 上游所有包，已 colcon build。
2. **补丁层** `sim/patch_upstream.py`：容器启动时修补镜像内 `install/.../description.launch.py`。
3. **覆盖层** `lightdog/`：容器启动时挂载进工作区并 `colcon build --symlink-install`。

叠加后 `ros2 launch lightdog lightdog_sim.launch.py` 就是完整的 LightDog 仿真。
上游 `src/` 里一份源码都没被改过，升级时不会产生冲突。

## 为什么不用 subtree / submodule

试过 subtree：仓库从 149 个文件、`.git` 42MB 起步，其中 33MB 是上游文档的
gif/png（`linorobot2.gif` 一个就 16MB），对产品毫无用处；而真实改动只有 13 个文件，
每次升级都要在这 13 个文件里解冲突。

submodule 也不合适：它指向上游，我们的 13 处修改无处安放（除非再维护一个 fork），
克隆方还要 `--recursive` 并单独配镜像。

现在这套：仓库 20 个文件、`.git` 126KB，上游升级只需改 `upstream.lock` 一个 sha。

## 唯一的上游补丁

`description.launch.py` 在 Jazzy 上必崩：

```
[ERROR] [launch]: Caught exception in launch: Unable to parse the value of parameter
robot_description as yaml. If the parameter is meant to be a string, try wrapping it in
launch_ros.parameter_descriptions.ParameterValue(value, value_type=str)
```

上游把 `Command(['xacro ', ...])` 直接塞进 parameters，xacro 输出以 `<?xml` 开头，
launch_ros 按 YAML 解析必然失败。修法就是报错信息里给的标准写法。

`sim/patch_upstream.py` 是幂等的：已打过就 SKIP；如果上游改了这段实现导致锚点找不到，
它会**报错退出**让容器起不来，而不是静默跑一个坏配置。

## 拉上游源码 / 升级上游

```bash
# 拉取（走镜像，本机 github.com 直连不通）
scripts/fetch_upstream.sh
scripts/fetch_upstream.sh --force   # 强制重拉

# 升级：改 upstream.lock 里的 UPSTREAM_COMMIT，然后
scripts/fetch_upstream.sh
cd sim && docker compose --env-file .env up -d --force-recreate
docker logs -f sim      # 确认 patch 步骤是 OK 而不是 FAIL
```

升级后必查三件事：

1. `[patch] OK` —— 不是 FAIL、不是 SKIP（SKIP 说明镜像里已有补丁，OK 说明这次打上了）
2. `/led/brightness` 改值后 `material_color` 的 emissive 有变化
3. `/battery_state` 有数据（gz 传输建链比较慢，可能要等几分钟）

## 环境前提

- `github.com` 本机直连不通，可用通道是 `gh-proxy.com` 镜像 + 清空代理变量
  （WorkBuddy 进程自带的 `HTTP_PROXY` 指向沙箱代理，会把 git 请求吃掉）。
- 宿主 `core.autocrlf=true`，仓库根 `.gitattributes` 已强制 `* text=auto eol=lf`。
  不声明的话，检出会把 `sim/entrypoint.sh` 写成 CRLF，容器里 bash 直接
  `bad interpreter: /bin/bash^M`。
