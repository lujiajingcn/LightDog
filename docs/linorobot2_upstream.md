# linorobot2 上游子树说明

本仓库的 `linorobot2/` 目录以 **git subtree（--squash）** 方式引入第三方项目
[linorobot/linorobot2](https://github.com/linorobot/linorobot2)，并在其上叠加 LightDog 的产品改造。

| 项 | 值 |
|---|---|
| 上游仓库 | `https://github.com/linorobot/linorobot2` |
| 上游分支 | `jazzy` |
| 基线提交 | `f6b0bd0350c4a9d6b815ef4bb5c701a1e5129711`（Merge PR #242） |
| 本地前缀 | `linorobot2/` |
| 引入方式 | `git subtree add --prefix=linorobot2 <repo> <ref> --squash` |

## 为什么用 subtree 而不是 submodule

`linorobot2/` 里有相当一批**必须随产品走**的改动（LED/电池 URDF、Gazebo 桥接、容器挂载、
launch 修复）。用 submodule 指向上游会让这些改动无处安放（除非再维护一个 fork），
克隆方还需要额外 `--recursive` 和镜像配置。subtree 则是「单次 clone 即完整、改动留在主仓库历史里」，
适合 vendored + patched 的用法。

## 网络前提

本机 `github.com` 直连不通（全 IP 超时），只有 `api.github.com` / `raw.githubusercontent.com` /
`gh-proxy.com` 可用。因此所有拉取命令都要：

1. 清空代理变量（WorkBuddy 进程自带的 `HTTP_PROXY` 指向沙箱代理，会把外网请求吃掉）
2. 走 gh-proxy 镜像

```bash
export http_proxy= https_proxy= HTTP_PROXY= HTTPS_PROXY=
git remote add linorobot2-upstream \
  https://gh-proxy.com/https://github.com/linorobot/linorobot2.git
```

## 升级上游

```bash
export http_proxy= https_proxy= HTTP_PROXY= HTTPS_PROXY=

# 1) 取上游（建议先浅拉，避免一次拉全历史）
git fetch --depth=200 linorobot2-upstream jazzy

# 2) 合并到子树前缀，--squash 让上游历史压缩成一个提交
git subtree pull --prefix=linorobot2 linorobot2-upstream jazzy --squash

# 3) 冲突几乎必然出在下表「本地改造」列出的文件上，按产品需求保留本地语义后提交
```

若 `git fetch` 卡住或 502，换镜像重试（`https://ghproxy.net/https://github.com/...`、
`https://ghfast.top/https://github.com/...`），或先 `--depth=1` 再逐步加深。
**不要**为了走通连接去改 hosts——GitHub 的 IP 会抖动，改 hosts 往往让情况更糟。

## 本地改造清单（冲突高发区）

新增：

```
linorobot2/linorobot2_description/urdf/lightdog/led.urdf.xacro
linorobot2/linorobot2_description/urdf/lightdog/battery.urdf.xacro
linorobot2/docker/sim_start.sh
linorobot2/docker/slam_start.sh
linorobot2/docker/Dockerfile.xvfb
linorobot2/docker/SIM_使用说明.md
```

修改：

```
linorobot2/linorobot2_description/urdf/robots/2wd.urdf.xacro     include + 调用 LED/电池宏
linorobot2/linorobot2_description/urdf/robots/4wd.urdf.xacro     同上
linorobot2/linorobot2_description/launch/description.launch.py   修 robot_description 未声明 str
linorobot2/linorobot2_gazebo/launch/gazebo.launch.py             加 material_color / battery 桥接
linorobot2/docker/Dockerfile                                     追装 xvfb / x11-utils / x11vnc
linorobot2/docker/docker-compose.yaml                            挂载 urdf/launch/lightdog，改代码免重建
linorobot2/docker/.env                                           SIM/WORLD/DISPLAY 等本机仿真配置
```

合计 13 个文件、约 +414 行。上游升级后用这条命令复核本地改造是否还在：

```bash
BASE=$(git log --format=%H --grep="vendor: 引入 linorobot2" -1)
git diff --stat "$BASE" HEAD -- linorobot2
```

## 行尾策略

仓库根 `.gitattributes` 强制 `* text=auto eol=lf`，并对 `*.sh / *.py / *.xacro / *.dae` 显式声明 LF。
原因：宿主机 `core.autocrlf=true`，若不声明，检出会把 `docker/*.sh` 写成 CRLF，
容器里 bash 直接报 `bad interpreter: /bin/bash^M`。
`.dae` 是 XML 文本（不是二进制），误标为 `binary` 会把它冻结成 CRLF，产生整文件的假差异。

新增文本文件后若怀疑行尾被污染：

```bash
git ls-files --eol <path>     # i/=索引  w/=工作区
git add --renormalize .
```

## 不要入库的内容

`linorobot2_navigation/maps/my_map.*` 是 SLAM 跑出来的地图，`**/__pycache__/` 是字节码，
`docker/agent_outputs/` 是临时输出——都已在根 `.gitignore` 里排除。
