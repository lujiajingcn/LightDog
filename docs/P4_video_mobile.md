# P4：视频存储 + 手机互联

**状态：仿真端到端跑通**（2026-11-01）
**代码**：`lightdog/lightdog_recorder.py`、`lightdog/lightdog_server.py`、
`lightdog/web/`、`sim/p4.sh`

---

## 一、目标

对应 `Readme.md` 的**使用场景 2（看护老人）**：机器人要能录像、要在人倒地时把
「事发前后」存下来、还要让紧急联系人用手机看到画面。

拆成三件事：

1. **存储**：摄像头画面持续落盘，卡写满自己滚，掉电只丢最后一段
2. **切片**：出事时把「事发前 N 秒 + 事发后 M 秒」单独存一份，能直接播
3. **互联**：手机浏览器打开就能看实时画面、回放事件、配紧急联系人

第 3 条是这一阶段真正的新东西 —— 前两个阶段机器人只在 ROS 内部自转，
从这里开始它要**对外提供服务**。

---

## 二、拓扑

```
  /camera/color/image_raw（640×480，仿真约 15 Hz sim / 3 Hz 墙钟）
        │
        ├──> lightdog_recorder ──> /data/clips/seg_XXXXXX.mp4   环形分段
        │         │                /data/events/<id>/video.mp4  事件切片
        │         │                /data/events/<id>/meta.json
        │         └──> /recorder/state（JSON，1 Hz）
        │         <──  /recorder/event     （P5 往这里发跌倒事件）
        │         <──  /recorder/snapshot
        │
        └──> lightdog_server ──> HTTP :8000
                  │                ├ MJPEG 实时流
                  │                ├ mp4 点播（Range）
                  │                └ 状态 / 事件 / 联系人 REST
                  <── /dock/state, /dock/battery_ratio, /battery_state,
                      /person/present, /cmd_vel_mux_state
```

两个节点**只通过 `/data` 目录和一条 `/recorder/event` 消息耦合**：
关掉后端录像照跑，关掉录像后端仍能出实时流。这是故意的 —— 看护设备的
存储和远程访问是两件独立的事，任何一件挂了都不该拖死另一件。

---

## 三、四个关键设计

### 1. 分段写盘，而不是一路录到底

每 `segment_sec`（默认 10 秒，ROS 时钟）落一个独立 mp4，滚掉最老的。

理由只有一个：**SD 卡会掉电**。一路写一个 2 小时的 mp4，掉电时整个文件都废；
分段之后最多丢当前这一段。这是行车记录仪的标准做法。

文件先写 `seg_XXXXXX.part.mp4` 再 `os.replace()` 改名，保证目录里出现的每个
`.mp4` 都是完整的。

### 2. 事件必须带「事发前」

看护场景里真正有用的是**老人是怎么倒下的**，不是「倒下之后躺在地上」。
所以事件切片要往前捞 `pre` 秒 —— 这要求环形缓冲一直在线。

实现上绕了一下：不在事件当下就合成视频，而是等到 `t_ev + post` 再动手。
因为后置窗口的帧那时才存在；等到那时，窗口内的分段**都已经完整落盘**，
合成只需要读文件，不需要在内存里额外留一份长缓冲。

事件触发时会**立刻强制 flush 当前段**，避免「事发前 2 秒」还卡在内存里等段满。

### 3. 帧率按实测写，不能写死

仿真相机实测约 15 Hz（sim 时间），在 0.22x 实时因子下墙钟只有 3 Hz 左右。
如果按 30 fps 写文件头，回放会变成 6 倍速幻灯片；按 15 写,手机上又是 5 倍速。

所以每段收完帧之后才用 `帧数 / 实际时长` 算出 fps 再写文件头。实测出来
14.3 / 13.9 / 15.5 / 16.6 不等 —— 这正是真实帧率该有的抖动。

### 4. 为什么是 MJPEG-over-HTTP，不是 RTSP / WebRTC

原计划写的是 RTSP 或 WebRTC，实际落地选了 MJPEG-over-HTTP。原因不是做不了：

| 方案 | 能不能做 | 结论 |
|---|---|---|
| MediaMTX / go2rtc 二进制 | 容器**无外网**，下载不了 release | ✗ |
| aiortc（WebRTC） | `pip install` 不通；手写 DTLS/SRTP/ICE 不现实 | ✗ |
| gstreamer `rtspclientsink` | 镜像里没有 gst-launch | ✗ |
| 手写 RTSP + RTP/JPEG (RFC 2435) | 能做，但要剥 DQT/DHT、算 qtable、处理分片，客户端兼容性风险高 | 暂缓 |
| **MJPEG over HTTP** | 浏览器原生、零依赖、`<img>`/canvas 都能吃 | **✓ 采用** |

关键判断：**RTSP 的收益（接第三方 NVR）对这个产品不是刚需**，
而手机浏览器打开就能看才是。用 300 行有兼容性风险的手写 RTSP，
换一个紧急联系人根本不会用的功能，不划算。

顺带说：`/api/v1/stream.mjpg` 是标准 MJPEG-over-HTTP，
**VLC 可以直接打开这个 URL 当流播放** —— 需要第三方客户端时这就是出口。

真正要做 WebRTC（低延迟双向），正路是给镜像装 `aiortc` 或塞一个 `go2rtc` 静态
二进制，这属于构建期的事，记在「已知限制」里。

---

## 四、交付物

| 文件 | 作用 |
|---|---|
| `lightdog/lightdog_recorder.py` | 环形分段 + 事件切片 + 快照 + 配额清理 |
| `lightdog/lightdog_server.py` | HTTP 后端（MJPEG / mp4 / REST），纯标准库 |
| `lightdog/web/index.html` | 手机页面结构 |
| `lightdog/web/style.css` | 暗色移动端样式（无框架） |
| `lightdog/web/app.js` | 流解析、画布渲染、事件列表、联系人 |
| `lightdog/config/p4_params.yaml` | 两个节点的参数 |
| `lightdog/launch/lightdog_p4.launch.py` | 起上述两个节点 |
| `sim/p4.sh` | 宿主机助手（start/stop/status/urls/event/snap/files/probe/watch/log/clean） |
| `sim/docker-compose.yaml` | 端口 `8000`、卷 `../data:/data`、`ENABLE_P4`、`LIGHTDOG_TOKEN` |
| `sim/entrypoint.sh` | 启动后自动拉起 P4（`ENABLE_P4=1` 时） |
| `sim/.env` | `ENABLE_P4`、`LIGHTDOG_TOKEN` |

### 接口

| 方法 | 路径 | 说明 |
|---|---|---|
| GET | `/` | 手机页面 |
| GET | `/api/v1/health` | 健康检查（**唯一免鉴权**） |
| GET | `/api/v1/status` | 电量 / 回充 / 跟随 / 相机 / 存储 / 事件数 |
| GET | `/api/v1/stream.mjpg` | MJPEG 实时流 |
| GET | `/api/v1/snapshot.jpg` | 当前单帧 |
| GET | `/api/v1/events` | 事件列表 |
| POST | `/api/v1/events` | 触发事件 `{type,label,pre,post,note}` → `/recorder/event` |
| GET | `/api/v1/events/<id>` | 事件元数据 |
| GET | `/api/v1/events/<id>/video` | 事件视频（**支持 Range**） |
| GET | `/api/v1/events/<id>/thumb.jpg` | 事件缩略图 |
| GET | `/api/v1/clips` | 环形分段列表 |
| GET | `/api/v1/clips/<name>` | 分段视频（**支持 Range**） |
| GET | `/api/v1/snapshots`、`/api/v1/snapshots/<name>` | 快照 |
| GET/PUT | `/api/v1/contacts` | 紧急联系人（P5 用） |

鉴权：`auth_token` 非空时，`/api/v1/*`（除 health）都要 token，三种带法都收：
`?token=`、`X-Auth-Token:`、`Authorization: Bearer`。**`?token=` 是必须支持的**，
因为 `<video>` / `<img>` 没法带自定义 header。

---

## 五、实测验收

### 5.1 环形分段落盘

```
{"file": "seg_000000.mp4", "seq": 0, "t0": 318.95, "t1": 323.08,
 "wall": 1793541757.8, "frames": 60, "fps": 14.286, "bytes": 8404, "codec": "avc1"}
{"file": "seg_000001.mp4", "seq": 1, "t0": 323.14, "t1": 327.17,
 "wall": 1793541775.9, "frames": 57, "fps": 13.896, "bytes": 8182, "codec": "avc1"}
```

10 秒一段约 150 帧 / 10~16 KB（静态场景，H.264 帧间压缩后极小）。
`codec: avc1` 说明走的是 H.264 —— 这是手机能播的前提。

### 5.2 事件切片（含事发前）

触发 `bash sim/p4.sh event fall`，后置窗口到点后自动合成：

```json
{
  "id": "evt_20261101_142306_fall",
  "type": "fall", "label": "检测到跌倒",
  "t_ev": 109.7, "t_from": 99.7, "t_to": 119.7,
  "clips": ["seg_000021.mp4", "seg_000022.mp4", "seg_000023.mp4"],
  "frames": 299, "fps": 14.99, "bytes": 31198, "codec": "avc1",
  "has_video": true
}
```

299 帧 / 14.99 fps = **19.95 秒**，与 `pre=10 + post=10` 的 20 秒窗口吻合；
来源 3 个分段。`cv2.VideoCapture` 回读确认 **299 帧全部可解码**，
`640×480`、`fps` 与元数据一致。

### 5.3 HTTP 接口自检（`bash sim/p4.sh probe`）

```
/api/v1/health                  200  application/json                       53B
/api/v1/status                  200  application/json                      740B
/api/v1/snapshot.jpg            200  image/jpeg                          13162B
/api/v1/stream.mjpg             200  multipart/x-mixed-replace; boundary=…
/api/v1/events                  200  application/json                      675B
/api/v1/clips                   200  application/json                     1391B
/api/v1/contacts                200  application/json                       16B
/api/v1/snapshots               200  application/json                       17B
/                               200  text/html; charset=utf-8             3725B
/style.css                      200  text/css; charset=utf-8              9278B
/app.js                         200  application/javascript; charset=utf-8 21154B

Range 支持（iOS 播放录像的前提，必须是 206）
/api/v1/events/evt_20261101_142306_fall/video  206  bytes 0-1023/31198
/api/v1/clips/seg_000023.mp4                   206  bytes 0-1023/14761
```

路径穿越拦截：`GET /../etc/passwd` → `400 bad path`。

### 5.4 手机页面（无头 Edge 真实渲染，500×1000）

三个页签都截了图核对：

- **实时**：真实相机画面（turtlebot3_world 的绿色柱子）、`REC` 角标、
  `640×480 · 3.8fps`、电量 `99%`、贴片
  `录像 开 / 分段 10 · 0.13MB / 剩余 117.3GB / 人 无 / 运动源 idle / 事件 1`、
  `拍一张` + `标记事件`
- **事件**：`检测到跌倒 22:11:30 · 20秒 fall` + 缩略图（能看清场景）、
  `最近录像 11 段` 列表
- **设置**：紧急联系人空态 + 添加表单、设备信息 kv 表、
  访问令牌输入框（`留空表示不鉴权`）

### 5.5 「存储卡」语义

`../data` 挂到容器 `/data`，容器重建后文件还在：

```
data/clips/seg_000000.mp4 … seg_000025.mp4
data/clips.jsonl
data/events/evt_20261101_142306_fall/{meta.json,thumb.jpg,video.mp4}
```

实测 `du -sh data` = **559K**（26 段 + 3 个事件）。这就是真机上 SD 卡的角色：
容器（= 机器人本体）可以随便重建，卡里的东西不动。

顺带验证了**索引恢复**：重启后 recorder 打出
`已恢复 12 个历史分段，下一个 seq=12`，接着从 12 往下写，不会覆盖旧文件。

### 5.6 端到端复验（2026-11-02，容器 sim）

一条命令跑完（脚本在包内，直接在容器里执行）：

```
docker exec sim bash -lc 'source /opt/ros/jazzy/setup.bash; \
  source /home/ros/linorobot2_ws/install/setup.bash; \
  python3 /home/ros/linorobot2_ws/src/lightdog/tests/verify_p4.py'
```

结果 27 项全通过。要点：

| 项 | 实测 |
|---|---|
| 静态页 `/` | 200，4904B |
| `snapshot.jpg` | 200，16KB，`640×480` 解码，mean=158 |
| 分段回放 | `seg_000107.mp4` 254 帧 / 10.03s，索引帧数与实测**完全一致** |
| MJPEG | `multipart/x-mixed-replace`，6s 内 3 帧 |
| 事件切片 | `pre=8/post=8` -> 412 帧 / 16.0s，视频 39KB 全可解码，缩略图 640×480 |
| 联系人 | PUT 两个 -> `/data/contacts.json` 210B，读回一致 |
| Range（`p4.sh probe`） | 事件视频与分段均 **206**，`bytes 0-1023/N` |

本轮修的两个问题：

1. **`/api/v1/clips` 只有文件名和大小**，手机端没法显示「这段多长、几点录的」。
   recorder 早就把 `t0/t1/frames/fps` 写进 `clips.jsonl`，后端却没读。
   现在按 `(mtime,size)` 缓存合并进返回值（缺索引时自动退化成旧行为）。
2. **事件窗口字段被静默忽略**：HTTP 层原本把请求 JSON 原样转发，而 recorder
   只认 `pre`/`post`，客户端照 meta 里的 `pre_sec`/`post_sec` 写就会按默认
   10/10 切 —— `202` 受理成功、窗口却是错的。现在两个名字都收，并夹到 0~300s。

另外 `max_event_frames` 从 900 提到 1200：合成是从窗口**头部**往后写的，
撞上限丢的是尾巴（对 P5 就是「倒下之后」那段）。实测 25.9fps × 35s = 910 帧，
旧值已经丢掉 0.3s；真机 30Hz 会丢 5s。40s 窗口复验得 1063 帧 / 40.0s，确认不再截断。

---

## 六、踩的坑

1. **`VideoWriter` 靠扩展名选 muxer**

   为了「先写临时文件再改名」，文件名用了 `seg_000000.mp4.part`。结果
   `cv2.VideoWriter` 打不开编码器，`avc1` 和 `mp4v` 双双失败，一条日志
   `编码器 avc1/mp4v 都打不开，录像不可用` —— 但**控制台还很安静**，
   因为错误日志被 stdout 缓冲压在后面才吐出来。

   改法：临时文件也必须以 `.mp4` 结尾（`seg_000000.part.mp4`）。

2. **空的 `auth_token:=` 让整条 launch 直接退出**

   `ros2 launch ... auth_token:=""` 报
   `malformed launch argument 'auth_token:=', expected format '<name>:=<value>'`，
   于是**两个节点都没起来**，但 entrypoint 只看到 `nohup` 成功，什么也不报。

   改法：参数按需拼接，空值就整个不传。

3. **`MSYS_NO_PATHCONV=1` 会让 `curl -o /dev/null` 失效**

   `p4.sh` 里 `export MSYS_NO_PATHCONV=1`（docker exec 需要）。这之后
   Git Bash 不再把 `/dev/null` 转成 `NUL`，`curl.exe` 去写字面路径
   `\dev\null`（写失败但不报错），于是 `%{size_download}` 恒为 **0**。
   自检表里所有响应都是 `0B`。

   改法：curl 的输出丢弃用 `NUL`；shell 的 `2>/dev/null` 不受影响，不用改。

4. **中文 Windows 的 `ipconfig` 是 GBK，GNU sed 会静默放弃替换**

   `ipconfig | grep IPv4 | sed -E 's/^[^:]*:[[:space:]]*//'` 原封不动返回原行 ——
   `地址` 三个字节不是合法 UTF-8，sed 在 UTF-8 locale 下碰到非法多字节序列
   就整行跳过，**不报错**。

   改法：`awk '{print $NF}'` 按空白取最后一个字段，不受编码影响。

5. **`realpath` 判路径穿越会把正常静态文件全判成越界**

   `colcon --symlink-install` 的 `install(DIRECTORY)` 是「目录真建、
   文件逐个做符号链接」，所以 `web/index.html` 的 realpath 指向
   `src/lightdog/web/index.html`，不在 `web/` 的 realpath 下面 →
   `/` 和 `/app.js` 全部 `400 bad path`。

   改法：先 `normpath` 做**词法**越界判断，再让 `open()` 跟随符号链接。

6. **同一毫秒内两帧 → 页面显示 `Infinityfps`**

   `1000 / (now - lastFrameAt)` 在 `dt == 0` 时是 `Infinity`，而 fps 是
   指数平滑的，一旦变成 `Infinity` 就永远回不来。截图里就是
   `640×480 · Infinityfps`。改法：`now > S.lastFrameAt` 才更新。

7. **设置页显示的是容器 IP**

   `status` 里的 `lan_ip` 是服务端进程视角的地址，跑在 Docker 里就是
   `172.17.0.2`，手机根本连不上，显示出来只会让人以为配错了。
   改法：页面用 `location.host`（浏览器实际访问的地址），永远是对的。

8. **手机端实时画面不用 `<img src="stream.mjpg">`**

   iOS Safari 对 `multipart/x-mixed-replace` 的支持时好时坏，失败了还是静默
   黑屏、`onerror` 也不触发。改法：`fetch` + `ReadableStream` 自己从字节流里
   切 JPEG（扫 `FFD8…FFD9`，熵编码的字节填充保证 `FFD9` 只出现在帧尾），
   拿到 Blob 再画 canvas。所有现代手机浏览器都支持，重连逻辑也自己可控。

9. **ROS 时间不保证跨重启单调，事件视频会混进上一次运行的画面**

   这条是实测抓到的真 bug，也是最值得记的一条。

   容器重启后 `/clock` 从 0 重新开始。索引里的旧分段还带着上次的时间戳
   （`t0=37.26`），新录的分段从 `t0=7.33` 开始 —— 两者**混在同一个索引里**。
   于是事件窗口 `[36.17, 56.17]` 把重启前录的 `seg_000000`（t0=37.26）
   也判成"相交"，一个本该 20 秒的事件剪出 **621 帧 / 32.8 fps**、
   画面在两次运行的场景之间跳。

   对应的清理逻辑也一起错了：`retain_sec` 原本拿 ROS 时间算
   `now - t1`，重启后 `now` 变小、差值为负，旧录像**永远不会被清掉**。

   改法两条：
   - 每个分段记一个 `session`（节点启动时的墙钟秒），**事件切片只在同一会话内取段**
     —— 跨重启的画面本来就不该拼进同一个事件。
   - `retain_sec` 的计时基准从 ROS 时间换成**墙钟**，跨重启成立。

   真机同样会踩：NTP 校时跳变、底盘供电重启后时钟回退，症状一模一样。
   修完复测：299 帧 / 14.99 fps = **19.95 秒**，正好是 20 秒窗口。

---

## 七、已知限制

1. **mp4 的 `moov` 在文件尾部，不是 faststart**

   实测原子顺序：`ftyp / free / mdat / moov`。没有 ffmpeg 就没法做
   `-movflags +faststart`。iOS Safari 靠 Range 请求也能播（先取头、再跳尾读
   moov），但**起播会多一次往返**。片段越大越明显（当前片段才 10~30KB，
   感知不到；真机 30fps 长事件会变明显）。

   要做：给镜像装 ffmpeg，或自己重写 atom 顺序。

2. **没有 RTSP / WebRTC**（见「三、4」）。当前出口是 MJPEG-over-HTTP
   （浏览器 + VLC 都能吃）和 H.264 mp4 点播。

3. **没有音频**。`Readme.md` 说车上有麦克风和扬声器，这一阶段只做了视频；
   ROS 侧也还没有音频话题。

4. **没有 HTTPS、没有用户体系**。局域网内明文 HTTP + 单一共享令牌。
   跨公网必须走反向代理（TLS + 账号）。

5. **相机的实际帧率是仿真决定的**。同一套代码在软件渲染下约 3 帧/秒（墙钟），
   真机 30 帧/秒。所有参数（`segment_sec`/`retain_sec`/`max_disk_mb`）都是
   按「10 秒一段」设的，真机上 10 秒 = 300 帧，配额要相应放大。

6. **事件合成是阻塞式重编码**。一个 20 秒窗口（约 300 帧）在容器里要几百毫秒，
   放在独立线程里做，不影响录像；但同一时刻来十个事件会排队。

7. **`/data` 没有做落盘同步**（`fsync`）。进程崩溃没事（分段已 rename），
   整机断电可能丢最后一个分段的尾部。真机应该用带超级电容的 SD 卡或定期 sync。

8. **`events/` 会无限增长**，除非打开保留策略。`lightdog_recorder` 每 60 秒
   （墙钟）跑一次 `_gc_events` / `_gc_snapshots`：

   | 参数 | 默认 | 含义 |
   |---|---|---|
   | `event_retain_days` | 30 | 超过这么多天的事件目录整个删掉 |
   | `max_event_mb` | 512 | `events/` 总量上限，从**最旧**的开始删 |
   | `min_events` | 20 | 至少留这么多条，最近的取证永远不会被容量策略删掉 |
   | `max_snapshots` | 200 | `snapshots/` 只保留最新这么多张 |

   两个保护：正在合成的目录（含 `video.part.mp4`）一律跳过，不会删到写了一半
   的事件；删除失败只记 warn，不会把整个节点带崩。真机上按存储卡容量调
   `max_event_mb`，否则半夜写满分区，告警录像也跟着一起失败。

---

## 八、与其它阶段的关系

- **P2（跟随）/ P3（回充）**：P4 只**读**它们的状态话题（`/person/present`、
  `/dock/state`、`/cmd_vel_mux_state`）用来在手机上显示，不改变它们的行为。
  录像在两种状态下都在跑 —— 看护设备没有「不录」的时刻。

- **P5（跌倒检测 + 告警链路）**：P4 给 P5 留了两个口子，P5 不需要知道录像的
  任何内部细节：

  1. 发一条消息就能拿到带「事发前」的视频：
     ```
     ros2 topic pub --once /recorder/event std_msgs/msg/String \
       '{data: "{\"type\":\"fall\",\"label\":\"检测到跌倒\",\"pre\":15,\"post\":10}"}'
     ```
  2. 读 `/data/contacts.json`（或 `GET /api/v1/contacts`）拿紧急联系人，
     事件视频的 URL 就是 `GET /api/v1/events/<id>/video`。

  换句话说，P5 要做的事是「判断 + 决定打给谁」，取画面和取证据这一步
  在 P4 已经闭环了。

## 八、移动端体验与传输安全（后补）

阶段验收之后又补了三处，都是「手机上真的用起来」才会暴露的问题：

| 补的东西 | 解决什么 | 怎么用 |
|---|---|---|
| `GET /api/v1/events/stream`（SSE） | 手机端原来 1Hz 轮询事件列表，锁屏回来要等一轮才刷新，且轮询在弱网下会堆积请求 | `new EventSource('/api/v1/events/stream?token=...')`。**SSE 的 `EventSource` 不能自定义 header**，所以 token 只能走 query —— 这是规范限制，不是设计选择 |
| 可选 HTTPS（`ssl_certfile` / `ssl_keyfile`） | WiFi 明文下同网段任何人都能看摄像头 | 配了证书就 `https://`。**证书读失败时服务直接退出（`return 2`）而不是退回明文** —— 静默降级比不加密更危险 |
| `POST /api/v1/estop` | 手机上要能一键刹车，且不能因为告警节点挂了而失效 | 直接发 `/safety/estop`，**不经过调度层**。语义是闭锁，见 [P6_system_layer.md](P6_system_layer.md) §五 |
| `faststart`（ffmpeg `-c copy -movflags +faststart`） | 录像的 `moov` 在文件尾，手机端边下边播要等整段下完 | 设为 `true` 即可，没装 ffmpeg 时**保留原文件**并记一条 warn，不影响录像本身 |

SSE 的几个实现细节：响应头带 `X-Accel-Buffering: no`（否则反代会把流缓冲住），
每条消息用 `data: {...}\n\n`，空闲时发 `: keep-alive` 注释行保活（有些代理
60s 无数据就断）。`sse_interval` 默认 1.0s。

> 上真机前请先过一遍 **[hardware_readiness.md](hardware_readiness.md)**（参数清单 / 失效矩阵 / 真机上不能跑的节点）。
