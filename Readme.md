**硬件：**
四轮小车，有麦克风，有摄像头，有led灯泡，有支持无线充电的电池，无线充电底座，有wifi模块，有存储卡，有扬声器。

**功能：**
可以灵活移动，通过麦克风能听到指令，通过摄像头能看见动作，可以调节led亮度，电量低时自动去无线充电底座充电，通过wifi模块联网，通过存储卡存储摄像头拍摄的视频，可以播放声音。

**备注：**
为了不影响摄像头，可以把led灯泡放在小车的四周。
车底部放置一个常亮的小功率led指示灯。

**使用场景1：**
夜间去卫生间：晚上趴在床边，底部常亮的指示灯可以显示位置（功率很小不会影响睡眠），当人下床时，根据声音和摄像头判断，亮起主led灯泡，跟随人活动，照亮去卫生间的路，不用开房间里的灯。

**使用场景2：**

看护老人：与手机互联，存储紧急联系人电话，如果检测到人倒地并长时间无法站立，拨打紧急联系人电话，并允许紧急联系人查看摄像头画面。

**使用场景2：**

监控：平时白天充当可移动监控。

---

# 当前实现状态

上面是需求原文，这一节是按需求逐条对照的**实际进度**。判断标准只有一条：
**在仿真里跑通过、有测试或验收脚本兜着**才算「完成」，写了一半的不算。

| 需求 | 状态 | 落在哪 | 还差什么（硬件相关） |
|---|---|---|---|
| 可以灵活移动 | ✅ 完成（仿真）/ 🟡 待固件 | `lightdog_twist_mux` 仲裁 + `/cmd_vel_muxed`；`lightdog_mcu_bridge` 串口桥接；协议定稿 + 两侧实现；**固件应用层已就绪**（`firmware/app/`，PC 侧自检覆盖失联/急停/按钮/里程计） | 底盘、编码器、电机驱动；按 `firmware/port/stm32f4/` 接线并把引脚/减速比填进去（`make -C firmware test` 可先验，判据是 0 失败） |
| 通过麦克风能听到指令 | 🟡 链路完成 | `lightdog_voice`：`/voice/command` 文本进 → 语义解析 → 调度层动作。13 类意图，含「别跟着我」「解除急停」这类高风险歧义 | 平台已定：**树莓派 5**（无 CUDA）→ ASR 走 whisper.cpp/sherpa-onnx、TTS 走 Piper，见 `docs/hardware_readiness.md` §8.5/8.6。真机只需把 ASR 输出灌进 `/voice/command`，本节点一行不用改 |
| 通过摄像头能看见动作 | ✅ 完成（仿真） | `lightdog_person_detector` + `lightdog_fall_detector`；后端可插拔（`color` / `yolo`） | 真机换 `backend: yolo`；相机高度 0.3745m 偏低，需抬高后重标定 |
| 可以调节 led 亮度 | 🟡 ROS 侧完成 | `/led/brightness` + 自动调光 + 手动覆盖（`/light/mode`）+ 语音控制；按时间积分的渐变 | **LED 固件**（订阅 `/led/brightness` 转 PWM） |
| 电量低时自动去充电 | ✅ 完成（仿真） | `lightdog_dock_manager`（ArUco 对准/重试/充电确认）+ `lightdog_mission` 的低电迟滞（35% 接近 / 45% 解除） | 10Ah 级电池 + 60~100W 磁共振无线充；`marker_length` 需卡尺实测 |
| 通过 wifi 模块联网 | ✅ 完成 | `lightdog_server`：HTTP API + 手机端页面 + **SSE 实时事件流** + 可选 HTTPS（证书失败时拒绝明文启动） | WiFi 模块本身；真机务必设 `auth_token` |
| 存储卡存视频 | ✅ 完成 | `lightdog_recorder`：事件前后各 N 秒合成；可选 `faststart`（手机端边下边播） | 存储卡容量决定 `event_retain_days` |
| 可以播放声音 | 🟡 链路完成 | `lightdog_voice`：`/alert/voice` → `/voice/say`；`tts_command` 非空时直接起 TTS 子进程（去重 1.5s、截断 200 字） | **扬声器 + TTS 引擎**（espeak-ng / 云 TTS 都行，换个命令模板即可） |
| 摔倒 → 打电话给紧急联系人 | ✅ 完成（仿真）/ 🟡 待现场验证 | 告警状态机 + 自建 SIP 外呼（`lightdog_call.py`，AMI）：能区分接通/无人接听/忙；`ack` 会挂断正在响的那一通 | 方案已定：**自建 SIP**（Asterisk）。部署与四步验收见 `deploy/sip/README.md`；协议层已用假 AMI 服务器测过（`tests/test_call.py`），真机需现场走一遍 |

图例：✅ 逻辑闭环且仿真验收 · 🟡 软件侧完成、等硬件或外部服务 · ❌ 未开工

**一句话结论**：软件侧已经没有「必须等硬件才能动笔」的空白了。剩下的工作集中在
三处 —— 算力平台（已定 Pi 5，见 `docs/hardware_readiness.md` §〇）、MCU 固件、
以及传感器/机构的机械标定。MCU 侧的**应用逻辑已经写完并有 PC 自检兜着**
（`firmware/app/`：帧分发 / 输出决策 / 里程计 / 上报，覆盖失联、急停、物理按钮、
恢复后第一拍零速），剩下的是按原理图填引脚与减速比 —— 那是 `port/stm32f4/`
里的一张接线表，不是重新设计。

硬件到位后是「接线 + 填参数 + 标定」，不是「从零开始写」。

## 怎么跑起来

```bash
# 仿真（Docker Desktop + WSL2，容器名 sim）
cd sim && docker compose --env-file .env up -d

# 系统层：调度 + 语音
bash sim/system.sh start
bash sim/system.sh status

# 让「人」出现，观察模式切到 FOLLOW
bash sim/p2.sh spawn
bash sim/p2.sh start

# 语音链路（模拟 ASR 输入）
bash sim/system.sh listen 把灯打开
bash sim/system.sh listen 不要再跟着我了

# 离线回归（宿主机，不需要 ROS）
python lightdog/tests/run.py
```

真机上由 systemd 托管（开机自启 + 崩溃重启 + 日志有界），不是手动敲上面那些脚本：

```bash
sudo ./deploy/install.sh --dry-run     # 先看会生成什么 unit
sudo ./deploy/install.sh
sudo ./deploy/verify.sh                # unit / 路径 / 进程 / 话题
sudo systemctl start lightdog.target
lightdog-estop                         # 急停状态（set / clear --yes）

# 连跑 2 小时的浸泡（泄漏 / 线程 / fd / 存储增长），跑完自动出报告
sudo python3 deploy/soak.py run --duration 7200 --out /tmp/soak.csv
```

见 `deploy/README.md`。

节点/话题/参数的细节见 `docs/` 下的阶段文档，硬件相关的坑见
`docs/hardware_readiness.md`，**要买什么、买几个、为什么是这个规格见 `docs/bom.md`**。