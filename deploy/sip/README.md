# 自建 SIP（Asterisk）—— 外呼通道部署

跌倒告警三级确认通过后，`lightdog_alert_manager` 要真的打电话。选型已定（D2）：
**自建 SIP**，Asterisk 装在本机或家里一台常开的机器上，家属的手机/软电话作为分机。

```
alert_manager ──AMI(TCP 5038, 明文, 只绑本机)──> Asterisk ──SIP/UDP 5060──> 家属手机
      │                                            │
      └─ OriginateResponse: 接通 / 无人接听 / 忙 / 拥塞        └─ Playback(lightdog/alarm)
```

## 为什么是 AMI 而不是 `asterisk -rx "channel originate ..."`

| 方面 | `-rx` 命令 | AMI |
|---|---|---|
| 知道「接没接」 | 退出码 0/1，只能猜 | `OriginateResponse` 明确给出 Reason |
| 是否阻塞调用方 | 每次新起进程，`subprocess.run` 卡住调用方 | 一条长连接，事件驱动 |
| 注入面 | 号码/姓名要拼进命令行 | 走 TCP 字段，不经 shell |
| 挂断正在响的那一通 | 得再起一个进程按频道名找 | `CoreShowChannels` + `Hangup` |

对应代码：`lightdog/lightdog_call.py`（不依赖 rclpy，宿主机可直接跑）。

## 部署（树莓派 5 本机）

```bash
sudo apt install asterisk ffmpeg
sudo cp deploy/sip/manager.conf.example /etc/asterisk/manager.conf
sudo cp deploy/sip/pjsip.conf.example   /etc/asterisk/pjsip.conf
sudo cp deploy/sip/extensions.conf.example /etc/asterisk/extensions.conf

# 1) 填三个密码
sudo sed -i "s/secret = 在这里填.*/secret = $(openssl rand -hex 24)/" /etc/asterisk/manager.conf
sudo sed -i "s/password = 换成随机串/password = $(openssl rand -hex 8)/" /etc/asterisk/pjsip.conf
sudo grep -A1 '^\[lightdog\]' /etc/asterisk/manager.conf    # 抄下这个 secret

# 2) 生成告警语音（8kHz 单声道，Piper 中文）
sudo ./deploy/sip/make_prompt.sh

# 3) 起服务
sudo systemctl enable --now asterisk
sudo asterisk -rx "manager reload"
sudo asterisk -rx "pjsip reload"
sudo asterisk -rx "dialplan reload"
```

```bash
# 4) 把 secret 写进参数文件（0600，别写进 unit 的 -p，systemctl status 会漏出去）
# 路径用 ros2 pkg prefix 定位，不要硬编码：参数文件在 <ws>/install/lightdog/
# share/lightdog/config/ 下（和 install.sh 的 PARAM_DIR 是同一个目录），
# 而 lightdog 从来不会装进 /opt/ros/<distro> 那个系统前缀里。
PARAMS="$(ros2 pkg prefix lightdog)/share/lightdog/config/p5_params.yaml"
sudo sed -i "s/^    sip_ami_secret: ''/    sip_ami_secret: '<上一步那个 secret>'/" "$PARAMS"
sudo chmod 600 "$PARAMS"

# sed 匹配不到行时**返回 0 且什么都不做**，所以必须回读确认。
# 没写进去的表现是：告警照常弹窗，一个电话都不打。
grep -E "^    sip_ami_secret: '.+'" "$PARAMS" \
  || echo "!! secret 没写进去：确认 p5_params.yaml 里这行是 4 个空格缩进、且值为空串"
```

> `call_mode` 默认是 `auto`：`sip_ami_secret` 非空就自动走 SIP，空就退化成
> 「只记录」。所以**不需要**再改 `call_mode`，也不需要在仿真/真机之间维护两份
> 参数文件。

## 验证（按顺序，每步都能独立定位故障）

```bash
# ① AMI 能登录（不拨号）
python3 lightdog/lightdog_call.py --check --secret "$SECRET"

# ② 分机注册上了吗
sudo asterisk -rx "pjsip show endpoints"     # 1001/1002 的 Contacts 应该是 Avail

# ③ 语音提示能播吗（手动打一通，会真的响铃）
sudo asterisk -rx "channel originate PJSIP/1001 extension s@alarm"

# ④ 走一遍完整链路（1 秒内出结果，不写任何 ROS 状态）
python3 lightdog/lightdog_call.py --dial 1001 --ring 45 --max-call 20 --secret "$SECRET"
```

第 ④ 步的输出就是 `call_attempts` 里会写的东西：

```json
{"outcome": "no_answer", "outcome_text": "无人接听", "reason": 3, ...}
{"outcome": "answered", "outcome_text": "已接通 12s", "reason": 0,
 "channel": "PJSIP/1001-0000000a", "hung_up_by": "ack"}
```

## 结果语义（这张表决定了产品的行为）

| outcome | Asterisk Reason | 含义 | 系统随后做什么 |
|---|---|---|---|
| `answered` | 0 / 4 | 有人接了 | 播放告警语音，最多占线 `sip_max_call`（120s） |
| `no_answer` | 3 | 响铃超时没人接 | 间隔 `call_interval` 后打下一个联系人 |
| `busy` | 5 | 对方在通话 | 同上 |
| `congestion` | 8 | 分机/中继不可达 | 同上，日志里会多一行 |
| `hangup` | 1 | 未接就挂了 | 同上 |
| `failed` | — | AMI 报错 / 密码错 / 连不上 | **不**计入「已经打出去」，告警记录里 `dialed=false` |
| `timeout` | — | 等 `OriginateResponse` 超时 | 同上 |
| `cancelled` | 1 + 本地 abort | ack / 复位打断了这一通 | 记录 `hung_up_by`，停止响铃 |

「没人接」**不是**失败。把两者混成一个 `failed`，事后翻记录时分不清
「那晚家里没人接电话」和「那晚 SIP 挂了」—— 这两种结论对应完全不同的整改。

## 安全

- AMI 是**明文**的。`bindaddr = 127.0.0.1` + `permit = 127.0.0.1/255.255.255.0`
  是硬要求，不是建议。Asterisk 挪到家里另一台机器时，只放行机器人那一个 IP。
- SIP 不要暴露到公网。5060/UDP 被扫描到只是几小时的事，之后就是盗打账单。
  要用手机流量接听就走 WireGuard/Tailscale，别直接端口映射。
- 分机密码、AMI secret 都是「能花钱」的凭据：不进 git，不写进 unit 的 `-p`
  （`systemctl status` 与 journal 会打印完整命令行）。
- 告警外呼**不会**因为 `is_test` 而响铃：`test` 命令在 ALARM 之外也会被
  `_maybe_call` 的第一行拦掉。

## 还没做的

1. **双向语音**。目前接听方只能听预录语音，说不出话。真要做到「家属能对老人
   喊话」，需要 Asterisk 18+ 的 AudioSocket/ExternalMedia 把通话音频流到本机，
   再由 TTS/播放器桥接 —— 这是下一步，不是当前链路的隐含行为。
2. **归属地/号码格式化**。`sip_channel_template` 里 `{number}` 原样代入，
   国际格式（+86…）与分机号（1001）走的是同一个模板。要打 PSTN 时按中继
   要求在 dialplan 里加前缀，别在联系人里存两套号码。
3. **通话录音**。`extensions.conf` 里加一行 `Record()` 就有了，但那会把
   家属的声音写进 SD 卡，涉及隐私与保留策略，先不做。
