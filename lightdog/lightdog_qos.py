"""跨节点共享的 QoS 约定。

这个文件存在的理由只有一个：**急停是「状态」，不是「事件」**。

普通的传感器话题丢一条就丢一条，下一帧就补上了。急停不一样 —— 它是闭锁的
布尔量，丢一条 `true` 意味着机器人继续走。所以急停必须满足两条：

1. **可靠传输**（RELIABLE）：不允许悄悄丢。
2. **有记忆**（TRANSIENT_LOCAL）：订阅者晚到也能拿到「当前是不是急停」。

第 2 条不是锦上添花。实测过一次：运营者按下急停，`lightdog_mission` 因为
别的原因重启（改参数、进程崩了被拉起），它从 `estop=False` 开始初始化 ——
内存里那个「已经急停」的事实跟着旧进程一起没了。车还有 `twist_mux` 挡着，
但调度层会以为自己可以跟随，于是放开 `/follow/enable`、去抢 Nav2 的目标。
一旦哪天 mux 也重启，这个洞就真的漏出来了。

改成 TRANSIENT_LOCAL(depth=1) 之后，「最后一次急停状态」被发布端保留，
后启动的订阅者握手时直接收到最新的那一条 —— 重启不再等于解除急停。

## 兼容性

QoS 匹配是「发布者能力 >= 订阅者要求」：

- 用 `SAFETY_QOS` 发布、用默认（VOLATILE）订阅 → **能收到**，只是拿不到历史。
  所以外部脚本、临时调试节点不改也能用。
- 两边都用 `SAFETY_QOS` → 后启动的一方立刻拿到当前闭锁状态（我们要的）。

## 谁必须用

    发布 `/safety/estop`   lightdog_voice（语音「急停」） / lightdog_server（POST /api/v1/estop）
    订阅 `/safety/estop`   lightdog_twist_mux（执闸） / lightdog_mission（让路）
    将来 MCU 急停按钮桥   也必须用 SAFETY_QOS 发布，见 docs/firmware_protocol.md
"""

from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

# depth=1 而不是 10：只要「最后那一条」，历史里堆十条 true 没有任何意义，
# 反而会在重连时把早已解除的急停当成当前状态。
SAFETY_QOS = QoSProfile(
    depth=1,
    history=QoSHistoryPolicy.KEEP_LAST,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)
