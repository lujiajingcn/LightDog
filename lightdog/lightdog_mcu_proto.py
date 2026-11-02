#!/usr/bin/env python3
"""LightDog 下位机串口协议（ROS 侧编解码）。

选型已定（D3）：STM32。协议因此从「语义 + 时序」变成**定稿的字节布局**：

    帧 = SYNC(0xAA 0x55) | LEN(u8) | TYPE(u8) | PAYLOAD(LEN) | CRC16(u16, 小端)

CRC16 是 CCITT-FALSE（poly 0x1021 / init 0xFFFF / 不反射 / xorout 0），
**只覆盖 LEN|TYPE|PAYLOAD**，不含 SYNC 两个字节。这样固件可以「收到 SYNC 之后
才开始喂 CRC」——增量算，不需要先缓存整帧。

## 为什么不用 micro-ROS

micro-ROS 在 F4 上能跑，但它把「上位机与固件之间」变成了一条 XRCE-DDS 链路：
出问题时要同时怀疑 ROS、DDS、agent、串口四层。而这条链路上跑的东西只有
「两个 float 的速度」和「一个 float 的亮度」，用 40 字节的定长帧就够了。
**安全关键的那一段越简单越好** —— 这一段是唯一能在 `kill -9` 之后刹住车的东西。

代价是要自己写桥接节点（`lightdog_mcu_bridge.py`），但那一层在宿主机上可测，
而 XRCE 的故障现场往往只有「连不上」三个字。

## 帧里为什么没有序号

- `cmd_vel` 丢一帧无所谓：下一条会覆盖它。
- 心跳丢一帧无所谓：300ms = 连丢 3 帧才动作。
- 加序号就要在固件里维护「收到过哪些」，而它能额外提供的信息（乱序/重复）
  在 UART 这种有序链路上本来就不存在。

固件要自定义类型时**必须**同时在 `docs/firmware_protocol.md` §六 的表里登记，
否则上位机会把它当未知帧丢掉（`unknown_types` 计数，日志里只有一行）。

## 兼容性

`decode_*` 严格按长度校验：长度不对返回 None 而不是猜。半截的浮点数比没有更危险。
"""

import struct

SYNC = b'\xaa\x55'
MAX_PAYLOAD = 32

# ---- 下行（ROS -> STM32）----
T_CMD_VEL = 0x01        # f32 linear.x, f32 angular.z
T_LED = 0x02            # f32 brightness 0..1
T_HEARTBEAT = 0x03      # 空 payload
T_ESTOP = 0x04          # u8 0/1

# ---- 上行（STM32 -> ROS）----
T_BATTERY = 0x81        # f32 percentage(0..1), f32 voltage, f32 current(A)
T_ODOM = 0x82           # f32 x, y, yaw, v, w
T_ESTOP_STATE = 0x83    # u8 0/1（物理按钮/固件闭锁）
T_FAULT = 0x84          # u16 位掩码

DOWN_TYPES = {T_CMD_VEL: 'cmd_vel', T_LED: 'led',
              T_HEARTBEAT: 'heartbeat', T_ESTOP: 'estop'}
UP_TYPES = {T_BATTERY: 'battery', T_ODOM: 'odom',
            T_ESTOP_STATE: 'estop_state', T_FAULT: 'fault'}

TYPE_NAMES = dict(DOWN_TYPES)
TYPE_NAMES.update(UP_TYPES)

#: 故障位掩码。多位可同时成立，所以是位而不是枚举。
F_LINK_LOST = 0x0001
F_OVERCURRENT = 0x0002
F_UNDERVOLTAGE = 0x0004
F_ENCODER = 0x0008
F_ESTOP_BUTTON = 0x0010
F_OVERTEMP = 0x0020

FAULT_NAMES = {
    F_LINK_LOST: 'link_lost',
    F_OVERCURRENT: 'overcurrent',
    F_UNDERVOLTAGE: 'undervoltage',
    F_ENCODER: 'encoder',
    F_ESTOP_BUTTON: 'estop_button',
    F_OVERTEMP: 'overtemp',
}


def fault_text(mask):
    """把位掩码翻成人能读的字符串；0 返回 'ok'（不要返回空串）。"""
    mask = int(mask or 0)
    if mask == 0:
        return 'ok'
    names = [name for bit, name in FAULT_NAMES.items() if mask & bit]
    rest = mask & ~sum(FAULT_NAMES)
    if rest:
        names.append(f'0x{rest:04x}')
    return '+'.join(names)


# ---------------------------------------------------------------- CRC

def crc16(data, crc=0xFFFF):
    """CRC-16/CCITT-FALSE。查表会更快，但这段每秒只跑几百次，
    位运算实现更容易在固件里逐字节对照，也少一张 512 字节的表。"""
    crc &= 0xFFFF
    for byte in bytes(data):
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


# ---------------------------------------------------------------- 编码

def encode(ftype, payload=b''):
    payload = bytes(payload)
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(f'payload {len(payload)} 字节，超过上限 {MAX_PAYLOAD}')
    body = bytes([len(payload), int(ftype)]) + payload
    return SYNC + body + crc16(body).to_bytes(2, 'little')


def encode_cmd_vel(linear_x, angular_z):
    return encode(T_CMD_VEL, struct.pack('<ff', linear_x, angular_z))


def encode_led(brightness):
    return encode(T_LED, struct.pack('<f', brightness))


def encode_heartbeat():
    return encode(T_HEARTBEAT)


def encode_estop(on):
    return encode(T_ESTOP, bytes([1 if on else 0]))


def encode_battery(percentage, voltage, current=0.0):
    return encode(T_BATTERY, struct.pack('<fff', percentage, voltage, current))


def encode_odom(x, y, yaw, v=0.0, w=0.0):
    return encode(T_ODOM, struct.pack('<fffff', x, y, yaw, v, w))


def encode_estop_state(on):
    return encode(T_ESTOP_STATE, bytes([1 if on else 0]))


def encode_fault(mask):
    return encode(T_FAULT, struct.pack('<H', int(mask) & 0xFFFF))


# ---------------------------------------------------------------- 解码

def decode_cmd_vel(payload):
    if len(payload) != 8:
        return None
    return struct.unpack('<ff', payload)


def decode_led(payload):
    if len(payload) != 4:
        return None
    return struct.unpack('<f', payload)[0]


def decode_estop(payload):
    if len(payload) != 1:
        return None
    return bool(payload[0])


def decode_battery(payload):
    if len(payload) != 12:
        return None
    return struct.unpack('<fff', payload)


def decode_odom(payload):
    if len(payload) != 20:
        return None
    return struct.unpack('<fffff', payload)


def decode_estop_state(payload):
    if len(payload) != 1:
        return None
    return bool(payload[0])


def decode_fault(payload):
    if len(payload) != 2:
        return None
    return struct.unpack('<H', payload)[0]


# ---------------------------------------------------------------- 解析

class Frame:
    """一个校验通过的帧。payload 是 bytes，不做拷贝之外的任何解释。"""

    __slots__ = ('type', 'payload')

    def __init__(self, ftype, payload):
        self.type = ftype
        self.payload = payload

    def __eq__(self, other):
        return (isinstance(other, Frame)
                and self.type == other.type and self.payload == other.payload)

    def __repr__(self):
        return f'Frame(0x{self.type:02x}/{TYPE_NAMES.get(self.type, "?")}, {self.payload.hex()})'


class Parser:
    """流式解析。串口没有「一次一帧」这回事：半帧、两帧粘连、噪声混在中间都要处理。

    三条策略：
      1. CRC 错**只丢 SYNC 两个字节**，不丢整帧 —— 后面可能还埋着一个真帧，
         尤其是被截断的那一帧。
      2. `LEN > MAX_PAYLOAD` 说明这个 0xAA55 是数据里碰巧出现的，丢 2 字节继续扫。
      3. 找不到下一个 SYNC 时清空缓冲；否则缓冲区会随着噪声无限增长，
         而内存耗尽的表现是「进程被 OOM 杀掉」，跟串口看起来毫无关系。
    """

    def __init__(self, max_buffer=4096):
        self.buf = bytearray()
        self.max_buffer = int(max_buffer)
        self.crc_errors = 0
        self.len_errors = 0
        self.dropped = 0
        self.frames = 0

    def feed(self, data):
        out = []
        self.buf += bytes(data)
        while True:
            i = self.buf.find(SYNC)
            if i < 0:
                # 全丢，但保留末尾那个可能是 0xAA 的字节：UART 一次只给 1 个
                # 字节时，0xAA 和 0x55 会被拆在两次 feed 里，全丢就永远对不上。
                keep = 1 if (self.buf and self.buf[-1] == SYNC[0]) else 0
                self.dropped += len(self.buf) - keep
                del self.buf[:len(self.buf) - keep]
                break
            if i > 0:
                self.dropped += i
                del self.buf[:i]
            if len(self.buf) < 3:
                break
            ln = self.buf[2]
            if ln > MAX_PAYLOAD:
                self.len_errors += 1
                del self.buf[:2]
                continue
            total = 6 + ln
            if len(self.buf) < total:
                break
            covered = bytes(self.buf[2:4 + ln])
            want = int.from_bytes(self.buf[4 + ln:6 + ln], 'little')
            if crc16(covered) != want:
                self.crc_errors += 1
                del self.buf[:2]
                continue
            out.append(Frame(self.buf[3], bytes(self.buf[4:4 + ln])))
            self.frames += 1
            del self.buf[:total]
        # 兜底：一直没有 SYNC 时清空，避免噪声把内存吃光
        if len(self.buf) > self.max_buffer:
            self.dropped += len(self.buf)
            del self.buf[:]
        return out

    def stats(self):
        return {'frames': self.frames, 'crc_errors': self.crc_errors,
                'len_errors': self.len_errors, 'dropped_bytes': self.dropped,
                'buffered': len(self.buf)}
