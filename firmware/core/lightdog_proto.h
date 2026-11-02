/*
 * LightDog 下位机串口协议 —— 与 lightdog/lightdog_mcu_proto.py 逐字节一致。
 *
 * 帧：SYNC(0xAA 0x55) | LEN(u8) | TYPE(u8) | PAYLOAD(LEN) | CRC16(u16 小端)
 *     CRC16 = CCITT-FALSE（poly 0x1021 / init 0xFFFF / 不反射 / xorout 0），
 *     **只覆盖 LEN|TYPE|PAYLOAD**，不含 SYNC —— 固件可以边收边算。
 *
 * 这个文件不含任何 HAL / 寄存器操作：它是纯 C99，能在 PC 上编、在 PC 上测。
 * STM32 侧要做的只是「UART 收字节 -> ld_parser_push」「ld_encode -> UART 发」。
 * 把安全关键的那一段和具体芯片解耦，是唯一能在没有板子时验证它的办法。
 */
#ifndef LIGHTDOG_PROTO_H
#define LIGHTDOG_PROTO_H

#include <stddef.h>
#include <stdint.h>

#define LD_SYNC0        0xAAu
#define LD_SYNC1        0x55u
#define LD_MAX_PAYLOAD  32u
#define LD_FRAME_MAX    (6u + LD_MAX_PAYLOAD)

/* 下行 ROS -> STM32 */
#define LD_T_CMD_VEL      0x01u   /* f32 linear.x, f32 angular.z */
#define LD_T_LED          0x02u   /* f32 brightness 0..1 */
#define LD_T_HEARTBEAT    0x03u   /* 空 payload */
#define LD_T_ESTOP        0x04u   /* u8 0/1 */

/* 上行 STM32 -> ROS */
#define LD_T_BATTERY      0x81u   /* f32 percentage, f32 voltage, f32 current */
#define LD_T_ODOM         0x82u   /* f32 x, y, yaw, v, w */
#define LD_T_ESTOP_STATE  0x83u   /* u8 0/1 */
#define LD_T_FAULT        0x84u   /* u16 位掩码 */

/* 故障位掩码（可同时成立） */
#define LD_F_LINK_LOST     0x0001u
#define LD_F_OVERCURRENT   0x0002u
#define LD_F_UNDERVOLTAGE  0x0004u
#define LD_F_ENCODER       0x0008u
#define LD_F_ESTOP_BUTTON  0x0010u
#define LD_F_OVERTEMP      0x0020u

typedef struct {
    uint8_t type;
    uint8_t len;
    uint8_t payload[LD_MAX_PAYLOAD];
} ld_frame_t;

/* 256 字节 ≈ 6 帧最大帧长。串口一次 ISR 通常只给 1~64 字节。 */
#define LD_PARSER_BUF 256u

typedef struct {
    uint8_t  buf[LD_PARSER_BUF];
    uint16_t n;
    uint32_t frames;
    uint32_t crc_errors;
    uint32_t len_errors;
    uint32_t dropped;
} ld_parser_t;

uint16_t ld_crc16(const uint8_t *data, size_t len);

/*
 * 编码一帧到 out。返回写出的字节数；参数越界返回 0（**不要**返回一个截断的帧：
 * 半截的速度指令比没有更危险）。
 */
size_t ld_encode(uint8_t type, const uint8_t *payload, uint8_t plen,
                 uint8_t *out, size_t cap);

void ld_parser_init(ld_parser_t *p);

/*
 * 喂一段收到的字节，把解析出的帧写进 out（最多 max_out 个），返回帧数。
 * 没凑齐的半个帧会留在内部缓冲里等后续字节。
 */
int ld_parser_push(ld_parser_t *p, const uint8_t *data, size_t len,
                   ld_frame_t *out, int max_out);

#endif /* LIGHTDOG_PROTO_H */
