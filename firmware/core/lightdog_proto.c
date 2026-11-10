/*
 * LightDog 下位机串口协议实现（纯 C99，无 HAL）。
 *
 * 和 lightdog/lightdog_mcu_proto.py 是同一套字节规则的两份实现：
 * 黄金向量（test/vectors.txt）由两侧共同读取，任何一侧改错都会同时失败。
 * 这么做不是为了「双保险」—— 是因为固件跑不到 CI 上，只能靠共享向量对齐。
 */
#include "lightdog_proto.h"

#include <string.h>

uint16_t ld_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFFu;
    size_t i;
    int bit;

    for (i = 0; i < len; ++i) {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (bit = 0; bit < 8; ++bit) {
            if (crc & 0x8000u) {
                crc = (uint16_t)((uint16_t)(crc << 1) ^ 0x1021u);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}

size_t ld_encode(uint8_t type, const uint8_t *payload, uint8_t plen,
                 uint8_t *out, size_t cap)
{
    size_t total;
    uint16_t crc;

    if (plen > LD_MAX_PAYLOAD) {
        return 0;
    }
    if (payload == NULL && plen > 0) {
        return 0;
    }
    total = (size_t)6 + plen;
    if (out == NULL || cap < total) {
        return 0;
    }

    out[0] = LD_SYNC0;
    out[1] = LD_SYNC1;
    out[2] = plen;
    out[3] = type;
    if (plen > 0) {
        memcpy(&out[4], payload, plen);
    }
    crc = ld_crc16(&out[2], (size_t)2 + plen);   /* LEN|TYPE|PAYLOAD */
    out[4 + plen] = (uint8_t)(crc & 0xFFu);
    out[5 + plen] = (uint8_t)(crc >> 8);
    return total;
}

void ld_parser_init(ld_parser_t *p)
{
    if (p == NULL) {
        return;
    }
    memset(p, 0, sizeof(*p));
}

/*
 * 把缓冲里的字节挪走 n 个。协议最多 40 字节一帧，memmove 比环形缓冲简单得多，
 * 而在 STM32 上 40 字节的 memmove 只要几百纳秒 —— 不值得为它引入环形索引。
 */
static void drop_front(ld_parser_t *p, uint16_t n)
{
    if (n == 0) {
        return;
    }
    if (n >= p->n) {
        p->n = 0;
        return;
    }
    memmove(p->buf, &p->buf[n], (size_t)(p->n - n));
    p->n = (uint16_t)(p->n - n);
}

int ld_parser_push(ld_parser_t *p, const uint8_t *data, size_t len,
                   ld_frame_t *out, int max_out)
{
    int count = 0;
    size_t i;

    if (p == NULL || (data == NULL && len > 0u)) {
        return 0;
    }
    /* len==0 是合法的「把上次没交付完的帧继续吐出来」调用，
     * 见 ld_app_rx 的排空循环。 */
    if (len > (size_t)(LD_PARSER_BUF - p->n)) {
        /* 缓冲装不下：整段丢弃，等下一次 SYNC 重新对齐。
         * 宁可丢帧也不能覆盖还没解析的字节（那会把半帧变成错帧）。 */
        p->dropped += (uint32_t)p->n + (uint32_t)len;
        p->n = 0;
        return 0;
    }
    if (len > 0u) {
        memcpy(&p->buf[p->n], data, len);
        p->n = (uint16_t)(p->n + len);
    }

    for (;;) {
        uint16_t sync_at = 0xFFFFu;
        uint8_t plen;
        uint16_t total;
        uint16_t covered_len;
        uint16_t want;
        uint16_t got;

        for (i = 0; i + 1 < p->n; ++i) {
            if (p->buf[i] == LD_SYNC0 && p->buf[i + 1] == LD_SYNC1) {
                sync_at = (uint16_t)i;
                break;
            }
        }
        if (sync_at == 0xFFFFu) {
            /* 没有 SYNC，全丢；但保留最后一个可能是 0xAA 的字节，
             * 否则 0xAA 和 0x55 被拆在两次 push 里就永远对不上。 */
            uint16_t keep = (p->n > 0 && p->buf[p->n - 1] == LD_SYNC0) ? 1u : 0u;
            p->dropped += (uint32_t)(p->n - keep);
            drop_front(p, (uint16_t)(p->n - keep));
            break;
        }
        if (sync_at > 0) {
            p->dropped += sync_at;
            drop_front(p, sync_at);
        }
        if (p->n < 3) {
            break;                        /* 还读不到 LEN */
        }
        plen = p->buf[2];
        if (plen > LD_MAX_PAYLOAD) {
            /* LEN 离谱：这个 0xAA55 是数据里碰巧出现的，丢 2 字节继续扫 */
            p->len_errors++;
            drop_front(p, 2);
            continue;
        }
        total = (uint16_t)(6u + plen);
        if (p->n < total) {
            break;                        /* 半帧，等后续字节 */
        }
        covered_len = (uint16_t)(2u + plen);
        want = (uint16_t)p->buf[4u + plen] | (uint16_t)((uint16_t)p->buf[5u + plen] << 8);
        got = ld_crc16(&p->buf[2], covered_len);
        if (got != want) {
            /* 只丢 SYNC：被截断的那一帧后面常常紧跟一个完好的帧 */
            p->crc_errors++;
            drop_front(p, 2);
            continue;
        }
        if (out != NULL && count >= max_out) {
            /* 输出槽满了就把这一帧**留在缓冲里**，等下一次 push 再交付。
             * 原实现仍执行 p->frames++ / drop_front，帧被静默消费却没人
             * 处理：一次 DMA 空闲中断收到 5 个以上合法帧时，第 5 帧起
             * 全部消失，其中若是心跳就既没喂狗也没刷新时间戳。 */
            break;
        }
        if (out != NULL) {
            out[count].type = p->buf[3];
            out[count].len = plen;
            if (plen > 0) {
                memcpy(out[count].payload, &p->buf[4], plen);
            }
            count++;
        }
        p->frames++;
        drop_front(p, total);
    }
    return count;
}
