/*
 * LightDog 协议 + 看门狗自检（在 PC 上跑，不需要 STM32）。
 *
 *     make -C firmware test
 *
 * 分两部分：
 *   1. 黄金向量 vectors.txt —— 与 Python 侧共读同一个文件，逐字节对齐。
 *   2. 看门狗时序 —— 300ms 边界、恢复帧数、急停优先级、计数器回绕。
 *      这一组是**安全链**的判据，必须比文档更早被验证。
 */
#include <stdio.h>
#include <string.h>

#include "lightdog_proto.h"
#include "lightdog_watchdog.h"

static int g_checks = 0;
static int g_failed = 0;

#define CHECK(cond, ...)                                                    \
    do {                                                                    \
        g_checks++;                                                         \
        if (!(cond)) {                                                      \
            g_failed++;                                                     \
            printf("FAIL %s:%d  ", __FILE__, __LINE__);                      \
            printf(__VA_ARGS__);                                            \
            printf("\n");                                                   \
        }                                                                   \
    } while (0)

static int hex2bin(const char *hex, uint8_t *out, size_t cap)
{
    size_t n = 0;
    int hi, lo;

    if (hex == NULL || *hex == '\0' || strcmp(hex, "-") == 0) {
        return 0;
    }
    while (hex[0] != '\0' && hex[1] != '\0') {
        char a = hex[0];
        char b = hex[1];
        hi = (a >= '0' && a <= '9') ? a - '0'
             : (a >= 'a' && a <= 'f') ? a - 'a' + 10
             : (a >= 'A' && a <= 'F') ? a - 'A' + 10 : -1;
        lo = (b >= '0' && b <= '9') ? b - '0'
             : (b >= 'a' && b <= 'f') ? b - 'a' + 10
             : (b >= 'A' && b <= 'F') ? b - 'A' + 10 : -1;
        if (hi < 0 || lo < 0) {
            return -1;
        }
        if (n >= cap) {
            return -1;
        }
        out[n++] = (uint8_t)((hi << 4) | lo);
        hex += 2;
    }
    return (int)n;
}

static int frame_eq(const uint8_t *a, size_t na, const uint8_t *b, size_t nb)
{
    return na == nb && memcmp(a, b, na) == 0;
}

/* ------------------------------------------------------------ 向量 */

static void test_vectors(const char *path)
{
    FILE *f = fopen(path, "r");
    char line[1024];
    int rows = 0;

    if (f == NULL) {
        g_checks++;
        g_failed++;
        printf("FAIL 打不开向量文件 %s\n", path);
        return;
    }
    while (fgets(line, (int)sizeof(line), f) != NULL) {
        char name[160], type_hex[16], pay_hex[160], frame_hex[320];
        uint8_t type = 0;
        uint8_t payload[LD_MAX_PAYLOAD];
        uint8_t want[LD_FRAME_MAX];
        uint8_t got[LD_FRAME_MAX];
        ld_frame_t frames[4];
        ld_parser_t parser;
        size_t len = strlen(line);
        int n_type;
        int n_pay;
        int n_want;
        int n_got;
        int nf;

        while (len > 0 && (line[len - 1] == '\n' || line[len - 1] == '\r')) {
            line[--len] = '\0';
        }
        if (len == 0 || line[0] == '#') {
            continue;
        }
        name[0] = '\0';
        if (sscanf(line, "%159[^|]|%15[^|]|%159[^|]|%319s",
                   name, type_hex, pay_hex, frame_hex) != 4) {
            CHECK(0, "向量行格式不对：%s", line);
            continue;
        }
        n_type = hex2bin(type_hex, &type, 1);
        n_pay = hex2bin(pay_hex, payload, sizeof(payload));
        n_want = hex2bin(frame_hex, want, sizeof(want));
        CHECK(n_type == 1 && n_pay >= 0 && n_want > 0,
              "[%s] 向量里有非法 hex", name);
        if (n_type != 1 || n_pay < 0 || n_want <= 0) {
            continue;
        }

        /* 编码结果必须逐字节等于向量 */
        n_got = (int)ld_encode(type, payload, (uint8_t)n_pay, got, sizeof(got));
        CHECK(frame_eq(got, (size_t)n_got, want, (size_t)n_want),
              "[%s] 编码不一致：c=%d 字节，向量=%d 字节", name, n_got, n_want);

        /* 喂回解析器必须还原出同样的 type/payload */
        ld_parser_init(&parser);
        nf = ld_parser_push(&parser, want, (size_t)n_want, frames, 4);
        CHECK(nf == 1, "[%s] 应解析出 1 帧，实际 %d", name, nf);
        if (nf == 1) {
            CHECK(frames[0].type == type, "[%s] type 0x%02x != 0x%02x",
                  name, frames[0].type, type);
            CHECK(frames[0].len == (uint8_t)n_pay &&
                  memcmp(frames[0].payload, payload, (size_t)n_pay) == 0,
                  "[%s] payload 不一致", name);
        }
        rows++;
    }
    fclose(f);
    CHECK(rows >= 8, "向量文件只有 %d 行，覆盖不足", rows);
    printf("  向量：%d 行（%s）\n", rows, path);
}

/* ---------------------------------------------------------- 解析边界 */

static void test_parser_edges(void)
{
    uint8_t f1[LD_FRAME_MAX];
    uint8_t f2[LD_FRAME_MAX];
    uint8_t both[2 * LD_FRAME_MAX];
    size_t n1, n2;
    ld_parser_t p;
    ld_frame_t out[4];
    int nf;
    size_t i;

    n1 = ld_encode(LD_T_CMD_VEL, (const uint8_t *)"\x00\x00\x80\x3e\x00\x00\x00\x00",
                   8, f1, sizeof(f1));
    n2 = ld_encode(LD_T_HEARTBEAT, NULL, 0, f2, sizeof(f2));
    CHECK(n1 == 14 && n2 == 6, "帧长算错：cmd_vel=%zu heartbeat=%zu", n1, n2);

    /* 1) 两帧粘连 */
    memcpy(both, f1, n1);
    memcpy(both + n1, f2, n2);
    ld_parser_init(&p);
    nf = ld_parser_push(&p, both, n1 + n2, out, 4);
    CHECK(nf == 2, "粘连两帧只解析出 %d 帧", nf);
    CHECK(nf == 2 && out[0].type == LD_T_CMD_VEL && out[1].type == LD_T_HEARTBEAT,
          "粘连两帧的类型/顺序不对");

    /* 2) 逐字节喂（UART ISR 一次给 1 个字节是最坏情况） */
    ld_parser_init(&p);
    nf = 0;
    for (i = 0; i < n1; ++i) {
        nf += ld_parser_push(&p, &f1[i], 1, &out[nf], 4 - nf);
    }
    CHECK(nf == 1 && out[0].type == LD_T_CMD_VEL, "逐字节喂只解析出 %d 帧", nf);
    CHECK(p.dropped == 0, "逐字节喂丢了 %u 字节", (unsigned)p.dropped);

    /* 3) 前面有噪声 */
    ld_parser_init(&p);
    nf = ld_parser_push(&p, (const uint8_t *)"\x01\x02\x03\x04", 4, out, 4);
    CHECK(nf == 0 && p.n == 0, "纯噪声不该产生帧");
    nf = ld_parser_push(&p, f1, n1, out, 4);
    CHECK(nf == 1, "噪声之后的帧没恢复：%d", nf);
    CHECK(p.dropped == 4, "噪声应计 4 字节，实际 %u", (unsigned)p.dropped);

    /* 4) CRC 错：丢弃该帧，但后面那帧必须还能解出来 */
    ld_parser_init(&p);
    both[0] = f1[0];
    n1 = ld_encode(LD_T_CMD_VEL, (const uint8_t *)"\x00\x00\x80\x3e\x00\x00\x00\x00",
                   8, f1, sizeof(f1));
    both[0] = f1[0];
    both[1] = f1[1];
    both[2] = f1[2];
    both[3] = f1[3];
    memcpy(&both[4], f1 + 4, n1 - 4);
    both[n1 - 1] ^= 0xFFu;                     /* 破坏 CRC */
    memcpy(both + n1, f2, n2);
    nf = ld_parser_push(&p, both, n1 + n2, out, 4);
    CHECK(p.crc_errors == 1, "CRC 错应计 1 次，实际 %u", (unsigned)p.crc_errors);
    CHECK(nf == 1 && out[0].type == LD_T_HEARTBEAT,
          "CRC 错之后的帧没恢复：%d 帧", nf);

    /* 5) LEN 超上限：说明那个 0xAA55 是数据里碰巧的 */
    ld_parser_init(&p);
    {
        uint8_t bad[] = {LD_SYNC0, LD_SYNC1, 0xFF, LD_T_LED, 0x00};
        nf = ld_parser_push(&p, bad, sizeof(bad), out, 4);
        CHECK(nf == 0 && p.len_errors == 1,
              "LEN=0xFF 应计 len_error，实际 nf=%d len_errors=%u",
              nf, (unsigned)p.len_errors);
    }
    nf = ld_parser_push(&p, f2, n2, out, 4);
    CHECK(nf == 1, "LEN 错之后没恢复：%d", nf);

    /* 6) 编码参数越界必须拒绝，而不是写半帧 */
    CHECK(ld_encode(LD_T_LED, (const uint8_t *)"0123456789", 40, both, sizeof(both)) == 0,
          "payload 超过上限时应当返回 0");
    CHECK(ld_encode(LD_T_LED, NULL, 4, both, sizeof(both)) == 0,
          "payload 为 NULL 但长度为 4 时应当返回 0");

    /* 7) 缓冲区塞满：不崩、计数增加，下一条真帧仍能解出来 */
    ld_parser_init(&p);
    {
        uint8_t junk[LD_PARSER_BUF + 8];
        memset(junk, 0x00, sizeof(junk));
        nf = ld_parser_push(&p, junk, sizeof(junk), out, 4);
        CHECK(nf == 0 && p.dropped > 0, "超长噪声应被丢弃并计数");
    }
    nf = ld_parser_push(&p, f2, n2, out, 4);
    CHECK(nf == 1, "缓冲清空之后没恢复：%d", nf);
}

/* ---------------------------------------------------------- 看门狗 */

static void feed(ld_wd_t *w, uint32_t *t, uint32_t step, int times)
{
    int i;
    for (i = 0; i < times; ++i) {
        *t += step;
        ld_wd_heartbeat(w, *t);
    }
}

static void test_watchdog(void)
{
    ld_wd_t w;
    uint32_t t = 1000u;

    /* 1) 开机没有心跳：不许使能 */
    ld_wd_init(&w, t);
    ld_wd_tick(&w, t + 10u);
    CHECK(w.enabled == 0, "没有心跳就使能了");
    CHECK(ld_wd_output_enabled(&w) == 0, "没有心跳就允许输出 PWM");

    /* 2) 3 帧心跳之后才使能，且第一拍要求零速 */
    feed(&w, &t, 50u, 3);
    ld_wd_tick(&w, t);
    CHECK(w.enabled == 1, "3 帧心跳之后没使能");
    CHECK(w.hold_zero == 1, "恢复使能时没有要求先出零速");
    CHECK(ld_wd_output_enabled(&w) == 0, "hold_zero 期间不该允许 PWM");
    ld_wd_zero_applied(&w);
    CHECK(ld_wd_output_enabled(&w) == 1, "出过零速之后应当允许 PWM");

    /* 3) 只收到 2 帧心跳：仍不许使能（单帧抖动不能把车救活） */
    ld_wd_init(&w, t);
    feed(&w, &t, 50u, 2);
    ld_wd_tick(&w, t);
    CHECK(w.enabled == 0, "2 帧心跳就使能了");

    /* 4) 300ms 边界：差 1ms 就失联，且是严格小于 */
    ld_wd_init(&w, t);
    feed(&w, &t, 50u, 3);
    ld_wd_tick(&w, t);
    CHECK(w.enabled == 1, "边界用例前置条件失败");
    ld_wd_tick(&w, t + 299u);
    CHECK(w.enabled == 1, "299ms 就判失联了（太激进）");
    ld_wd_tick(&w, t + 300u);
    CHECK(w.enabled == 0, "300ms 整没有判失联");
    CHECK((w.fault & LD_F_LINK_LOST) != 0, "失联没置故障位");
    /* 失联后再来一帧不够，必须重新攒 3 帧 */
    t = t + 300u;
    ld_wd_heartbeat(&w, t);
    ld_wd_tick(&w, t);
    CHECK(w.enabled == 0, "失联后一帧心跳就恢复了");
    feed(&w, &t, 10u, 2);
    ld_wd_tick(&w, t);
    CHECK(w.enabled == 1, "失联后 3 帧心跳仍未恢复");
    CHECK((w.fault & LD_F_LINK_LOST) == 0, "恢复后故障位没清");

    /* 5) 急停优先级：心跳新鲜、但急停闭锁 -> 立即失能 */
    ld_wd_set_estop(&w, 1, 1);
    CHECK(w.enabled == 0, "急停之后 enabled 仍为 1（要等 tick 才生效？）");
    CHECK((w.fault & LD_F_ESTOP_BUTTON) != 0, "物理急停没置故障位");
    feed(&w, &t, 10u, 5);
    ld_wd_tick(&w, t);
    CHECK(w.enabled == 0, "急停期间被心跳「救活」了");
    CHECK(ld_wd_output_enabled(&w) == 0, "急停期间允许输出 PWM");

    /* 6) 解除急停后同样要重新攒 3 帧 */
    ld_wd_set_estop(&w, 0, 1);
    ld_wd_tick(&w, t);
    CHECK(w.enabled == 0, "解除急停的瞬间就使能了");
    feed(&w, &t, 10u, 3);
    ld_wd_tick(&w, t);
    CHECK(w.enabled == 1, "解除急停后 3 帧仍未使能");
    CHECK((w.fault & LD_F_ESTOP_BUTTON) == 0, "按钮故障位没清");

    /* 7) 计数器回绕：49 天后的 ms 计数会绕过 0，不能把「刚收到」判成「失联 49 天」 */
    ld_wd_init(&w, 0xFFFFFFF0u);
    feed(&w, &t, 0u, 0);
    ld_wd_heartbeat(&w, 0xFFFFFFF0u);
    ld_wd_heartbeat(&w, 0xFFFFFFFBu);
    ld_wd_heartbeat(&w, 0x00000006u);        /* 回绕之后 */
    ld_wd_tick(&w, 0x00000050u);             /* 距最后一帧 74ms */
    CHECK(w.enabled == 1, "计数器回绕被误判成失联");
    CHECK((w.fault & LD_F_LINK_LOST) == 0, "回绕后故障位没清");
    ld_wd_tick(&w, 0x00000100u);             /* 距最后一帧 250ms */
    CHECK(w.enabled == 1, "回绕后 250ms 就判失联");
    ld_wd_tick(&w, 0x0000012Fu);             /* 297ms */
    CHECK(w.enabled == 1, "回绕后 297ms 就判失联");
    ld_wd_tick(&w, 0x00000130u);             /* 298ms -> 仍在 300 内 */
    CHECK(w.enabled == 1, "回绕后 298ms 就判失联");
}

int main(int argc, char **argv)
{
    const char *vectors = (argc > 1) ? argv[1] : "test/vectors.txt";

    CHECK(ld_crc16((const uint8_t *)"123456789", 9) == 0x29B1u,
          "CRC16/CCITT-FALSE 校验值应为 0x29B1，实际 0x%04x",
          ld_crc16((const uint8_t *)"123456789", 9));

    test_vectors(vectors);
    test_parser_edges();
    test_watchdog();

    printf("%d 项检查，%d 项失败\n", g_checks, g_failed);
    return g_failed == 0 ? 0 : 1;
}
