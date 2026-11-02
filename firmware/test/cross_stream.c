/*
 * 交叉验证：把一段**固定的随机字节流**喂给固件侧解析器，逐帧打印。
 *
 * 为什么要它：`test_proto.c` 覆盖的是「每种边界单独构造一次」，而这支程序
 * 覆盖的是「噪声、截断、CRC 错、粘连按随机顺序混在一起」。真实串口上就是
 * 这个样子，而两侧实现（C / Python）一旦在这里分叉，症状是「偶尔丢一条速度
 * 指令」—— 和「电机抽一下」一样难查。
 *
 * 分块大小必须与 lightdog/tests/test_mcu_stream.py 里的完全一致，否则两侧
 * 看到的「半帧」时机不同，比对就没有意义。公式：blk = (i*37+13) % 17 + 1。
 *
 * 用法：cross_stream <stream.bin>   （输出格式：<TYPE 两位 hex> <PAYLOAD hex>）
 */
#include <stdio.h>
#include <stdlib.h>

#include "lightdog_proto.h"

int main(int argc, char **argv)
{
    FILE *f;
    static uint8_t data[1 << 20];
    size_t n;
    size_t i = 0;
    ld_parser_t p;
    ld_frame_t out[8];

    if (argc < 2) {
        fprintf(stderr, "用法：%s <stream.bin>\n", argv[0]);
        return 2;
    }
    f = fopen(argv[1], "rb");
    if (f == NULL) {
        fprintf(stderr, "打不开 %s\n", argv[1]);
        return 2;
    }
    n = fread(data, 1, sizeof(data), f);
    fclose(f);

    ld_parser_init(&p);
    while (i < n) {
        size_t blk = (size_t)((i * 37u + 13u) % 17u + 1u);
        size_t take = (i + blk > n) ? (n - i) : blk;
        int c;
        int k;
        int j;

        c = ld_parser_push(&p, &data[i], take, out, 8);
        for (k = 0; k < c; ++k) {
            printf("%02x ", out[k].type);
            for (j = 0; j < (int)out[k].len; ++j) {
                printf("%02x", out[k].payload[j]);
            }
            printf("\n");
        }
        i += blk;                    /* 与 Python 侧一致：按未裁剪的步长前进 */
    }

    fprintf(stderr, "frames=%lu crc_errors=%lu len_errors=%lu dropped=%lu buffered=%u\n",
            (unsigned long)p.frames, (unsigned long)p.crc_errors,
            (unsigned long)p.len_errors, (unsigned long)p.dropped,
            (unsigned)p.n);
    return 0;
}
