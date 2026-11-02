# LightDog 固件参考实现的自检（在 PC 上跑，不需要 STM32）
#
#   make test          # 编译 + 跑黄金向量 + 看门狗时序
#   make cross         # 编译交叉验证程序（与 Python 侧逐帧比对，见测试）
#   make clean
#
# 交叉编译到 STM32 时不要用这个 Makefile：真正的固件由 CubeMX/CMake 管，
# 这里只需要保证 `core/` 两个 .c 是纯 C99、不引 HAL 头文件。
CC      ?= cc
CFLAGS  ?= -std=c99 -Wall -Wextra -Wpedantic -Werror -O2 -Icore
BUILD   := build

.PHONY: all test cross clean

all: $(BUILD)/test_proto

$(BUILD)/test_proto: core/lightdog_proto.c core/lightdog_watchdog.c test/test_proto.c
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) $^ -o $@

test: $(BUILD)/test_proto
	./$(BUILD)/test_proto test/vectors.txt

# 交叉验证：读一段二进制流，逐帧打印。参数与期望输出由
# lightdog/tests/test_mcu_proto.py 的 TestCrossImplementation 生成，
# 手工复跑：./build/cross_stream /tmp/stream.bin
$(BUILD)/cross_stream: core/lightdog_proto.c test/cross_stream.c
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) $^ -o $@

cross: $(BUILD)/cross_stream

clean:
	rm -rf $(BUILD)
