# SPDX-License-Identifier: MIT
CROSS ?= arm-none-eabi-
CC := $(CROSS)gcc
OBJCOPY := $(CROSS)objcopy
CFLAGS := -mcpu=cortex-m0plus -mthumb -Os -g -Wall -Wextra -Werror \
          -ffreestanding -fno-builtin -ffunction-sections -fdata-sections
LDFLAGS := -nostdlib -Wl,--gc-sections -Wl,-Map=build/cpu2.map -T cpu2.ld
SRC := startup.c main.c tl.c radio.c mac.c
OBJ := $(SRC:%.c=build/%.o)

all: build/stm32wb55-cpu2.bin
build/%.o: %.c $(wildcard *.h) | build
	$(CC) $(CFLAGS) -c $< -o $@
build/stm32wb55-cpu2.elf: $(OBJ) cpu2.ld
	$(CC) $(CFLAGS) $(OBJ) $(LDFLAGS) -lgcc -o $@
build/stm32wb55-cpu2.bin: build/stm32wb55-cpu2.elf
	$(OBJCOPY) -O binary $< $@
build:
	mkdir -p build
clean:
	rm -rf build
.PHONY: all clean
