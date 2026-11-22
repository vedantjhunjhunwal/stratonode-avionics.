#!/bin/sh
# Cross-compile StratoNode firmware for the STM32G474 (Cortex-M4F, hard float).
set -e
cd "$(dirname "$0")/.."
mkdir -p build/m4
FLAGS="-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -Os -std=gnu11 -Wall -Wextra -Werror \
 -ffunction-sections -fdata-sections -Iinclude"
arm-none-eabi-gcc $FLAGS src/drivers.c src/node.c src/app.c target/main.c target/startup.c \
  -T target/stm32g474ce.ld -Wl,--no-warn-rwx-segments -nostartfiles --specs=nano.specs --specs=nosys.specs -Wl,--gc-sections \
  -Wl,-Map=build/m4/stratonode.map -lm -o build/m4/stratonode.elf
arm-none-eabi-size build/m4/stratonode.elf
if arm-none-eabi-nm build/m4/stratonode.elf | grep -qE " T (malloc|_malloc_r|free)$"; then
  echo "ERROR: heap allocator linked in"; exit 1
fi
echo "no heap: malloc/free not linked"
arm-none-eabi-objcopy -O binary build/m4/stratonode.elf build/m4/stratonode.bin
