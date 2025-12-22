CPU := cortex-m4
DEVICE_FAMILY := STM32F4

STARTUP_FILE := ${CURDIR}/src/utils/startup_stm32f446retx
LD_FILE := ${CURDIR}/src/utils/STM32f446RETX_FLASH.ld

CC_FLAGS := -mcpu=$(CPU) -mthumb --specs=nano.specs \
			-fdata-sections -ffunction-sections -Wl,--gc-sections
# -mfpu=fpv4-sp-d16 -mfloat-abi=hard for f-4
# specs files can be found in [arm-none-eabi-gcc -print-sysroot -> ./lib]
CC_FLAGS += -g3 -O0 # add debug symbols, optimization
# CC_FLAGS += -Wall -Wextra -Werror -Wundef -Wshadow -Wdouble-promotion \
			-Wformat-truncation -Wconversion -Wno-deprecated -Wno-shadow	# warnings
CC_FLAGS += -v

LD_FLAGS := --print-memory-usage --no-warn-rwx-segment -u _printf_float

# System libraries
LIBS := m

BUILD_DIR := ${CURDIR}/build
OUT_DIR := ${CURDIR}/out
BINARY_NAME := ${notdir ${CURDIR}}


DRIVERS_PATH = /Users/sat/STM32/drivers/STM32F4

CMSIS_INC := $(DRIVERS_PATH)/CMSIS/include
CMSIS_DEVICE_INC := $(DRIVERS_PATH)/CMSIS/Device/ST/STM32F4xx/include
