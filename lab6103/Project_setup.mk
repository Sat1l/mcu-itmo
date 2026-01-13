CPU := cortex-m4
DEVICE_FAMILY := STM32F1

STARTUP_FILE := ${CURDIR}/src/utils/startup_stm32f103xb.s
LD_FILE := ${CURDIR}/src/utils/STM32F103C8Tx_FLASH.ld

CC_FLAGS := -mcpu=$(CPU) -mthumb --specs=nano.specs \
			-fdata-sections -ffunction-sections -Wl,--gc-sections
# -mfpu=fpv4-sp-d16 -mfloat-abi=hard for f-4
# specs files can be found in [arm-none-eabi-gcc -print-sysroot -> ./lib]
CC_FLAGS += -g3 -O0 # add debug symbols, optimization
CC_FLAGS += -Wall -Wextra -Werror -Wundef -Wshadow -Wdouble-promotion \
			-Wformat-truncation -Wconversion -Wno-deprecated -Wno-shadow	# warnings

LD_FLAGS := --print-memory-usage --no-warn-rwx-segment -u _printf_float

# System libraries
LIBS := m

BUILD_DIR := ${CURDIR}/build
OUT_DIR := ${CURDIR}/out
BINARY_NAME := ${notdir ${CURDIR}}


DRIVERS_PATH = /Users/sat/STM32/drivers/$(DEVICE_FAMILY)

CMSIS_INC := $(DRIVERS_PATH)/CMSIS/include
CMSIS_DEVICE_INC := $(DRIVERS_PATH)/CMSIS/Device/ST/STM32F1xx/include
