.PHONY: build clean flash erase info
include Project_setup.mk

# Binaries
CC := arm-none-eabi-gcc
AR := arm-none-eabi-ar
AS := arm-none-eabi-as
OBJCPY := arm-none-eabi-objcopy
SIZE := arm-none-eabi-size

# Color codes
RED := \033[0;31m
GREEN := \033[0;32m
BGREEN := \033[1;32m
UGREEN := \033[4;32m
NC := \033[0m  

# Format flags
LD_FLAGS := $(addprefix -Xlinker , $(LD_FLAGS)) 
LIBS := $(addprefix -l, $(LIBS))

MODULES := $(wildcard ./src/*/)
SOURCES := $(wildcard $(addsuffix *.[cs], ${MODULES}))

# paths for sources search 
vpath %.c ${MODULES} 
vpath %.s ${MODULES} 
vpath %.h ${MODULES} 

# Make include paths
MODULES_INCLUDE := $(addprefix -I, ${MODULES})
INCLUDES := -I$(CMSIS_INC) -I$(CMSIS_DEVICE_INC) ${MODULES_INCLUDE} 

APP_OBJECTS := $(addprefix ${BUILD_DIR}/, $(addsuffix .o, $(basename $(notdir ${SOURCES}))))

##### COMPILING #####

# Create build directory if not exists
$(BUILD_DIR):
	@mkdir -p $(BUILD_DIR)

$(OUT_DIR):
	@mkdir -p $(OUT_DIR)

# Dummy rule for headers to avoid "No rule to make target" error
%.h:
	@> /dev/null

# User app objects 
${BUILD_DIR}/%.o: %.c %.h
	@echo "${GREEN}Compiling $(notdir $@)${NC}"
	@$(CC) $(CC_FLAGS) $(INCLUDES) -c $< -o $@

${BUILD_DIR}/%.o: %.s
	@echo "${GREEN}Compiling $(notdir ${STARTUP_FILE}) ${NC}"
	@$(AS) -c $< -o $@


# Linking
${BUILD_DIR}/${BINARY_NAME}.elf: $(APP_OBJECTS) $(LD_FILE)
	@echo "\n${GREEN}Linking...${NC}"
	@$(CC) -Wl,-T,$(LD_FILE) $(LD_FLAGS) $(INCLUDES) $(CC_FLAGS) $(APP_OBJECTS) ${LIBS} \
	-o ${BUILD_DIR}/${BINARY_NAME}.elf  

# Convert ELF to BIN
${OUT_DIR}/${BINARY_NAME}.bin: ${BUILD_DIR}/${BINARY_NAME}.elf 
	@echo "${GREEN}Building ${BINARY_NAME}.bin${NC}"
	@$(OBJCPY) -O binary ${BUILD_DIR}/${BINARY_NAME}.elf ${OUT_DIR}/${BINARY_NAME}.bin
	
	
# Build app 
build: ${BUILD_DIR} ${OUT_DIR} ${OUT_DIR}/${BINARY_NAME}.bin 
	@echo "\n${UGREEN}Build finished!${NC}\n"
	@${SIZE} ${BUILD_DIR}/${BINARY_NAME}.elf
	@echo "\n"

erase:
	@echo "${RED} Erasing flash...${NC}"
	@st-flash erase 

flash: build
	@st-flash --reset write ${OUT_DIR}/${BINARY_NAME}.bin 0x08000000

clean:
	@echo "${RED}Cleaning...${NC}"
	@rm -rf build/ out/

info:
	@echo $(basename $(notdir $(wildcard src/*.c)))
	@st-info --probe

get-exec-path:
	@echo ${BUILD_DIR}/${BINARY_NAME}.elf