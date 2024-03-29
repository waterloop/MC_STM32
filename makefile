##########################################################################################################################
# File automatically-generated by tool: [projectgenerator] version: [3.13.0-B3] date: [Thu Apr 29 21:47:12 MDT 2021]
##########################################################################################################################

# ------------------------------------------------
# Generic makefile (based on gcc)
#
# ChangeLog :
#	2017-02-10 - Several enhancements + project update mode
#   2015-07-22 - first version
# ------------------------------------------------

######################################
# target
######################################
TARGET = main

DEVICE_DIRNAME = STM32F405RGTx
BOARD = motor_controller

######################################
# building variables
######################################
# debug build?
DEBUG = 1
# optimization
OPT = -Og


#######################################
# paths
#######################################
# Build path
BUILD_DIR = build

######################################
# source
######################################
# C sources
# USER_INCLUDES := -I ./inc
CORE_C_SOURCES := $(shell find ./$(DEVICE_DIRNAME)/Core/Src -name "*.c")
CORE_CPP_SOURCES := $(shell find ./$(DEVICE_DIRNAME)/Core/Src -name "*.cpp")

HAL_SOURCES := $(shell find ./$(DEVICE_DIRNAME)/Drivers/STM32F4xx_HAL_Driver -name "*.c")

RTOS_SOURCES =  \
$(DEVICE_DIRNAME)/Middlewares/Third_Party/FreeRTOS/Source/event_groups.c \
$(DEVICE_DIRNAME)/Middlewares/Third_Party/FreeRTOS/Source/list.c \
$(DEVICE_DIRNAME)/Middlewares/Third_Party/FreeRTOS/Source/queue.c \
$(DEVICE_DIRNAME)/Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c \
$(DEVICE_DIRNAME)/Middlewares/Third_Party/FreeRTOS/Source/tasks.c \
$(DEVICE_DIRNAME)/Middlewares/Third_Party/FreeRTOS/Source/timers.c \
$(DEVICE_DIRNAME)/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.c \
$(DEVICE_DIRNAME)/Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c \
$(DEVICE_DIRNAME)/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c

C_SOURCES = $(CORE_C_SOURCES) $(HAL_SOURCES) $(RTOS_SOURCES)
CPP_SOURCES = $(CORE_CPP_SOURCES)

# ASM sources
ASM_SOURCES = $(DEVICE_DIRNAME)/Core/Startup/startup_stm32f405rgtx.s


#######################################
# binaries
#######################################
PREFIX = arm-none-eabi-
# The gcc compiler bin path can be either defined in make command via GCC_PATH variable (> make GCC_PATH=xxx)
# either it can be added to the PATH environment variable.
ifdef GCC_PATH
CC = $(GCC_PATH)/$(PREFIX)gcc
AS = $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
CP = $(GCC_PATH)/$(PREFIX)objcopy
SZ = $(GCC_PATH)/$(PREFIX)size
CPP_CC = $(GCC_PATH)/$(PREFIX)g++ -std=c++11
else
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
CPP_CC = $(PREFIX)g++ -std=c++11
endif
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S
 
#######################################
# CFLAGS
#######################################
# cpu
CPU = -mcpu=cortex-m4

# fpu
FPU = -mfpu=fpv4-sp-d16

# float-abi
FLOAT-ABI = -mfloat-abi=hard

# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# macros for gcc
# AS defines
AS_DEFS = 

C_DEFS = \
-D USE_HAL_DRIVER \
-D STM32F405xx \
-D MOTOR_CONTROLLER


# AS includes
AS_INCLUDES = 

# C includes
C_INCLUDES = \
-I ./$(DEVICE_DIRNAME)/Core/Inc \
-I ./$(DEVICE_DIRNAME)/Drivers/STM32F4xx_HAL_Driver/Inc \
-I ./$(DEVICE_DIRNAME)/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy \
-I ./$(DEVICE_DIRNAME)/Middlewares/Third_Party/FreeRTOS/Source/include \
-I ./$(DEVICE_DIRNAME)/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 \
-I ./$(DEVICE_DIRNAME)/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F \
-I ./$(DEVICE_DIRNAME)/Drivers/CMSIS/Device/ST/STM32F4xx/Include \
-I ./$(DEVICE_DIRNAME)/Drivers/CMSIS/Include \
-I ./WLoopCAN/include \
-I ./WLoopUtil/include

C_INCLUDES += $(USER_INCLUDES)

# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif


# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"


#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = $(DEVICE_DIRNAME)/STM32F405RGTX_FLASH.ld

# libraries
LIBS = -lc -lm -lnosys 
LIBDIR = 
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin


#######################################
# build the application
#######################################
# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))

# list of CPP program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(CPP_SOURCES:.cpp=.o)))
vpath %.cpp $(sort $(dir $(CPP_SOURCES)))

# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.c | $(BUILD_DIR) 
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@
	@echo ""

$(BUILD_DIR)/%.o: %.cpp | $(BUILD_DIR) 
	$(CPP_CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.cpp=.lst)) $< -o $@
	@echo ""

$(BUILD_DIR)/%.o: %.s | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@
	@echo ""

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) | libs
	$(CPP_CC) $(OBJECTS) ./WLoopCAN/bin/wloop_can.a ./WLoopUtil/bin/wloop_util.a $(LDFLAGS) -o $@
	@echo ""
	$(SZ) $@
	@echo ""

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@
	
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@	
	
$(BUILD_DIR):
	mkdir $@
	@echo ""

libs:
	cd WLoopCAN && make $(BOARD)
	cd WLoopUtil && make $(BOARD)

#######################################
# clean up
#######################################
clean:
	rm -rf $(BUILD_DIR)
	rm -rf ./WLoopCAN/bin
	rm -rf ./WLoopUtil/bin

analyze:
	$(PREFIX)objdump -t $(BUILD_DIR)/$(TARGET).elf

flash:
	st-flash write $(BUILD_DIR)/main.bin 0x08000000
	st-flash reset

#######################################
# dependencies
#######################################
-include $(wildcard $(BUILD_DIR)/*.d)

# *** EOF ***
