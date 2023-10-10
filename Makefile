##########################################################################################################################
# File automatically-generated by tool: [projectgenerator] version: [4.1.0] date: [Tue Oct 10 11:57:11 PDT 2023]
##########################################################################################################################

# ------------------------------------------------
# Generic Makefile (based on gcc)
#
# ChangeLog :
#	2017-02-10 - Several enhancements + project update mode
#   2015-07-22 - first version
# ------------------------------------------------

######################################
# target
######################################
TARGET = test


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
# C_SOURCES := $(wildcard \
# 	src/*.c|\
# 	system/src/*.c)
# C_SOURCES := $(filter-out \
# 	src/stm32f0-stdperiph/stm32f0xx_can.c|\
# 	src/stm32f0-stdperiph/stm32f0xx_cec.c|\
# 	src/stm32f0-stdperiph/stm32f0xx_comp.c|\
# 	src/stm32f0-stdperiph/stm32f0xx_crc.c|\
# 	src/stm32f0-stdperiph/stm32f0xx_crs.c|\
# 	src/stm32f0-stdperiph/stm32f0xx_dac.c|\
# 	src/stm32f0-stdperiph/stm32f0xx_dbgmcu.c|\
# 	src/stm32f0-stdperiph/stm32f0xx_exti.c|\
# 	src/stm32f0-stdperiph/stm32f0xx_flash.c|\
# 	src/stm32f0-stdperiph/stm32f0xx_i2c.c|\
# 	src/stm32f0-stdperiph/stm32f0xx_iwdg.c|\
# 	src/stm32f0-stdperiph/stm32f0xx_pwr.c|\
# 	src/stm32f0-stdperiph/stm32f0xx_rtc.c|\
# 	src/stm32f0-stdperiph/stm32f0xx_syscfg.c|\
# 	src/stm32f0-stdperiph/stm32f0xx_tim.c|\
# 	src/stm32f0-stdperiph/stm32f0xx_wwdg.c, $(C_SOURCES))

# C sources
C_SOURCES =  \
src/BlinkLed.c \
src/spi.c \
src/ssd1306.c \
src/Timer.c \
src/uart.c \
src/OSC/OSCMatch.c

C_SOURCES += \
system/src/stm32f0-stdperiph/stm32f0xx_adc.c \
system/src/stm32f0-stdperiph/stm32f0xx_dma.c \
system/src/stm32f0-stdperiph/stm32f0xx_gpio.c \
system/src/stm32f0-stdperiph/stm32f0xx_misc.c \
system/src/stm32f0-stdperiph/stm32f0xx_rcc.c \
system/src/stm32f0-stdperiph/stm32f0xx_spi.c \
system/src/stm32f0-stdperiph/stm32f0xx_usart.c \

C_SOURCES += \
system/src/newlib/_exit.c \
system/src/newlib/_sbrk.c \
system/src/newlib/_syscalls.c \
system/src/newlib/_write.c \
system/src/newlib/assert.c 
# system/src/newlib/_startup.c \

C_SOURCES += \
system/src/diag/Trace.c \
system/src/diag/trace_impl.c 

C_SOURCES += \
system/src/cortexm/_initialize_hardware.c \
system/src/cortexm/_reset_hardware.c \
system/src/cortexm/exception_handlers.c 

C_SOURCES += \
system/src/cmsis/system_stm32f0xx.c \
system/src/cmsis/vectors_stm32f0xx.c 


CXX_SOURCES = \
src/main.cpp \
src/SLIPEncodedSerial.cpp \
src/OSC/OSCData.cpp \
src/OSC/OSCMessage.cpp \
src/OSC/OSCTiming.cpp \
src/OSC/SimpleWriter.cpp

CXX_SRCS += \
system/src/newlib/_cxx.cpp 


#######################################
# binaries
#######################################
PREFIX = arm-none-eabi-
# The gcc compiler bin path can be either defined in make command via GCC_PATH variable (> make GCC_PATH=xxx)
# either it can be added to the PATH environment variable.
ifdef GCC_PATH
CC = $(GCC_PATH)/$(PREFIX)gcc
CXX = $(GCC_PATH)/$(PREFIX)g++
AS = $(GCC_PATH)/$(PREFIX)g++ -x assembler-with-cpp
CP = $(GCC_PATH)/$(PREFIX)objcopy
SZ = $(GCC_PATH)/$(PREFIX)size
else
CC = $(PREFIX)gcc
CXX = $(PREFIX)g++
AS = $(PREFIX)g++ -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
endif
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S
 
#######################################
# CFLAGS
#######################################
# cpu
CPU = -mcpu=cortex-m0

# fpu
# NONE for Cortex-M0/M0+/M3

# float-abi

# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# macros for gcc
# AS defines
AS_DEFS = 

# C defines
C_DEFS =  \
-DDEBUG \
-DUSE_FULL_ASSERT \
-DSTM32F051 \
-DUSE_STDPERIPH_DRIVER \
-DHSE_VALUE=8000000

# AS includes
AS_INCLUDES = 

# C includes
C_INCLUDES =  \
-Isrc \
-Iinclude \
-Isystem/include \
-Isystem/include/cmsis \
-Isystem/include/stm32f0-stdperiph

CXX_INCLUDES = \
-Isrc \
-Iinclude \
-Isystem/include \
-Isystem/include/cmsis \
-Isystem/include/stm32f0-stdperiph


# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif

# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"

CXXFLAGS = $(MCU) $(C_DEFS) $(CXX_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

ifeq ($(DEBUG), 1)
CXXFLAGS += -g -gdwarf-2
endif

# Generate dependency information
CXXFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"


#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT_MEM = ldscripts/mem.ld
LDSCRIPT_LIBS = ldscripts/libs.ld
LDSCRIPT_SECTIONS =ldscripts/sections.ld

# libraries
LIBS = -lc -lm -lnosys 
LIBDIR = 
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT_MEM) -T$(LDSCRIPT_LIBS) -T$(LDSCRIPT_SECTIONS) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin


#######################################
# build the application
#######################################
# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))

OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(CXX_SOURCES:.cpp=.o)))
vpath %.cpp $(sort $(dir $(CXX_SOURCES)))

# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.cpp Makefile | $(BUILD_DIR) 
	$(CXX) -c $(CXXFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.cpp=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CXX) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@
	
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@	
	
$(BUILD_DIR):
	mkdir $@		

#######################################
# clean up
#######################################
clean:
	-rm -fR $(BUILD_DIR)
  
#######################################
# dependencies
#######################################
-include $(wildcard $(BUILD_DIR)/*.d)

# *** EOF ***