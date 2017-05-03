#---------------------------------------------------------------------------
#	Секция настроек пользователя
#---------------------------------------------------------------------------
#VERSION			= Debug
VERSION		= Release
TOOL_PATH		= /usr/arm-none-eabi
SHELL			= bash
INC_PATH		= $(TOOL_PATH)/include
LIB_PATH		= $(TOOL_PATH)/lib
OOCD_VERSION			= 0.6.0-rc2
OOCD_INTERFACE_CONFIG	= interface/ftdi_jtag.cfg
OOCD_TARGET_CONFIG		= target/stm32f1x.cfg
PRJ_NAME		= test

SRC = startup_stm32f10x_md.s
SRC += core_cm3.c
SRC += stm32f10x_adc.c
SRC += stm32f10x_cec.c
SRC += stm32f10x_dbgmcu.c
SRC += stm32f10x_flash.c
SRC += stm32f10x_i2c.c
SRC += stm32f10x_pwr.c
SRC += stm32f10x_sdio.c
SRC += stm32f10x_usart.c
SRC += stm32f10x_bkp.c
SRC += stm32f10x_crc.c
SRC += stm32f10x_dma.c
SRC += stm32f10x_fsmc.c
SRC += stm32f10x_it.c
SRC += stm32f10x_rcc.c
SRC += stm32f10x_spi.c
SRC += stm32f10x_wwdg.c
SRC += misc.c
SRC += stm32f10x_can.c
SRC += stm32f10x_dac.c
SRC += stm32f10x_exti.c
SRC += stm32f10x_gpio.c
SRC += stm32f10x_iwdg.c
SRC += stm32f10x_rtc.c
SRC += stm32f10x_tim.c
SRC += system_stm32f10x.c
SRC += main.c

INCLUDE			= $(INC_DIR)
INCLUDE			+= $(INC_PATH)
DEFINE			= STM32F10X_MD
DEFINE			+= USE_STDPERIPH_DRIVER
DEFINE			+= F_CPU=72000000
#DEFINE			+= USE_OLIMEXINO_STM32
DEFINE			+= USE_STM32H_103
OPTIMIZE	 	= 2
DEBUG			= 3
LD_SCRIPT		= stm32_flash.ld
#----------------------------------------------------------------------------
#	Секция параметров сборки
#----------------------------------------------------------------------------
CROSS_COMPILE	= arm-none-eabi-
CC				= $(CROSS_COMPILE)gcc
AS				= $(CROSS_COMPILE)gcc
LD				= $(CROSS_COMPILE)gcc
CPPC			= $(CROSS_COMPILE)g++
PROG			= openocd-$(OOCD_VERSION)
SIZE			= $(CROSS_COMPILE)size
OBJCOPY			= $(CROSS_COMPILE)objcopy
OBJDUMP			= $(CROSS_COMPILE)objdump
CCFLAGS			= -Wall -mcpu=cortex-m3 -mthumb
CCFLAGS			+= $(addprefix -D,$(DEFINE)) $(addprefix -I,$(INCLUDE))
CCFLAGS			+= -ffunction-sections -fdata-sections
ifeq ($(VERSION),Debug)
CCFLAGS			+=  -g$(DEBUG) -Os -DUSE_FULL_ASSERT
endif
ifeq ($(VERSION),Release)
CCFLAGS			+= -O$(OPTIMIZE)
endif
ASFLAGS			= -Wall -mcpu=cortex-m3 -mthumb
ASFLAGS			+= -Wa,-adhlns=$(<:.s=.lst)
ASFLAGS			+= $(addprefix -D,$(DEFINE)) $(addprefix -I,$(INCLUDE))
ifeq ($(VERSION),Debug)
ASFLAGS			+= -g$(DEBUG) -O0 -DUSE_FULL_ASSERT
endif
LDFLAGS			= -nostartfiles -T$(LD_SCRIPT)
LDFLAGS			+= -Wl,-Map,$(IMAGE).map,--cref -Wl,--gc-sections
CPPCFLAGS		= $(CCFLAGS)
CPPCFLAGS		+= -x c++
PROGFLAGS		= -d0
PROGFLAGS		+= -f $(OOCD_INTERFACE_CONFIG) -f $(OOCD_TARGET_CONFIG)
PROGFLAGS		+= -c init -c targets
PROGFLAGS		+= -c "halt" -c "flash probe 0"
PROGFLAGS		+= -c "flash write_image erase $(IMAGE).elf 0x00000000 elf"
PROGFLAGS		+= -c "reset run" -c shutdown
SRC_DIR			= src
INC_DIR			= inc
OBJ_DIR			= obj/$(VERSION)
OUT_DIR			= bin/$(VERSION)
IMAGE			= $(OUT_DIR)/$(PRJ_NAME)
#----------------------------------------------------------------------------
#	Секция компиляции
#----------------------------------------------------------------------------
SOURCES		= $(wildcard $(addprefix $(SRC_DIR)/,$(SRC)))

ifneq ($(filter %.c,$(notdir $(SOURCES))),)
OBJECTS		+= $(addprefix $(OBJ_DIR)/,$(patsubst %.c, %.o,$(filter %.c,$(notdir $(SOURCES)))))
endif

ifneq ($(filter %.cpp,$(notdir $(SOURCES))),)
OBJECTS		+= $(addprefix $(OBJ_DIR)/,$(patsubst %.cpp, %.o,$(filter %.cpp,$(notdir $(SOURCES)))))
endif

ifneq ($(filter %.s,$(notdir $(SOURCES))),)
OBJECTS		+= $(addprefix $(OBJ_DIR)/,$(patsubst %.s, %.o,$(filter %.s,$(notdir $(SOURCES)))))
endif

ifneq ($(filter %.S,$(notdir $(SOURCES))),)
OBJECTS		+= $(addprefix $(OBJ_DIR)/,$(patsubst %.S, %.o,$(filter %.S,$(notdir $(SOURCES)))))
endif

#---------------------------------------------------------------------------
#	Секция сборки
#---------------------------------------------------------------------------
Program: kdevelop_path elf $(IMAGE).bin
	# $(PROG) $(PROGFLAGS)
	@echo "--------------------- COMPLETE -----------------------"

kdevelop_path:
	@echo -n > .kdev_include_paths
	@for i in $(INCLUDE) ; do \
	echo $$i >> .kdev_include_paths ; \
	done

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c
	@echo $<
	@echo "------------------------------------------------------"
	$(CC) $(CCFLAGS) -MMD -c $< -o $@

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	@echo $<
	@echo "------------------------------------------------------"
	$(CPPC) $(CPPCFLAGS) -MMD -c $< -o $@

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.s
	@echo $<
	@echo "------------------------------------------------------"
	$(AS) $(ASFLAGS) -c $< -o $@

$(VERSION): bin hex size
	@echo "--------------------- COMPLETE -----------------------"

bin:$(IMAGE).bin
hex:$(IMAGE).hex
lst:$(IMAGE).lst
elf:$(IMAGE).elf
size:$(IMAGE).elf
	@echo $@
	@echo "------------------------------------------------------"
	$(SIZE) $(IMAGE).elf

$(IMAGE).bin:$(IMAGE).elf
	@echo $@
	@echo "------------------------------------------------------"
	$(OBJCOPY) -O binary $< $@

$(IMAGE).hex:$(IMAGE).elf
	@echo $@
	@echo "------------------------------------------------------"
	$(OBJCOPY) -O ihex $< $@

$(IMAGE).lst:$(IMAGE).elf
	@echo $@
	@echo "------------------------------------------------------"
	$(OBJDUMP) -D $<  > $@

$(IMAGE).elf:$(OBJECTS)
	@echo $@
	@echo "------------------------------------------------------"
	$(LD) $(CCFLAGS) $(LDFLAGS) $^ -o $@

clean$(VERSION):
	rm -f $(OBJECTS)
	rm -f $(patsubst %.o, %.d,$(OBJECTS))
	rm -f $(IMAGE).hex $(IMAGE).bin $(IMAGE).elf $(IMAGE).map
	@echo "--------------------- COMPLETE -----------------------"

clean: clean$(VERSION)
	@echo "--------------------- COMPLETE -----------------------"

.PHONY:  Program Debug Release cleanDebug cleanRelease
include $(wildcard $(OUT_DIR)/*.d)
