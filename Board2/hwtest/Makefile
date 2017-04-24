TARGET		= hwtest
#DEVICE          = stm32f446re
DEVICE		= stm32f030c8
#DEVICE		= stm32f103c8

OPENCM3_DIR     = ../libopencm3

ARCH            += -mthumb
ARCH            += -mcpu=cortex-m0
ARCH            += -mfloat-abi=soft
FAMILY          = STM32F0
LDSCRIPT        = ../stm32f030x8.ld
OPENCM3_LIB     = opencm3_stm32f0

SRC             = $(wildcard,*.c) $(wildcard,*.cc)
OBJS            = test1.o ../silabs/radio.o radio_hal.o
INCLUDE         = -I. -I..

INCLUDE         += -I$(OPENCM3_DIR)/include/
CFLAGS          += -Os -ggdb3 $(INCLUDE) -D$(FAMILY) $(ARCH) -ffunction-sections -fdata-sections
CXXFLAGS        += -Os -ggdb3 $(INCLUDE) -D$(FAMILY) $(ARCH) -fno-rtti -ffunction-sections -fdata-sections
CPPFLAGS        += -MD
LDFLAGS         += -static -nostartfiles $(ARCH) -fno-rtti
LDFLAGS		+= -Wl,--start-group -Wl,--end-group -Wl,--gc-sections
LDFLAGS		+= -lgcc -lc --specs=nosys.specs
LDLIBS          += -L$(OPENCM3_DIR)/lib
LDLIBS          += -l$(OPENCM3_LIB)


#include $(OPENCM3_DIR)/mk/genlink-config.mk
include $(OPENCM3_DIR)/mk/gcc-config.mk

.PHONY: clean all

all: $(TARGET).elf $(TARGET).bin

clean:
	$(Q)$(RM) -rf $(TARGET).elf $(TARGET).bin $(OBJS)

flash:
	st-flash --reset write binary.bin 0x8000000

#include $(OPENCM3_DIR)/mk/genlink-rules.mk
include $(OPENCM3_DIR)/mk/gcc-rules.mk


#OOCD_INTERFACE = stlink-v2-1
#OOCD_BOARD = st_nucleo_f446re
 
