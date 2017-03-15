DEVICE          = stm32f446re
FAMILY          = STM32F4
LDSCRIPT        = stm32f446re.ld
OPENCM3_DIR     = libopencm3
OBJS            = main.o os.o
OBJS            += silabs/ezradio.o
INCLUDE         = -I.

INCLUDE         += -I$(OPENCM3_DIR)/include/
ARCH            += -mthumb
ARCH            += -mcpu=cortex-m4
ARCH            += -mfloat-abi=hard -mfpu=fpv4-sp-d16
CFLAGS          += -Os -ggdb3 $(INCLUDE) -D$(FAMILY) $(ARCH) -fno-rtti
CXXFLAGS        += -Os -ggdb3 $(INCLUDE) -D$(FAMILY) $(ARCH) -fno-rtti
CPPFLAGS        += -MD
LDFLAGS         += -static -nostartfiles $(ARCH) -fno-rtti
LDLIBS          += -L$(OPENCM3_DIR)/lib -Wl,--start-group -lgcc -lc -lnosys -Wl,--end-group 
LDLIBS          += -lopencm3_stm32f4

#include $(OPENCM3_DIR)/mk/genlink-config.mk
include $(OPENCM3_DIR)/mk/gcc-config.mk

.PHONY: clean all

all: binary.elf binary.bin

clean:
	$(Q)$(RM) -rf binary.* *.o

#include $(OPENCM3_DIR)/mk/genlink-rules.mk
include $(OPENCM3_DIR)/mk/gcc-rules.mk


#OOCD_INTERFACE = stlink-v2-1
#OOCD_BOARD = st_nucleo_f446re
 
