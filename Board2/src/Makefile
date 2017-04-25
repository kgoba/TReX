#DEVICE          = stm32f446re
DEVICE		= stm32f030c8
#DEVICE		= stm32f103c8

ARCH            += -mthumb

ifeq ($(DEVICE),stm32f446re)
  ARCH            += -mcpu=cortex-m4
  ARCH            += -mfloat-abi=hard -mfpu=fpv4-sp-d16
  FAMILY          = STM32F4
  LDSCRIPT        = ../stm32f446re.ld
  OPENCM3_LIB     = opencm3_stm32f4
else 
ifeq ($(DEVICE),stm32f103c8)
  ARCH            += -mcpu=cortex-m0
  ARCH            += -mfloat-abi=soft
  FAMILY          = STM32F1
  LDSCRIPT        = ../stm32f103x8.ld
  OPENCM3_LIB     = opencm3_stm32f1
else
  ARCH            += -mcpu=cortex-m0
  ARCH            += -mfloat-abi=soft
  FAMILY          = STM32F0
  LDSCRIPT        = ../stm32f030x8.ld
  OPENCM3_LIB     = opencm3_stm32f0
endif
endif

OPENCM3_DIR     = ../libopencm3
#SRC             = $(wildcard,*.c) $(wildcard,*.cc)
#OBJS		= $(patsubst *.c,*.o,$(SRC)) $(patsubst *.cc,*.o,$(SRC))
OBJS            = test2.o 
OBJS		+= main.o os.o usart.o
OBJS            += ../silabspp/ezradio.o
INCLUDE         = -I. -I..

INCLUDE         += -I$(OPENCM3_DIR)/include/
CFLAGS          += -Os -ggdb3 $(INCLUDE) -D$(FAMILY) $(ARCH) -ffunction-sections -fdata-sections
CXXFLAGS        += -Os -ggdb3 $(INCLUDE) -D$(FAMILY) $(ARCH) -fno-rtti -fno-exceptions -ffunction-sections -fdata-sections
CPPFLAGS        += -MD
LDFLAGS         += -static -nostartfiles $(ARCH) -fno-rtti
LDFLAGS		+= -Wl,--start-group -Wl,--end-group -Wl,--gc-sections
LDFLAGS		+= -lgcc -lc --specs=nosys.specs
LDLIBS          += -L$(OPENCM3_DIR)/lib
LDLIBS          += -l$(OPENCM3_LIB)


#include $(OPENCM3_DIR)/mk/genlink-config.mk
include $(OPENCM3_DIR)/mk/gcc-config.mk

.PHONY: clean all

all: binary.elf binary.bin

clean:
	$(Q)$(RM) -rf binary.* *.o

flash:
	st-flash --reset write binary.bin 0x8000000

#include $(OPENCM3_DIR)/mk/genlink-rules.mk
include $(OPENCM3_DIR)/mk/gcc-rules.mk


#OOCD_INTERFACE = stlink-v2-1
#OOCD_BOARD = st_nucleo_f446re
 
