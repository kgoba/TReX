OPENCM3_DIR     = ./libopencm3

#SRC             = $(wildcard src/*.c) $(wildcard src/*.cpp)
#OBJS		= $(patsubst %.c,%.o, $(patsubst %.cpp,%.o,$(SRC)))
OBJS            = src/ptest.o
#OBJS		+= main.o os.o usart.o
#OBJS            += ./silabspp/ezradio.o

INCLUDE         += -I.

#include platform/stm32f103x8.mk
include platform/stm32f030x8.mk
include platform/libopencm3.mk
