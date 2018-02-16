#pragma once

#include <stdint.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/spi.h>

void delay_ms(uint32_t milliSeconds);
void delay_us(uint32_t microSeconds);
uint32_t get_millis_elapsed(void);
