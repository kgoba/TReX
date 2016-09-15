#pragma once
#include <stdint.h>

extern "C" {
#include "stm32f0xx_hal.h"
}

#include "SPI.h"

void delay(uint32_t millis);
void delayMicroseconds(uint32_t micros);
//void digitalWrite(int pin);