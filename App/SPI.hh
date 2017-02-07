#pragma once

#include "app.h"

#include "spi.h"

class SPIDevice {
public:
  SPIDevice(int pinCS) : _pinCS(pinCS) {}

  void select() {
    //digitalWrite(_pinCS, LOW);
    HAL_GPIO_WritePin(TRX_nCS_GPIO_Port, TRX_nCS_Pin, GPIO_PIN_RESET);
    //SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  }

  void write(uint8_t x) {
    //SPI.transfer(x);
    HAL_SPI_Transmit(&hspi1, &x, 1, 100);
  }

  uint8_t read() {
    uint8_t x;
    HAL_SPI_Receive(&hspi1, &x, 1, 100);
    return x;
    //return SPI.transfer(0xFF);
  }

  void release() {
    //SPI.endTransaction();
    //digitalWrite(_pinCS, HIGH);
    HAL_GPIO_WritePin(TRX_nCS_GPIO_Port, TRX_nCS_Pin, GPIO_PIN_SET);
  }

private:
  int _pinCS;
};
