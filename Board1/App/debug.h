#pragma once
#include "app.h"

class TextOutStream {
public: 
  void print(const char *str) {
    while (*str) {
      _putc(*str++);
    }
  }
  
  void print(uint16_t number) {
    print((uint8_t)(number >> 8));
    print((uint8_t)(number));
  }
  
  void print(uint8_t number) {
    uint8_t nibble = number >> 4;
    _putc(nibble + ((nibble > 9) ? ('A' - 10) : '0'));
    nibble = number & 0x0F;
    _putc(nibble + ((nibble > 9) ? ('A' - 10) : '0'));
  }
  
protected:
  virtual bool _putc(char c) = 0;
};

class NoDebug : public TextOutStream {
public:

private:
  virtual bool _putc(char c) {
    // Do nothing
    return true;
  }
};

class Debug : public TextOutStream {
public:
  uint8_t available() {
    //return CDC_GetRXAvailable();
    return 0;
  }
  
private:
  bool isTXBusy() {
    //return CDC_GetTXState() != 0;
    return false;
  }

  uint8_t txBuf;
  uint8_t rxBuf;
  
  virtual bool _putc(char c) {
    while (isTXBusy()) {
      // Wait and do nothing
    }
    txBuf = c;
    HAL_StatusTypeDef status = HAL_UART_Transmit(&huart2, &txBuf, 1, 100);
    //uint8_t result = CDC_Transmit_FS(&txBuf, 1);
    return true;
  }
  
  bool _getc(char &c) {
    HAL_StatusTypeDef status = HAL_UART_Receive(&huart2, &rxBuf, 1, 100);
    c = rxBuf;
    //if (CDC_GetRXAvailable() == 0) return false;
    //c = CDC_ReadByte();
    return true;
  }  
};
