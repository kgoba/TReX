#include "app.h"
#include "si4x6x.h"
#include "radio_config_Si4463.h"

#include <string.h>

#define PIN_NUMBER(x, y)      ((x << 16) | (y))

#define PA_4                  PIN_NUMBER(0, 4)

#define DBG_TX PA_2
#define DBG_RX PA_3

//
//Serial dbg(DBG_TX, DBG_RX, "hi");
//I2C i2c(PF_0, PF_1);

extern "C" {
  void setup();
  void loop();
}

uint32_t xtalFrequency = 26000000UL;

Si446x tx(PA_4, xtalFrequency);


class USBSerial {
public:
  bool _putc(char c) {
    while (isTXBusy()) {
      // Wait and do nothing
    }
    txBuf = c;
    uint8_t result = CDC_Transmit_FS(&txBuf, 1);
    return true;
  }

  bool _getc(char &c) {
    if (CDC_GetRXAvailable() == 0) return false;
    c = CDC_ReadByte();
    return true;
  }
  
  uint8_t available() {
    return CDC_GetRXAvailable();
  }
  
private:
  bool isTXBusy() {
    return CDC_GetTXState() != 0;
  }

  uint8_t txBuf;
  uint8_t rxBuf;
};

bool initModemAlt()
{  
  for (uint8_t nTry = 3; nTry > 0; nTry--) {
    //Serial.println("Initializing radio...");
    uint8_t config[] = RADIO_CONFIGURATION_DATA_ARRAY;
    if (tx.configure(config)) return true;
    //Serial.println("Failed");
    delay(100);
  }    
  return false;
}



void setup()
{
  // Reset Si446X
  HAL_GPIO_WritePin(TRX_SDN_GPIO_Port, TRX_SDN_Pin, GPIO_PIN_RESET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(TRX_SDN_GPIO_Port, TRX_SDN_Pin, GPIO_PIN_SET);
  HAL_Delay(100);
}

USBSerial dbg;

void loop()
{
  static bool initialized = false;
  
  if (!initialized) {
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_SET);
    if (initModemAlt()) {
      initialized = true;
    }
    else {
      HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_RESET);
      delay(1000);
    }
  }
  //tx.powerUpTCXO();
  
  //uint8_t HiMsg[]="hello\r\n";
  //CDC_Transmit_FS(HiMsg, strlen((const char *)HiMsg));
  
  //char c;
  //if (dbg._getc(c)) {
  //  dbg._putc(c);
  //}
  //dbg._putc('X');
}



void delay(uint32_t millis) {
  HAL_Delay(millis);
}

void delayMicroseconds(uint32_t micros) {
  TIM1->CNT = 0;
  while (TIM1->CNT < micros) {}
}
