#include "app.h"
#include "si4x6x.h"
#include "radio_config_Si4463.h"

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
}



void delay(uint32_t millis) {
  HAL_Delay(millis);
}

void delayMicroseconds(uint32_t micros) {
  TIM1->CNT = 0;
  while (TIM1->CNT < micros) {}
}
