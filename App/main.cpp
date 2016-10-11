#include "app.h"
#include "si4x6x.h"
#include "radio_config_Si4463.h"

#include "debug.h"

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

static Debug dbg;

void setLED(bool on) {
//  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void setGreenLED(bool on) {
  tx.configureGPIO(on ? 3 : 2, 0, 0, 0, 0, 0, 0);
}

void setRedLED(bool on) {
  tx.configureGPIO(0, on ? 3 : 2, 0, 0, 0, 0, 0);
}


bool resetTX() {
  // Reset Si446X
  HAL_GPIO_WritePin(TRX_SDN_GPIO_Port, TRX_SDN_Pin, GPIO_PIN_SET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(TRX_SDN_GPIO_Port, TRX_SDN_Pin, GPIO_PIN_RESET);
  HAL_Delay(100);
}

bool initRadio()
{  
  resetTX();
   
  for (uint8_t nTry = 3; nTry > 0; nTry--) {
    dbg.print("Initializing radio... ");
    uint8_t config[] = RADIO_CONFIGURATION_DATA_ARRAY;
    if (tx.configure(config)) {
      dbg.print("Success!\n");
      return true;
    }
    dbg.print("Failed\n");
    delay(100);
  }    
  return false;
}

void setup()
{
  dbg.print("---= RESET =---\n");
}

const int kPacketLength = 7;
const int kPacketsInBurst = 8;


void checkReceived() {
  dbg.print("Listening...\n");
  tx.startRX(0, 0, Si446x::kStateNoChange, Si446x::kStateRX, Si446x::kStateRX);
  
  uint8_t rxBuf[kPacketLength];
  
  uint16_t nTry = 100;
  while (nTry > 0) {
    Si446x::IRQStatus irqStatus;
    tx.getIntStatus(irqStatus);

    if (irqStatus.isPacketRXPending()) {   
      dbg.print("Received data\n");
      setGreenLED(true);
      uint8_t pktLen = tx.getAvailableRX();
      while (pktLen >= kPacketLength) {
        dbg.print("  Received packet\n");
        tx.readRX(rxBuf, kPacketLength);
        pktLen -= kPacketLength;
      }
    }
    
    nTry--;
    delay(20);
  }
}

void transmit() {
  dbg.print("Transmitting packet burst...");

  uint8_t data[kPacketLength] = { 0x06, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06 };

  for (uint8_t idx = 0; idx < kPacketsInBurst; idx++) 
  {
    tx.getIntStatus();
    tx.writeTX(data, kPacketLength); 
    tx.startTX(0, kPacketLength);      
    uint16_t nTry = 100;
    while (nTry > 0) {
      Si446x::IRQStatus irqStatus;
      tx.getIntStatus(irqStatus);

      if (irqStatus.isPacketSentPending()) {     
        break;
      }
      delay(10);
      nTry--;
    }
  }

  delay(200);
  dbg.print("done\n");
}  


void loop()
{
  static bool initialized = false;
 
  if (!initialized) {
    setLED(true);
    if (initRadio()) {
      tx.getIntStatus();    // clear all interrupts
      tx.setXOTune(55);
      
      Si446x::PartInfo info;
      tx.getPartInfo(info);
      dbg.print("Part ID: ");
      dbg.print(info.getPartID());
      dbg.print(" Rev: ");
      dbg.print(info.getRevision());
      dbg.print("\n");

      dbg.print("State: ");
      dbg.print(tx.getState());
      dbg.print("\n");      

      initialized = (info.getPartID() == 0x4463);
      
      if (initialized) {
        setLED(false);
        //tx.startRX(0, 0, Si446x::kStateNoChange, Si446x::kStateRX, Si446x::kStateRX);
      }
    }
    else {
      setLED(false);
      delay(1000);
    }
  }
  else {         
    setGreenLED(false);
    checkReceived();

    setRedLED(true);
    transmit();    
    setRedLED(false);
  }
}



void delay(uint32_t millis) {
  HAL_Delay(millis);
}

void delayMicroseconds(uint32_t uSecs) {
  
	// Dummy loop with 16 bit count wrap around
	//volatile uint32_t start = TIM1->CNT;
  //while((TIM1->CNT-start) <= uSecs);
  //HAL_Delay(1);
  //TIM1->CNT = 0;
  //while (TIM1->CNT < micros) {}
}
