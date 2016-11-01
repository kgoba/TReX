#include "app.h"
#include "si4x6x.h"
#include "radio_config_Si4463.h"

#include "debug.h"

#include <string.h>


const bool isMaster = true;

const int kPacketLength = 7;
const int kPacketsInBurst = 10;


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


void resetTX() {
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


bool checkReceived(uint8_t *rxBuf, uint32_t timeout = 1) {
  tx.startRX(0, kPacketLength, Si446x::kStateNoChange, Si446x::kStateRX, Si446x::kStateRX);
    
  //uint16_t nTry = 100;
  while (true) {
    Si446x::IRQStatus irqStatus;
    tx.getIntStatus(irqStatus);

    if (irqStatus.isPacketRXPending()) {   
      //dbg.print("Received data\n");
      uint8_t pktLen = tx.getAvailableRX();
      if (pktLen >= kPacketLength) {
        dbg.print("  Received ");
        dbg.print(pktLen); 
        dbg.print(" bytes, packet: ");
        tx.readRX(rxBuf, kPacketLength);
        for (uint8_t idx = 0; idx < kPacketLength; idx++) {
          dbg.print(rxBuf[idx]);
          dbg.print(" ");
        }
        dbg.print("\n");
        pktLen -= kPacketLength;
      }
      return true;
    }
    
    if (timeout == 0) break;
    delay(100);
    timeout--;
  }
  return false;
}

void transmit(uint8_t *data) {
  dbg.print("Transmitting packet burst...");

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
    delay(100);
  }

  //delay(200);
  dbg.print("done\n");
}  


void loop()
{
  static bool initialized = false;
 
  if (!initialized) {
    setLED(true);
    if (initRadio()) {
      tx.getIntStatus();    // clear all interrupts
      tx.setXOTune(56);
      
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
      }
    }
    else {
      setLED(false);
      delay(1000);
    }
  }
  else {         
    uint8_t packet[kPacketLength];
    if (isMaster) {
      for (uint8_t idx = 0; idx < kPacketLength; idx++) {
        packet[idx] = idx;
      }
      // Send 10 packets
      setRedLED(true);
      transmit(packet);    
      setRedLED(false);

      dbg.print("Listening...\n");
      uint8_t nReceived = 0;
      if (checkReceived(packet, 10)) {
        nReceived++;
        setGreenLED(true);
        // Receive until no more packets in 500 ms
        while (checkReceived(packet, 5)) {
          nReceived++;
        }
        setGreenLED(false);
        
        // Parse reply data
        uint8_t nTransmitted = 0;
        if ((packet[0] == packet[1]) && (packet[0] == packet[2])) {
          nTransmitted = packet[0];
        }
        dbg.print("Replies: "); dbg.print(nReceived); dbg.print(", received: "); dbg.print(nTransmitted);
        dbg.print("\n");
      }

      delay(2000);
      //setGreenLED(false);
      //checkReceived();
    }
    else {
      // Check if packet can be received in 5 seconds
      dbg.print("Listening...\n");
      uint8_t nReceived = 0;
      if (checkReceived(packet, 50)) {
        nReceived++;
        setGreenLED(true);
        // Receive until no more packets in 500 ms
        while (checkReceived(packet, 5)) {
          nReceived++;
        }
        setGreenLED(false);

        packet[0] = nReceived;
        packet[1] = nReceived;
        packet[2] = nReceived;
        // Transmit reply
        setRedLED(true);
        transmit(packet);
        setRedLED(false);

        delay(1000);
      }
    }
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
