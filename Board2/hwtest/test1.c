#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>

#include <stdint.h>
#include <stdio.h>
#include <errno.h>

#include "systick.h"
#include "debug.h"

#include <silabs/radio.h>
#include "radio_hal.h"
#include "afsk.h"

#define TCXO_FREQ_HZ            26000000uL
#define TRANSMIT_FREQUENCY_HZ  144250000uL
#define AFSK_DEVIATION_HZ            500uL

void clock_setup()
{
    //rcc_clock_setup_in_hsi_out_16mhz();
    rcc_clock_setup_in_hsi_out_24mhz();
}

void gpio_setup()
{
}

void partNumberTest()
{
    while (1) {        
        uint8_t rc;
        rc = si446x_getPartID(NULL);
        //if (!rc) testFail(4);
    }
}

void prnTest() 
{
    uint8_t rc = 0;
    
    if (!rc) rc = si446x_setFrequency(TRANSMIT_FREQUENCY_HZ, 5000);
    if (!rc) rc = si446x_setModulation(EZR_MOD_SOURCE_PRN | EZR_MOD_TYPE_2FSK);
    if (!rc) rc = si446x_setDataRate(1000);
    if (!rc) rc = si446x_setPower(0x10);
    if (rc) testFail(3);  
    
    si446x_tune();
    delay(100);
    
    while (1) {
        LED_ON;
        si446x_txOn();        
        delay(200);
        
        LED_OFF;
        si446x_txOff();
        delay(2000);       
    }
}

void afskTest() 
{
    uint8_t rc = 0;
    uint8_t modulation = EZR_MOD_TYPE_2FSK | EZR_MOD_SOURCE_DIRECT_MODE | 
                         EZR_MOD_TX_MODE_ASYNC | EZR_MOD_TX_MODE_GPIO1;

    afsk_setup();    
    
    if (!rc) rc = si446x_setFrequency(TRANSMIT_FREQUENCY_HZ, AFSK_DEVIATION_HZ);
    if (!rc) rc = si446x_setModulation(modulation);
    if (!rc) rc = si446x_setDataRate(1200);
    if (!rc) rc = si446x_setPower(0x10);
    if (!rc) rc = si446x_setupGPIO1(EZR_GPIO_MODE_TX_DATA);
    if (rc) testFail(3);
        
    si446x_tune();
    delay(100);

    while (1) {
        LED_ON;
        si446x_txOn();
        delay(20);

        /* Start data modulation */
        delay(200);

        LED_OFF;
        si446x_txOff();
        delay(2000);
    }
}

int main()
{
    uint8_t rc;

    /* Configure hardware peripherals */
    clock_setup();
    systick_setup();
    gpio_setup();
    spi_setup();
    si446x_setup();
    debug_setup();
    LED_OFF;

    /* Start up SI446x */
    si446x_shutdown();
    delay(200);
    si446x_wakeup();
    delay(200);
  
    rc = si446x_boot(TCXO_FREQ_HZ);
    if (rc) testFail(2);
    
    //partNumberTest();
    //prnTest();
    afskTest();
}
