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
#define TRANSMIT_FREQUENCY_HZ  144850000uL
#define AFSK_DEVIATION_HZ           1500uL

/*  PIN MAP
    -----------------------------
    PA0     Out     TCXO_nEN
    PA1     Out     TRX_SHDN
    PA2     UART2   GPS_TX      AF1
    PA3     UART2   GPS_RX      AF1
    PA4     Out     TRX_nCS
    PA5     SPI1    SCK         AF0
    PA6     SPI1    MISO        AF0
    PA7     SPI1    MOSI        AF0
    PA9     UART1   Dbg_TX      AF1
    PA10    UART1   Dbg_RX      AF1
    PA12    Out     LED
    PB0     In      TRX_nIRQ
    PB1     In/Out  TRX_GPIO1
    PB2     In/Out  TRX_GPIO0
    PB6     Out     FLASH_nCS
    PB7     Out     GPS_nEN
    PB10    I2C1    SCL         AF1
    PB11    I2C1    SDA         AF1
    PC13    In      GPS_PPS
*/

void clock_setup()
{
    //rcc_clock_setup_in_hsi_out_16mhz();
    rcc_clock_setup_in_hsi_out_24mhz();
}

void gpio_setup()
{
}

void gps_setup(void)
{
	/* Setup GPIO pins for USART1 transmit/receive. */
    rcc_periph_clock_enable(RCC_GPIOA);
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2);    // TX
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO3);   // RX
	gpio_set_af(GPIOA, GPIO_AF1, GPIO2);
	gpio_set_af(GPIOA, GPIO_AF1, GPIO3);

	/* Setup USART parameters. */
    rcc_periph_clock_enable(RCC_USART2);
	usart_set_baudrate(USART2, 9600);
	usart_set_databits(USART2, 8);
	usart_set_stopbits(USART2, USART_CR2_STOP_1_0BIT);
	usart_set_mode(USART2, USART_MODE_TX_RX);
	usart_set_parity(USART2, USART_PARITY_NONE);
	usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);

	/* Finally enable the USART. */
	usart_enable(USART2);
    
    /* setup GPS_nEN, GPS_PPS pins */
    rcc_periph_clock_enable(RCC_GPIOB);
    gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO7);
    gpio_set(GPIOB, GPIO7);

    rcc_periph_clock_enable(RCC_GPIOC);
    gpio_mode_setup(GPIOC, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, GPIO13);
}

void gps_enable()
{
    gpio_clear(GPIOB, GPIO7);
}

void gps_disable()
{
    gpio_set(GPIOB, GPIO7);
}

void gpsTest()
{
    LED_ON; delay(200); LED_OFF; delay(200);
    
    gps_setup();
    
    LED_ON; delay(200); LED_OFF;
    
    gps_enable();
    
    while (1) {
        usart_send_blocking(USART2, 'X');
    }
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

    uint8_t message[128];
    uint16_t messageLength = 128;
    
    for (uint16_t i = 0; i < messageLength; i++) {
        message[i] = ' ' + i;
    }

    afsk_setup();    
    
    if (!rc) rc = si446x_setFrequency(TRANSMIT_FREQUENCY_HZ, AFSK_DEVIATION_HZ);
    if (!rc) rc = si446x_setModulation(modulation);
    if (!rc) rc = si446x_setDataRate(1200*64);
    if (!rc) rc = si446x_setPower(0x10);
    if (!rc) rc = si446x_setupGPIO1(EZR_GPIO_MODE_TX_DATA);
    if (rc) testFail(3);
        
    si446x_tune();
    delay(100);

    while (1) {
        LED_ON;
        si446x_txOn();
        delay(20);

        afsk_send(message, 8 * messageLength);
        while (afsk_busy());

        LED_OFF;
        delay(20);
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
    
    //gpsTest();
    //partNumberTest();
    //prnTest();
    afskTest();
}
