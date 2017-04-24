/** 
 *   The next test - UART
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/flash.h>

#include <stdint.h>
#include <stdio.h>

#include "silabs/ezradio.hh"

#include "os.hh"
#include "pins.hh"
#include "usart.hh"

/*  PIN MAP
    -----------------------------
    PA12    Out     LED
    PA10    UART1   Dbg_RX      AF1
    PA9     UART1   Dbg_TX      AF1
    PA7     SPI1    MOSI        AF0
    PA6     SPI1    MISO        AF0
    PA5     SPI1    SCK         AF0
    PA4     Out     !TRX_CS
    PA3     UART2   GPS_RX      AF1
    PA2     UART2   GPS_TX      AF1
    PA1     Out     TRX_SHDN
    PA0     Out     !TCXO_EN
    PB11    I2C1    SDA         AF1
    PB10    I2C1    SCL         AF1
    PB7     Out     !GPS_EN
    PB6     Out     !FLASH_CS
    PB2     In/Out  TRX_GPIO0
    PB1     In/Out  TRX_GPIO1
    PB0     In      !TRX_IRQ
    PC13    In      GPS_PPS
*/

/*
class SPI1Config {
public:
    enum {
        GPIOx = GPIOA,
        GPIO_MOSI = GPIO7,
        GPIO_MISO = GPIO6,
        GPIO_SCK  = GPIO5,
        GPIO_AFx = GPIO_AF0,
        SPIx = SPI1
    };
    
    static const rcc_periph_clken RCC_SPI_CLOCK = RCC_SPI1;
    static const rcc_periph_clken RCC_GPIO_CLOCK = RCC_GPIOA;
};

template<typename traits, typename SelectPin>
class EZRadioSPIDevice : public SPIDeviceInterface {
public:
    EZRadioSPIDevice() {
        
    }
    
    void enableSPI(uint32_t clock = 1000000, uint32_t mode = 0) {
        rcc_periph_clock_enable(traits::RCC_SPI_CLOCK);
        rcc_periph_clock_enable(traits::RCC_GPIO_CLOCK);
        
        gpio_mode_setup(traits::GPIOx, GPIO_MODE_AF, GPIO_PUPD_NONE, traits::GPIO_MOSI | traits::GPIO_MISO | traits::GPIO_SCK);
        gpio_set_output_options(traits::GPIOx, GPIO_OTYPE_PP, GPIO_OSPEED_HIGH, traits::GPIO_MOSI | traits::GPIO_SCK);
        gpio_set_output_options(traits::GPIOx, GPIO_OTYPE_PP, GPIO_OSPEED_HIGH, traits::GPIO_MISO);
        gpio_set_af(traits::GPIOx, traits::GPIO_AFx, traits::GPIO_MOSI | traits::GPIO_MISO | traits::GPIO_SCK);

        uint32_t cpol = (mode & 0x02) >> 1; // SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE == 0
        uint32_t cpha = (mode & 0x01);      // SPI_CR1_CPHA_CLK_TRANSITION_1   == 0
        
        // SPI periperal clock frequency = rcc_apb1_frequency
        // SPI_CR1_BAUDRATE_FPCLK_DIV_2
        
        clock = SPI_CR1_BAUDRATE_FPCLK_DIV_64;
    	spi_init_master(traits::SPIx, clock, cpol,
    			cpha, SPI_CR1_CRCL_8BIT, SPI_CR1_MSBFIRST);
    	spi_enable_ss_output(traits::SPIx);

    	spi_enable(traits::SPIx);
        spi_set_crcl_8bit(traits::SPIx);
    }
    
    void enable() {
        selectPin.enable();
        release();
        
        enableSPI();
    }
    
    virtual void select() 
    {
        selectPin.clear();
    }
    
    virtual void release() 
    {
        selectPin.set();
    }

    virtual void write(uint8_t x) 
    {
        spi_send8(traits::SPIx, x);
    }
    
    virtual uint8_t read() 
    {
        spi_send(traits::SPIx, 0xFF);
        return spi_read8(traits::SPIx);
        //return spi_xfer(traits::SPIx, 0xFF);
    }
        
private:
    SelectPin   selectPin;
};

DigitalOutput<GPIOA, GPIO1> pinTXShutdown;
DigitalOutput<GPIOA, GPIO0> pinTCXOShutdown;
EZRadioSPIDevice<SPI1Config, DigitalOutput<GPIOA, GPIO4> > radioDev;

EZRadioPro radio(radioDev, 26 * 1000 * 1000);
*/

/*
void transmit(uint8_t *data) {
  dbg.print("Transmitting packet burst...");

  for (uint8_t idx = 0; idx < kPacketsInBurst; idx++) 
  {
    radio.getIntStatus();
    radio.writeTX(data, kPacketLength); 
    radio.startTX(0, kPacketLength);      
    uint16_t nTry = 100;
    while (nTry > 0) {
      EZRadioPro::IRQStatus irqStatus;
      radio.getIntStatus(irqStatus);

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
*/

DigitalOutput<GPIOA, GPIO1> pinTXShutdown;
DigitalOutput<GPIOA, GPIO0> pinTCXOShutdown;
DigitalOutput<GPIOA, GPIO4> pinTXSelect;


#define SPI_GPIO    GPIOA
#define SPI_AF      GPIO_AF0
#define MOSI_PIN    GPIO7
#define MISO_PIN    GPIO6
#define SCK_PIN     GPIO5

void spi_setup()
{
    /* Enable all clocks needed for SPI1 */
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_SPI1);
    
    /* Configure GPIOs: SCK=PA5, MISO=PA6 and MOSI=PA7 */
    gpio_mode_setup(SPI_GPIO, GPIO_MODE_AF, GPIO_PUPD_NONE, MOSI_PIN);
    gpio_set_output_options(SPI_GPIO, GPIO_OTYPE_PP, GPIO_OSPEED_HIGH, MOSI_PIN);
    gpio_set_af(SPI_GPIO, SPI_AF, MOSI_PIN);

    gpio_mode_setup(SPI_GPIO, GPIO_MODE_AF, GPIO_PUPD_PULLUP, MISO_PIN);
    gpio_set_output_options(SPI_GPIO, GPIO_OTYPE_PP, GPIO_OSPEED_HIGH, MISO_PIN);
    gpio_set_af(SPI_GPIO, SPI_AF, MISO_PIN);

    gpio_mode_setup(SPI_GPIO, GPIO_MODE_AF, GPIO_PUPD_NONE, SCK_PIN);
    gpio_set_output_options(SPI_GPIO, GPIO_OTYPE_PP, GPIO_OSPEED_HIGH, SCK_PIN);
    gpio_set_af(SPI_GPIO, SPI_AF, SCK_PIN);    
    
    spi_init_master(SPI1, SPI_CR1_BAUDRATE_FPCLK_DIV_16, SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
                    SPI_CR1_CPHA_CLK_TRANSITION_1, SPI_CR1_CRCL_8BIT, SPI_CR1_MSBFIRST);
    
    //spi_enable_ss_output(SPI1);
    //spi_disable_software_slave_management(SPI1);
    spi_enable_software_slave_management(SPI1); // SSM = 1
    spi_set_nss_high(SPI1);                     // SSI = 1
    spi_enable_ss_output(SPI1);                 // SSOE = 1
    spi_set_data_size(SPI1, SPI_CR2_DS_8BIT);
    spi_fifo_reception_threshold_8bit(SPI1);
    
    /* Enable SPI1 periph. */
    spi_enable(SPI1);

}

DigitalOutput<GPIOA, GPIO12> pinLED;

void setup()
{
    usart_init();
    printf("*** RESET ***\n");

    pinLED.enable();    
    
    spi_setup();
    
    pinTCXOShutdown.enable();
    pinTCXOShutdown.clear();
    
    pinTXShutdown.enable();
    pinTXShutdown.clear();
    
    pinTXSelect.enable();
    pinTXSelect.set();
    
    //radioDev.enable();
    
    pinLED.set();
    delay_ms(100);
    pinLED.clear();
    delay_ms(100);
}

void loop()
{
    pinLED.set();
    
    pinTXSelect.clear();
    delay_ms(1);
    spi_send8(SPI1, (uint8_t)0x44);
    //printf("Sent 0x44\n");
    spi_send8(SPI1, (uint8_t)0xff);
    printf("Read %02x back\n", spi_read8(SPI1));
    pinTXSelect.set();
    
    pinLED.clear();
    delay_ms(100);
    /*
    pinLED.set();
    printf("Powering up\n");
    radio.powerUpTCXO();
    
    EZRadioPro::PartInfo radioInfo;
    printf("Getting part info\n");
    radio.getPartInfo(radioInfo);
    printf("Part id: %04x rev %02x\n", radioInfo.getPartID(), radioInfo.getRevision());
    
    pinLED.clear();
    */
}
