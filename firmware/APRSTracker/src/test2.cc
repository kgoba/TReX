/** 
 *   The next test - UART
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/usart.h>

#include <stdint.h>
#include <stdio.h>

#include "silabspp/ezradio.hh"

#include "os.hh"
#include "pins.hh"
#include "usart.hh"

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

template<uint32_t USARTx>
class USARTBase {
public:
    static void enableClock() {
        rcc_periph_clock_enable(clock);
    }

    static void setup() {
        usart_set_baudrate(USARTx, 9600);
        usart_set_databits(USARTx, 8);
        usart_set_stopbits(USARTx, USART_CR2_STOP_1_0BIT);
        usart_set_mode(USARTx, USART_MODE_TX_RX);
        usart_set_parity(USARTx, USART_PARITY_NONE);
        usart_set_flow_control(USARTx, USART_FLOWCONTROL_NONE);
    }

    static void enableRXInterrupt() {
        usart_enable_rx_interrupt(USARTx);
    }

    static void enable() {
        usart_enable(USARTx);
    }

    static void enableIRQ() {
        nvic_enable_irq(irqn);
    }

    static const AlternateFunction  afRX;
    static const AlternateFunction  afTX;
    static const rcc_periph_clken   clock;
    static const uint8_t            irqn;
};

template<> const AlternateFunction  USARTBase<USART1>::afRX     = USART1_RX_AF;
template<> const AlternateFunction  USARTBase<USART1>::afTX     = USART1_TX_AF;
template<> const rcc_periph_clken   USARTBase<USART1>::clock    = RCC_USART1;
template<> const uint8_t            USARTBase<USART1>::irqn     = NVIC_USART1_IRQ;

template<> const AlternateFunction  USARTBase<USART2>::afRX     = USART2_RX_AF;
template<> const AlternateFunction  USARTBase<USART2>::afTX     = USART2_TX_AF;
template<> const rcc_periph_clken   USARTBase<USART2>::clock    = RCC_USART2;
template<> const uint8_t            USARTBase<USART2>::irqn     = NVIC_USART2_IRQ;


template<uint32_t USARTx, typename PinRX, typename PinTX>
class USART : public USARTBase<USARTx> {
public:
    typedef AlternateGPIO<PinRX, USARTBase<USARTx>::afRX> PinRXAF;
    typedef AlternateGPIO<PinTX, USARTBase<USARTx>::afTX> PinTXAF;
    typedef USARTBase<USARTx> Base;

    static void enableClock() {
        PinRX::enableClock();
        PinTX::enableClock();
        Base::enableClock();
    }
    
    static void setup() {
        PinRXAF::setup();
        PinTXAF::setup();
        Base::setup();
    }
};


DigitalOutput<PA12> pinLED;
DigitalOutput<PB6>  pinFlashSelect;
DigitalOutput<PB7>  pinGPSShutdown;
DigitalOutput<PA0>  pinTCXOShutdown;
DigitalOutput<PA1>  pinTXShutdown;
DigitalOutput<PA4>  pinTXSelect;
//DigitalOutput<PB1>  pinTXGPIO1;
DigitalOutput<PB2>  pinTXGPIO0;
//DigitalInput<PB0>   pinTXIRQ;
//DigitalInput<PC13>  pinGPSPPS;

//I2C<I2C1, PB11, PB10>     i2cBus;
//SPI<SPI1, PA7, PA6, PA5>  spiBus;
USART<USART2, PA3, PA2>     usartGPS;
USART<USART1, PA10, PA9>    usartDebug;



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

void setup()
{
    usart_init();
    //printf("*** RESET ***\n");

    GPIOPort<GPIOA>::enableClock();
    GPIOPort<GPIOB>::enableClock();
    GPIOPort<GPIOC>::enableClock();

    usartGPS.enableClock();
    usartDebug.enableClock();
    
    usartGPS.setup();
    usartDebug.setup();

    pinLED.setup();    
    
    spi_setup();
    
    pinTCXOShutdown.setup();
    pinTCXOShutdown.clear();
    
    pinTXShutdown.setup();
    pinTXShutdown.clear();
    
    pinTXSelect.setup();
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
    //printf("Read %02x back\n", spi_read8(SPI1));
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
