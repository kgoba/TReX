/** 
 *   The next test - UART
 */

#include <stdint.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/flash.h>

//#include "silabs/ezradio.hh"

#include "os.hh"
#include "pins.hh"


//DigitalOutput<GPIOB, GPIO1> pinTXSelect;

/*
class SPI2Config {
public:
    enum {
        GPIOx = GPIOB,
        GPIO_MASK = GPIO13 | GPIO14 | GPIO15,
        GPIO_AFx = GPIO_AF5,
        SPIx = SPI2
    };
    
    static const rcc_periph_clken RCC_SPI_CLOCK = RCC_SPI2;
    static const rcc_periph_clken RCC_GPIO_CLOCK = RCC_GPIOB;
};



template<typename traits, typename SelectPin>
class EZRadioSPIDevice : public SPIDeviceInterface {
public:
    EZRadioSPIDevice() {
        
    }
    
    void enableSPI(uint32_t clock = 1000000, uint32_t mode = 0) {
        rcc_periph_clock_enable(traits::RCC_SPI_CLOCK);
        rcc_periph_clock_enable(traits::RCC_GPIO_CLOCK);
        
        gpio_mode_setup(traits::GPIOx, GPIO_MODE_AF, GPIO_PUPD_NONE,
        			traits::GPIO_MASK);
        gpio_set_af(traits::GPIOx, traits::GPIO_AFx, traits::GPIO_MASK);
        
        uint32_t cpol = (mode & 0x02) >> 1;
        uint32_t cpha = (mode & 0x01);
        
        // SPI periperal clock frequency = rcc_apb1_frequency
        // SPI_CR1_BAUDRATE_FPCLK_DIV_2
    	spi_init_master(traits::SPIx, clock, cpol,
    			cpha, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);
    	spi_disable_ss_output(traits::SPIx);

    	spi_enable(traits::SPIx);
    }
    
    void enable() {
        selectPin.enableClock();
        selectPin.setupOutput();
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
        spi_xfer(traits::SPIx, x);
    }
    
    virtual uint8_t read() 
    {
        return spi_xfer(traits::SPIx, 0xFF);
    }
        
private:
    SelectPin   selectPin;
};
*/

/*
EZRadioSPIDevice<SPI2Config, GPIOPin<GPIOB, GPIO1> > radioDev;
EZRadioPro radio(radioDev, 26 * 1000 * 1000);

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

enum PinName
{
    PA9,
    PA10,
    NC = (-1)
};


template<uint32_t USARTx>
class USARTPeriph {
public:
    static void setup() {        
    	/* Setup GPIO pins for USART1 transmit/receive. */
        rcc_periph_clock_enable(RCC_GPIOA);
    	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9);
    	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO10);
    	//gpio_set_output_options(GPIOA, GPIO_OTYPE_OD, GPIO_OSPEED_25MHZ, GPIO10);
    	gpio_set_af(GPIOA, GPIO_AF1, GPIO9);
    	gpio_set_af(GPIOA, GPIO_AF1, GPIO10);

    	/* Setup USART parameters. */
    	usart_set_baudrate(USARTx, 9600);
    	usart_set_databits(USARTx, 8);
    	usart_set_stopbits(USARTx, USART_CR2_STOP_1_0BIT);
    	usart_set_mode(USARTx, USART_MODE_TX_RX);
    	usart_set_parity(USARTx, USART_PARITY_NONE);
    	usart_set_flow_control(USARTx, USART_FLOWCONTROL_NONE);

    	/* Enable the USART1 interrupt. */
        switch (USARTx) {
            case USART1: nvic_enable_irq(NVIC_USART1_IRQ); break;
            case USART2: nvic_enable_irq(NVIC_USART2_IRQ); break;
        }

    	/* Enable USART Receive interrupt. */
    	usart_enable_rx_interrupt(USARTx);

    	/* Finally enable the USART. */
    	usart_enable(USARTx);
    }
    

};

template<uint32_t USARTx>
class USARTBase {
public:
    static void setup()
    {
    	/* Setup USART parameters. */
    	usart_set_baudrate(USARTx, 9600);
    	usart_set_databits(USARTx, 8);
    	usart_set_stopbits(USARTx, USART_CR2_STOP_1_0BIT);
    	usart_set_mode(USARTx, USART_MODE_TX_RX);
    	usart_set_parity(USARTx, USART_PARITY_NONE);
    	usart_set_flow_control(USARTx, USART_FLOWCONTROL_NONE);

    	/* Enable the USART1 interrupt. */
        switch (USARTx) {
            case USART1: nvic_enable_irq(NVIC_USART1_IRQ); break;
            case USART2: nvic_enable_irq(NVIC_USART2_IRQ); break;
        }

    	/* Enable USART Receive interrupt. */
    	usart_enable_rx_interrupt(USARTx);

    	/* Finally enable the USART. */
    	usart_enable(USARTx);    
    }
    
    static void isr() {
    	static uint8_t data = 'A';

    	/* Check if we were called because of RXNE. */
        if (usart_get_interrupt_source(USARTx, USART_ISR_RXNE)) {
    		/* Indicate that we got data. */
    		gpio_toggle(GPIOA, GPIO12);

    		/* Retrieve the data from the peripheral. */
    		data = usart_recv(USARTx);

    		/* Enable transmit interrupt so it sends back the data. */
    		usart_enable_tx_interrupt(USARTx);
    	}

    	/* Check if we were called because of TXE. */
        if (usart_get_interrupt_source(USARTx, USART_ISR_TXE)) {
    		/* Put data into the transmit register. */
    		usart_send(USARTx, data);

    		/* Disable the TXE interrupt as we don't need it anymore. */
    		usart_disable_tx_interrupt(USARTx);
    	}
    }
};

template<PinName pinRX, PinName pinTX>
class USART {
public:
};

template<>
class USART<PA10, PA9> : public USARTBase<USART1> {
public:
    static void setup() {
    	/* Setup GPIO pins for USART1 transmit/receive. */
        rcc_periph_clock_enable(RCC_GPIOA);
    	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9);
    	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO10);
    	//gpio_set_output_options(GPIOA, GPIO_OTYPE_OD, GPIO_OSPEED_25MHZ, GPIO10);
    	gpio_set_af(GPIOA, GPIO_AF1, GPIO9);
    	gpio_set_af(GPIOA, GPIO_AF1, GPIO10);

        USARTBase<USART1>::setup();
    }
};


USART<PA10, PA9> dbgSerial;

void usart1_isr(void)
{
    USARTBase<USART1>::isr();
}


DigitalOutput<GPIOA, GPIO12> pinLED;

void setup()
{
    pinLED.enable();
    dbgSerial.setup();
}

void loop()
{
    //pinLED.set();
    //delay_ms(200);
    //pinLED.clear();
    //delay_ms(800);
}
    
    /*
    radioDev.enable();
        
    radio.shutdown();
    delay_ms(100);
    radio.powerUpTCXO();
    
    EZRadioPro::PartInfo partInfo;
    radio.getPartInfo(partInfo);
    */
    
    /*
    rcc_periph_clock_enable(RCC_GPIOA);
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO12);
    while(1)
    {
        gpio_set(GPIOA, GPIO12);
        delay_ms(200);
        gpio_clear(GPIOA, GPIO12);
        delay_ms(800);
    }
    */
