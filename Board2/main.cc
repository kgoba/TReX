#include <stdint.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/flash.h>

#include "silabs/ezradio.hh"

#include "os.hh"

template<uint32_t port>
class GPIOPort {
public:
    static void enableClock() {
    }
};

template<>
void GPIOPort<GPIOB>::enableClock() {
    rcc_periph_clock_enable(RCC_GPIOB);
}

template<uint32_t port, uint32_t pin>
class GPIOPin {
public:
    static void enableClock() {
        GPIOPort<port>::enableClock();
    }
    
    static void setupOutput() {
        gpio_mode_setup(port, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, pin);
    }
    
    static void set() {
        gpio_set(port, pin);
    }
    
    static void clear() {
        gpio_clear(port, pin);
    }
};

template<uint32_t port, uint32_t pin>
class DigitalOutput : public GPIOPin<port, pin> {
public:
    static void setup() {
        GPIOPin<port, pin>::setupOutput();
    }
};


//DigitalOutput<GPIOB, GPIO1> pinTXSelect;


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


void systick_setup(int freq)
{
    systick_set_reload(rcc_ahb_frequency / freq);
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    systick_counter_enable();
    systick_interrupt_enable();
}

void clock_setup()
{
    rcc_clock_setup_hse_3v3(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_48MHZ]);
}

EZRadioSPIDevice<SPI2Config, GPIOPin<GPIOB, GPIO1> > radioDev;
EZRadioPro radio(radioDev, 26 * 1000 * 1000);

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

int main(void)
{
    clock_setup();
    systick_setup(1000);
    
    rcc_periph_clock_enable(RCC_GPIOA);
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO5);

    radioDev.enable();
        
    radio.shutdown();
    delay_ms(100);
    radio.powerUpTCXO();
    
    EZRadioPro::PartInfo partInfo;
    radio.getPartInfo(partInfo);
    
    while(1)
    {
        gpio_set(GPIOA, GPIO5);
        delay_ms(200);
        gpio_clear(GPIOA, GPIO5);
        delay_ms(800);
    }
}
