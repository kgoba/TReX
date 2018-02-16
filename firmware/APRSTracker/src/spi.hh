#pragma once

#include <stdint.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/spi.h>

class GPIOPin {
public:
};

class SPI2Config {
protected:
    enum {
        GPIOx = GPIOB,
        GPIO_MASK = GPIO13 | GPIO14 | GPIO15,
        GPIO_AFx = GPIO_AF5,
        SPIx = SPI2
    };
    
    static const rcc_periph_clken RCC_SPI_CLOCK = RCC_SPI2;
    static const rcc_periph_clken RCC_GPIO_CLOCK = RCC_GPIOB;
};

template<typename traits>
class SPIPeriph : public traits {
public:
    SPIPeriph() {
    }
    
    void enable(uint32_t mode = 0) {
        rcc_periph_clock_enable(traits::RCC_SPI_CLOCK);
        rcc_periph_clock_enable(traits::RCC_GPIO_CLOCK);
        
        gpio_mode_setup(traits::GPIOx, GPIO_MODE_AF, GPIO_PUPD_NONE,
        			traits::GPIO_MASK);
        gpio_set_af(traits::GPIOx, traits::GPIO_AFx, traits::GPIO_MASK);
        
        uint32_t cpol = (mode & 0x02) >> 1;
        uint32_t cpha = (mode & 0x01);
        
        // SPI periperal clock frequency = rcc_apb1_frequency
    	spi_init_master(traits::SPIx, SPI_CR1_BAUDRATE_FPCLK_DIV_2, cpol,
    			cpha, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);
    	spi_disable_ss_output(traits::SPIx);

    	spi_enable(traits::SPIx);
    }

    void disable() {
    }
    
    void write(uint8_t value) {
        spi_write(traits::SPIx, value);
    }

    uint8_t read() {
        spi_write(traits::SPIx, 0xFF);
        return spi_read(traits::SPIx);
    }
};


class SPIDevice {
public:
  SPIDevice(int pinCS) : _pinCS(pinCS) {}

  void select() {
    //digitalWrite(_pinCS, LOW);
    HAL_GPIO_WritePin(TRX_nCS_GPIO_Port, TRX_nCS_Pin, GPIO_PIN_RESET);
    //SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  }

  void write(uint8_t x) {
    //SPI.transfer(x);
    HAL_SPI_Transmit(&hspi1, &x, 1, 100);
  }

  uint8_t read() {
    uint8_t x;
    HAL_SPI_Receive(&hspi1, &x, 1, 100);
    return x;
    //return SPI.transfer(0xFF);
  }

  void release() {
    //SPI.endTransaction();
    //digitalWrite(_pinCS, HIGH);
    HAL_GPIO_WritePin(TRX_nCS_GPIO_Port, TRX_nCS_Pin, GPIO_PIN_SET);
  }

private:
  int _pinCS;
};
