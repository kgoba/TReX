#include <stdint.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>


/** GPIO port templatized class 
 */
template<uint32_t port>
class GPIOPort {
public:
    static void enableClock() {
    }
};

template<>
void GPIOPort<GPIOA>::enableClock() {
    rcc_periph_clock_enable(RCC_GPIOA);
}

template<>
void GPIOPort<GPIOB>::enableClock() {
    rcc_periph_clock_enable(RCC_GPIOB);
}

template<>
void GPIOPort<GPIOC>::enableClock() {
    rcc_periph_clock_enable(RCC_GPIOC);
}


enum AlternateFunction {
    NONE_AF,
    
    EVENTOUT_AF,
    MCO_AF,
    IR_OUT_AF,
    SWDIO_AF,
    SWCLK_AF,

    USART1_RX_AF,
    USART1_TX_AF,
    USART1_RTS_AF,
    USART1_CTS_AF,
    USART1_CK_AF,
    USART2_RX_AF,
    USART2_TX_AF,
    USART2_RTS_AF,
    USART2_CTS_AF,
    USART2_CK_AF,
    USART3_RX_AF,
    USART3_TX_AF,
    USART3_RTS_AF,
    USART3_CTS_AF,
    USART3_CK_AF,
    USART4_RX_AF,
    USART4_TX_AF,
    USART4_RTS_AF,
    USART4_CTS_AF,
    USART4_CK_AF,

    SPI1_MOSI_AF,
    SPI1_MISO_AF,
    SPI1_SCK_AF,
    SPI1_NSS_AF,
    SPI2_MOSI_AF,
    SPI2_MISO_AF,
    SPI2_SCK_AF,
    SPI2_NSS_AF,

    I2C1_SDA_AF,
    I2C1_SCL_AF,
    I2C1_SMBA_AF,
    I2C2_SDA_AF,
    I2C2_SCL_AF,

    TIM1_CH1_AF,
    TIM1_CH2_AF,
    TIM1_CH3_AF,
    TIM1_CH4_AF,
    TIM1_CH1N_AF,
    TIM1_CH2N_AF,
    TIM1_CH3N_AF,
    TIM1_BKIN_AF,
    TIM1_ETR_AF,

    TIM3_CH1_AF,
    TIM3_CH2_AF,
    TIM3_CH3_AF,
    TIM3_CH4_AF,
    TIM3_ETR_AF,
        
    TIM15_CH1_AF,
    TIM15_CH2_AF,
    TIM15_CH1N_AF,
    TIM15_BKIN_AF,

    TIM16_CH1_AF,
    TIM16_CH1N_AF,
    TIM16_BKIN_AF,
};


/** GPIO pin templatized class 
 */
template<uint32_t port, uint32_t pin>
class GPIOPin {
public:
    static void enableClock() {
        GPIOPort<port>::enableClock();
    }

    static void setupInput(uint8_t pullup = GPIO_PUPD_NONE) {
        gpio_mode_setup(port, GPIO_MODE_INPUT, pullup, pin);
    }
    
    static void setupOutput(
        uint8_t pullup = GPIO_PUPD_NONE,
        uint8_t drive  = GPIO_OTYPE_PP,
        uint8_t speed  = GPIO_OSPEED_HIGH) 
    {       
        gpio_mode_setup(port, GPIO_MODE_OUTPUT, pullup, pin);
        gpio_set_output_options(port, drive, speed, pin);
    }
    
    static void setupAlternate(uint8_t af,
        uint8_t pullup = GPIO_PUPD_NONE,
        uint8_t drive  = GPIO_OTYPE_PP,
        uint8_t speed  = GPIO_OSPEED_HIGH)
    {
        gpio_mode_setup(port, GPIO_MODE_AF, pullup, pin);
        gpio_set_output_options(port, drive, speed, pin);
        gpio_set_af(port, af, pin);
    }
    
    static void set() {
        gpio_set(port, pin);
    }
    
    static void clear() {
        gpio_clear(port, pin);
    }
    
    static void toggle() {
        gpio_toggle(port, pin);
    }
    
    static uint16_t read() {
        return gpio_get(port, pin);
    }
};


typedef GPIOPin<GPIOA, GPIO0>   PA0;
typedef GPIOPin<GPIOA, GPIO1>   PA1;
typedef GPIOPin<GPIOA, GPIO2>   PA2;
typedef GPIOPin<GPIOA, GPIO3>   PA3;
typedef GPIOPin<GPIOA, GPIO4>   PA4;
typedef GPIOPin<GPIOA, GPIO5>   PA5;
typedef GPIOPin<GPIOA, GPIO6>   PA6;
typedef GPIOPin<GPIOA, GPIO7>   PA7;
typedef GPIOPin<GPIOA, GPIO8>   PA8;
typedef GPIOPin<GPIOA, GPIO9>   PA9;
typedef GPIOPin<GPIOA, GPIO10>  PA10;
typedef GPIOPin<GPIOA, GPIO11>  PA11;
typedef GPIOPin<GPIOA, GPIO12>  PA12;
typedef GPIOPin<GPIOA, GPIO13>  PA13;
typedef GPIOPin<GPIOA, GPIO14>  PA14;
typedef GPIOPin<GPIOA, GPIO15>  PA15;
typedef GPIOPin<GPIOB, GPIO0>   PB0;
typedef GPIOPin<GPIOB, GPIO1>   PB1;
typedef GPIOPin<GPIOB, GPIO2>   PB2;
typedef GPIOPin<GPIOB, GPIO3>   PB3;
typedef GPIOPin<GPIOB, GPIO4>   PB4;
typedef GPIOPin<GPIOB, GPIO5>   PB5;
typedef GPIOPin<GPIOB, GPIO6>   PB6;
typedef GPIOPin<GPIOB, GPIO7>   PB7;
typedef GPIOPin<GPIOB, GPIO8>   PB8;
typedef GPIOPin<GPIOB, GPIO9>   PB9;
typedef GPIOPin<GPIOB, GPIO10>  PB10;
typedef GPIOPin<GPIOB, GPIO11>  PB11;
typedef GPIOPin<GPIOB, GPIO12>  PB12;
typedef GPIOPin<GPIOB, GPIO13>  PB13;
typedef GPIOPin<GPIOB, GPIO14>  PB14;
typedef GPIOPin<GPIOB, GPIO15>  PB15;
typedef GPIOPin<GPIOC, GPIO0>   PC0;
typedef GPIOPin<GPIOC, GPIO1>   PC1;
typedef GPIOPin<GPIOC, GPIO2>   PC2;
typedef GPIOPin<GPIOC, GPIO3>   PC3;
typedef GPIOPin<GPIOC, GPIO4>   PC4;
typedef GPIOPin<GPIOC, GPIO5>   PC5;
typedef GPIOPin<GPIOC, GPIO6>   PC6;
typedef GPIOPin<GPIOC, GPIO7>   PC7;
typedef GPIOPin<GPIOC, GPIO8>   PC8;
typedef GPIOPin<GPIOC, GPIO9>   PC9;
typedef GPIOPin<GPIOC, GPIO10>  PC10;
typedef GPIOPin<GPIOC, GPIO11>  PC11;
typedef GPIOPin<GPIOC, GPIO12>  PC12;
typedef GPIOPin<GPIOC, GPIO13>  PC13;
typedef GPIOPin<GPIOC, GPIO14>  PC14;
typedef GPIOPin<GPIOC, GPIO15>  PC15;


/** Specialized (output) GPIO pin class 
 */
template<typename Pin>
class DigitalOutput : public Pin {
public:
    static void setup() {
        Pin::setupOutput();
    }
};


/** Specialized (input) GPIO pin class 
 */
template<typename Pin>
class DigitalInput : public Pin {
public:
    static void setup() {
        Pin::setupInput();
    }
};

template<typename Pin, AlternateFunction af>
class AlternateGPIO : public Pin {
public:
    static void setup() {
    }
};

template<> void AlternateGPIO<PA9, USART1_TX_AF>::setup() {
    setupAlternate(GPIO_AF1);
}

template<> void AlternateGPIO<PA10, USART1_RX_AF>::setup() {
    setupAlternate(GPIO_AF1);
}

template<> void AlternateGPIO<PA2, USART2_TX_AF>::setup() {
    setupAlternate(GPIO_AF1);
}

template<> void AlternateGPIO<PA3, USART2_RX_AF>::setup() {
    setupAlternate(GPIO_AF1);
}
