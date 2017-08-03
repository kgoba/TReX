#pragma once

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/spi.h>

#include "rcc.h"

template<uint32_t port, uint32_t pin>
class IOPin : public RCC<port> {
protected:
    static void set() {
        gpio_set(port, pin);
    }
    static void clear() {
        gpio_clear(port, pin);
    }
    static uint16_t read() {
        return gpio_get(port, pin);
    }
#if defined (STM32F1) 
    enum Speed { 
        LowSpeed = GPIO_MODE_OUTPUT_2_MHZ, 
        MidSpeed = GPIO_MODE_OUTPUT_10_MHZ, 
        HighSpeed = GPIO_MODE_OUTPUT_50_MHZ 
    };
    static void setupOutput(Speed speed = LowSpeed) {
        RCC<port>::enableClock();
        gpio_set_mode(port, (uint8_t)speed, GPIO_CNF_OUTPUT_PUSHPULL, pin);
    }
    static void setupInput() {
        RCC<port>::enableClock();
        gpio_set_mode(port, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, pin);
    }
    static void setupAlternate(uint8_t mode, uint8_t cnf) {
        RCC<port>::enableClock();
        gpio_set_mode(port, mode, cnf, pin);
    }
#else
    enum Speed { 
        LowSpeed = GPIO_OSPEED_2MHZ, 
        MidSpeed = GPIO_OSPEED_25MHZ, 
        HighSpeed = GPIO_OSPEED_100MHZ 
    };
    static void setupOutput(Speed speed = LowSpeed) {
        RCC<port>::enableClock();
        gpio_mode_setup(port, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, pin);
        gpio_set_output_options(port, GPIO_OTYPE_PP, (uint8_t)speed, pin);
    }
    static void setupInput() {
        RCC<port>::enableClock();
        gpio_mode_setup(port, GPIO_MODE_INPUT, GPIO_PUPD_NONE, pin);
    }
    static void setupAlternate(uint8_t af) {
        RCC<port>::enableClock();
        gpio_mode_setup(port, GPIO_MODE_AF, GPIO_PUPD_NONE, pin);
        gpio_set_output_options(port, GPIO_OTYPE_PP, (uint8_t)HighSpeed, pin);
        gpio_set_af(port, af, pin);
    }
#endif
};

class NC {};

typedef IOPin<GPIOA, GPIO0> PA_0;
typedef IOPin<GPIOA, GPIO1> PA_1;
typedef IOPin<GPIOA, GPIO2> PA_2;
typedef IOPin<GPIOA, GPIO3> PA_3;
typedef IOPin<GPIOA, GPIO4> PA_4;
typedef IOPin<GPIOA, GPIO5> PA_5;
typedef IOPin<GPIOA, GPIO6> PA_6;
typedef IOPin<GPIOA, GPIO7> PA_7;
typedef IOPin<GPIOA, GPIO8> PA_8;
typedef IOPin<GPIOA, GPIO9> PA_9;
typedef IOPin<GPIOA, GPIO10> PA_10;
typedef IOPin<GPIOA, GPIO11> PA_11;
typedef IOPin<GPIOA, GPIO12> PA_12;
typedef IOPin<GPIOA, GPIO13> PA_13;
typedef IOPin<GPIOA, GPIO14> PA_14;
typedef IOPin<GPIOA, GPIO15> PA_15;

typedef IOPin<GPIOB, GPIO0> PB_0;
typedef IOPin<GPIOB, GPIO1> PB_1;
typedef IOPin<GPIOB, GPIO2> PB_2;
typedef IOPin<GPIOB, GPIO3> PB_3;
typedef IOPin<GPIOB, GPIO4> PB_4;
typedef IOPin<GPIOB, GPIO5> PB_5;
typedef IOPin<GPIOB, GPIO6> PB_6;
typedef IOPin<GPIOB, GPIO7> PB_7;
typedef IOPin<GPIOB, GPIO8> PB_8;
typedef IOPin<GPIOB, GPIO9> PB_9;
typedef IOPin<GPIOB, GPIO10> PB_10;
typedef IOPin<GPIOB, GPIO11> PB_11;
typedef IOPin<GPIOB, GPIO12> PB_12;
typedef IOPin<GPIOB, GPIO13> PB_13;
typedef IOPin<GPIOB, GPIO14> PB_14;
typedef IOPin<GPIOB, GPIO15> PB_15;

typedef IOPin<GPIOC, GPIO0> PC_0;
typedef IOPin<GPIOC, GPIO1> PC_1;
typedef IOPin<GPIOC, GPIO2> PC_2;
typedef IOPin<GPIOC, GPIO3> PC_3;
typedef IOPin<GPIOC, GPIO4> PC_4;
typedef IOPin<GPIOC, GPIO5> PC_5;
typedef IOPin<GPIOC, GPIO6> PC_6;
typedef IOPin<GPIOC, GPIO7> PC_7;
typedef IOPin<GPIOC, GPIO8> PC_8;
typedef IOPin<GPIOC, GPIO9> PC_9;
typedef IOPin<GPIOC, GPIO10> PC_10;
typedef IOPin<GPIOC, GPIO11> PC_11;
typedef IOPin<GPIOC, GPIO12> PC_12;
typedef IOPin<GPIOC, GPIO13> PC_13;
typedef IOPin<GPIOC, GPIO14> PC_14;
typedef IOPin<GPIOC, GPIO15> PC_15;


// DigitalOut

template<typename Pin>
class DigitalOut : public Pin {
public:
    static void begin() {
        Pin::setupOutput();
    }
    static void set() {
        Pin::set();
    }
    static void clear() {
        Pin::clear();
    }
    static void write(uint16_t value) {
        if (value) set(); else clear();
    }

    uint16_t operator = (uint16_t value) {
        write(value);
        return value;
    }
};

// Specialization for unconnected (NC) pin
template<>
class DigitalOut<NC> {
public:
    static void begin() {}
    static void set() {}
    static void clear() {}
    static void write(uint16_t value) {}
};


// DigitalIn

template<typename Pin>
class DigitalIn : public Pin {
public:
    static void begin() {
        Pin::setupInput();
    }
    static uint16_t read() {
        return Pin::read();
    }
};

// Specialization for unconnected (NC) pin
template<>
class DigitalIn<NC> {
public:
    static void begin() {}
    static uint16_t read() { return 0; }
};


// DigitalAF

template<typename Pin, uint32_t periph>
class DigitalAF : public Pin {
public:
    static void begin() {
        Pin::setupAlternate(af);
    }
    
protected:
    const static uint8_t af;
};

template<> const uint8_t DigitalAF<PA_0, USART2>::af = GPIO_AF1;
template<> const uint8_t DigitalAF<PA_1, USART2>::af = GPIO_AF1;
template<> const uint8_t DigitalAF<PA_2, USART2>::af = GPIO_AF1;
template<> const uint8_t DigitalAF<PA_3, USART2>::af = GPIO_AF1;
template<> const uint8_t DigitalAF<PA_4, USART2>::af = GPIO_AF1;

template<> const uint8_t DigitalAF<PA_8,  USART1>::af = GPIO_AF1;
template<> const uint8_t DigitalAF<PA_9,  USART1>::af = GPIO_AF1;
template<> const uint8_t DigitalAF<PA_10, USART1>::af = GPIO_AF1;
template<> const uint8_t DigitalAF<PA_11, USART1>::af = GPIO_AF1;
template<> const uint8_t DigitalAF<PA_12, USART1>::af = GPIO_AF1;

template<> const uint8_t DigitalAF<PA_4, SPI1>::af   = GPIO_AF0;
template<> const uint8_t DigitalAF<PA_5, SPI1>::af   = GPIO_AF0;
template<> const uint8_t DigitalAF<PA_6, SPI1>::af   = GPIO_AF0;
template<> const uint8_t DigitalAF<PA_7, SPI1>::af   = GPIO_AF0;
template<> const uint8_t DigitalAF<PA_15, SPI1>::af   = GPIO_AF0;

template<> const uint8_t DigitalAF<PB_3, SPI1>::af   = GPIO_AF0;
template<> const uint8_t DigitalAF<PB_4, SPI1>::af   = GPIO_AF0;
template<> const uint8_t DigitalAF<PB_5, SPI1>::af   = GPIO_AF0;

template<> const uint8_t DigitalAF<PB_6, I2C1>::af   = GPIO_AF1;
template<> const uint8_t DigitalAF<PB_7, I2C1>::af   = GPIO_AF1;

// Specialization for unconnected (NC) pin
template<uint32_t periph> 
class DigitalAF<NC, periph> {
public:
    static void begin() {}
};
