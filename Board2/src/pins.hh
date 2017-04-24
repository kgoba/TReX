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


/** GPIO pin templatized class 
 */
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
    
    static void toggle() {
        gpio_toggle(port, pin);
    }
};

/** Specialized (output) GPIO pin class 
 */
template<uint32_t port, uint32_t pin>
class DigitalOutput : public GPIOPin<port, pin> {
public:
    static void enable() {
        GPIOPin<port, pin>::enableClock();
        GPIOPin<port, pin>::setupOutput();
    }
};