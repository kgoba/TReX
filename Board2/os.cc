#include "os.hh"
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>

void delay_us(uint32_t microSeconds)
{
    
}


volatile uint32_t system_millis;


void delay_ms(uint32_t delay)
{
    uint32_t wake = system_millis + delay;
    while (wake > system_millis)
    {
        // do nothing
    }
}


void sys_tick_handler(void)
{
    system_millis++;
}


uint32_t get_millis_elapsed(void)
{
    return system_millis;
}
