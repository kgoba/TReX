#include <stdint.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/nvic.h>

#include "os.hh"

void setup();
void loop();

void systick_setup(int freq)
{
    systick_set_reload(rcc_ahb_frequency / freq);
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    systick_counter_enable();
    systick_interrupt_enable();
}

void clock_setup()
{
    //rcc_clock_setup_hse_3v3(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_48MHZ]);
    rcc_clock_setup_in_hsi_out_16mhz();
}

int main(void)
{
    clock_setup();
    systick_setup(1000);

    setup();
    
    while(1)
    {
        loop();
    }
}