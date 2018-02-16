#include <stdint.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/cm3/systick.h>

#include "radio.h"

#define LED_ON      gpio_clear(GPIOC, GPIO13)
#define LED_OFF     gpio_set(GPIOC, GPIO13)

void delay(uint32_t delay);

void systick_setup(int freq)
{
    systick_set_reload(rcc_ahb_frequency / freq);
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    systick_counter_enable();
    systick_interrupt_enable();
}

void clock_setup()
{
    rcc_clock_setup_in_hse_8mhz_out_24mhz();
    //rcc_clock_setup_in_hse_8mhz_out_72mhz();
}

void gpio_setup()
{
    /* for LED */
    rcc_periph_clock_enable(RCC_GPIOC);
    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
}

void spi_setup()
{
    /* Enable all clocks needed for SPI1 */
    rcc_periph_clock_enable(RCC_AFIO);
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_SPI1);
    
    /* Configure GPIOs: SCK=PA5, MISO=PA6 and MOSI=PA7 */
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO6);
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO5 | GPIO7);
    
    spi_init_master(SPI1, SPI_CR1_BAUDRATE_FPCLK_DIV_256, SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
                    SPI_CR1_CPHA_CLK_TRANSITION_1, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);
    
    //spi_enable_ss_output(SPI1);
    //spi_disable_software_slave_management(SPI1);
    spi_disable_ss_output(SPI1);                // SSOE = 0
    spi_enable_software_slave_management(SPI1); // SSM = 1
    spi_set_nss_high(SPI1);                     // SSI = 1

    /* Enable SPI1 periph. */
    spi_enable(SPI1);
}

void si446x_setup()
{
    rcc_periph_clock_enable(RCC_GPIOA);
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO4);
    gpio_set(GPIOA, GPIO4);
    //gpio_set(GPIOA, GPIO3);    
}

void si446x_wakeup()
{
    //gpio_clear(GPIOA, GPIO3);
}

void si446x_shutdown()
{
    //gpio_set(GPIOA, GPIO3);
}

void si446x_select()
{
    gpio_clear(GPIOA, GPIO4);
    delay(1);
}

void si446x_release()
{
    delay(1);
    gpio_set(GPIOA, GPIO4);
}

uint8_t si446x_xfer(uint8_t out)
{
    spi_send(SPI1, out);
    return spi_read(SPI1);
}

uint8_t si446x_configure()
{
    uint8_t rc = 0;
    if (!rc) rc = si446x_setfrequency(434.250 * 1000000, 5.000 * 1000);
    if (!rc) rc = si446x_setmodulation(EZR_MOD_SOURCE_PRN | EZR_MOD_TYPE_2FSK);
    if (!rc) rc = si446x_setNCOModulo(0, SI446x_VCXO_FREQ / 10);
    if (!rc) rc = si446x_setdatarate(1000);
    if (!rc) rc = si446x_setpower(0x10);
    return rc;
}

void basicSPITest()
{
    while (1) {        
        uint8_t reply;
        
        si446x_select();
        delay(1);
        spi_send8(SPI1, (uint8_t)0x44);

        spi_send8(SPI1, (uint8_t)0xFF);
        reply = spi_read8(SPI1);
        delay(1);
        si446x_release();

        LED_ON;
        delay((reply == 0xFF) ? 800 : 200);
        LED_OFF;
        delay((reply == 0xFF) ? 200 : 800);
    }
}

void partNumberTest()
{
    while (1) {        
        char cmd[] = {0x01};
        uint8_t rc = si446x_sendcmd(1, 9, cmd);
        //LED_ON; delay(rc ? 900 : 100); LED_OFF; delay(rc ? 100 : 900);
    }
}

int main()
{
    clock_setup();
    systick_setup(1000);
    gpio_setup();    
    spi_setup();
    si446x_setup();
    
    LED_ON;
    si446x_shutdown();
    delay(200);
    LED_OFF;
    si446x_wakeup();
    delay(200);

    uint8_t rc;
    
    rc = si446x_boot();    
    if (rc) {
        while (1) {
            LED_ON; delay(100); LED_OFF; delay(100);
        }        
    }

    partNumberTest();

    rc = si446x_configure();
    if (rc) {
        while (1) {
            LED_ON; delay(200); LED_OFF; delay(100);
        }        
    }
        
    si446x_tune();
    delay(100);
    
    while (1) {
        LED_ON;
        si446x_tx_on();        
        delay(200);
        
        LED_OFF;
        si446x_tx_off();
        delay(2000);        
    }
}

volatile uint32_t system_millis;

void delay(uint32_t delay)
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
