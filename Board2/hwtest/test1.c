#include <stdint.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/nvic.h>

#include <stdio.h>
#include <errno.h>

#include <silabs/radio.h>
#include "radio_hal.h"

#define FD_DEBUG    1

#define TCXO_FREQ   26000000ul

#define LED_RCC     RCC_GPIOA
#define LED_GPIO    GPIOA
#define LED_PIN     GPIO12

#define LED_ON      gpio_set(LED_GPIO, LED_PIN)
#define LED_OFF     gpio_clear(LED_GPIO, LED_PIN)

#define RCC_MOD     RCC_GPIOB
#define PORT_MOD    GPIOB
#define PIN_MOD     GPIO1
#define AF_MOD      GPIO_AF0

void delay(uint32_t delay);

void systick_setup(int freq)
{
    systick_set_reload(rcc_ahb_frequency / freq);
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    systick_counter_enable();
    systick_interrupt_enable();
}

void usart_setup(void)
{
	/* Setup GPIO pins for USART1 transmit/receive. */
    rcc_periph_clock_enable(RCC_GPIOA);
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9);    // TX
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO10);   // RX
	gpio_set_af(GPIOA, GPIO_AF1, GPIO9);
	gpio_set_af(GPIOA, GPIO_AF1, GPIO10);

    rcc_periph_clock_enable(RCC_USART1);

	/* Setup USART parameters. */
	usart_set_baudrate(USART1, 9600);
	usart_set_databits(USART1, 8);
	usart_set_stopbits(USART1, USART_CR2_STOP_1_0BIT);
	usart_set_mode(USART1, USART_MODE_TX_RX);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

	/* Finally enable the USART. */
	usart_enable(USART1);
}

void clock_setup()
{
    //rcc_clock_setup_in_hsi_out_16mhz();
    rcc_clock_setup_in_hsi_out_24mhz();
}

void gpio_setup()
{
    /* setup LED */
    rcc_periph_clock_enable(LED_RCC);
    gpio_mode_setup(LED_GPIO, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_PIN);
}

void partNumberTest()
{
    while (1) {        
        char cmd[] = {0x01};
        uint8_t rc = si446x_sendcmd(1, 9, cmd);
        //LED_ON; delay(rc ? 900 : 100); LED_OFF; delay(rc ? 100 : 900);
    }
}

void prnTest() 
{
    uint8_t rc = 0;
    
    if (!rc) rc = si446x_setFrequency(144.250 * 1000000, 5.000 * 1000);
    if (!rc) rc = si446x_setModulation(EZR_MOD_SOURCE_PRN | EZR_MOD_TYPE_2FSK);
    if (!rc) rc = si446x_setDataRate(1000);
    if (!rc) rc = si446x_setPower(0x10);
    if (rc) {
        while (1) {
            LED_ON; delay(200); LED_OFF; delay(100);
        }        
    }
    
    si446x_tune();
    delay(100);
    
    while (1) {
        LED_ON;
        si446x_txOn();        
        delay(200);
        
        LED_OFF;
        si446x_txOff();
        //delay(2000);       
        
        for (uint8_t i = 0; i < 10; i++) {
            char cmd[] = {0x01};
            uint8_t rc = si446x_sendcmd(1, 9, cmd); 
            delay(200);
        }
    }
}

void afsk_setup()
{
    /* Configure modulation pin (GPIO1 on SI446x) */
    rcc_periph_clock_enable(RCC_MOD);

    /* Setup pin for TIM14_CH1 function */
    gpio_mode_setup(PORT_MOD, GPIO_MODE_AF, GPIO_PUPD_NONE, PIN_MOD);
    gpio_set_output_options(PORT_MOD, GPIO_OTYPE_PP, GPIO_OSPEED_HIGH, PIN_MOD);
    gpio_set_af(PORT_MOD, AF_MOD, PIN_MOD);

    // Reset and configure timer
    rcc_periph_clock_enable(RCC_TIM14);
    timer_reset(TIM14);
    timer_set_mode(TIM14, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_CENTER_1, TIM_CR1_DIR_UP);
    //timer_set_mode(TIM14, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_set_prescaler(TIM14, 24);
    timer_set_period(TIM14, 999);
    timer_enable_break_main_output(TIM14);

    timer_set_oc_mode(TIM14, TIM_OC1, TIM_OCM_PWM1);
    timer_set_oc_value(TIM14, TIM_OC1, 500);
    timer_enable_oc_output(TIM14, TIM_OC1);

    timer_enable_irq(TIM14, TIM_DIER_CC1IE);
    nvic_enable_irq(NVIC_TIM14_IRQ);

    timer_enable_counter(TIM14);
}

void tim14_isr(void)
{
    if (timer_get_flag(TIM14, TIM_SR_CC1IF)) {
        timer_clear_flag(TIM14, TIM_SR_CC1IF);
    }
}

void afsk_low()
{
    gpio_clear(PORT_MOD, PIN_MOD);
}

void afsk_high()
{
    gpio_set(PORT_MOD, PIN_MOD);
}

void afskTest() 
{
    uint8_t rc;
    
    if (!rc) rc = si446x_setFrequency(144.250 * 1000000, 0.500 * 1000);
    if (!rc) rc = si446x_setModulation(EZR_MOD_SOURCE_DIRECT_MODE | EZR_MOD_TX_MODE_ASYNC | EZR_MOD_TX_MODE_GPIO1 | EZR_MOD_TYPE_2FSK);
    if (!rc) rc = si446x_setDataRate(1200);
    if (!rc) rc = si446x_setPower(0x10);
    if (!rc) rc = si446x_setupGPIO1(EZR_GPIO_MODE_TX_DATA);
    if (rc) {
        while (1) {
            LED_ON; delay(200); LED_OFF; delay(100);
        }
    }
        
    si446x_tune();
    delay(100);

    afsk_setup();    
    while (1) {
        LED_ON;
        si446x_txOn();
        delay(20);

        //delay(200);
        for (uint32_t i = 0; i < 100; i++) {
            //afsk_low();
            delay(1);
            //afsk_high();
            delay(1);
        }

        LED_OFF;
        si446x_txOff();
        delay(2000);
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
    
    rc = si446x_boot(TCXO_FREQ);    
    if (rc) {
        while (1) {
            LED_ON; delay(100); LED_OFF; delay(100);
        }        
    }
    
    //partNumberTest();
    //prnTest();
    afskTest();
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

int _write(int file, char *ptr, int len)
{
	int ret = 0;

	if (file == FD_DEBUG) {
        while (len) {
    		usart_send_blocking(USART1, (uint8_t)*ptr);
            ret++;
            ptr++;
            len--;
        }
		return ret;
	}

	errno = EIO;
	return -1;    
}    
