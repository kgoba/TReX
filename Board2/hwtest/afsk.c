#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>

#include <stdint.h>

#define RCC_MOD     RCC_GPIOB
#define PORT_MOD    GPIOB
#define PIN_MOD     GPIO1
#define AF_MOD      GPIO_AF0

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
