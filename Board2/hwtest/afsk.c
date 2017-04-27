#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>

#include <stdint.h>

#define RCC_MOD     RCC_GPIOB
#define PORT_MOD    GPIOB
#define PIN_MOD     GPIO1
#define AF_MOD      GPIO_AF0

#define RCC_PWM     RCC_TIM14
#define TIM_PWM     TIM14
#define IRQ_PWM     NVIC_TIM14_IRQ

#define SYMBOL_RATE 1200
#define FREQ_MARK   1200
#define FREQ_SPACE  2200

/* Defines oversampling - timer overflows (PWM cycles) per symbol */
#define SAMPLES_PER_SYMBOL  16

#define PWM_PRESCALER       5
#define PWM_PERIOD          250

#define WAVETABLE_SIZE      192
#define PHASE_MAX           (WAVETABLE_SIZE << 8)

#define PHASE_DELTA_MARK    (PHASE_MAX * FREQ_MARK / SYMBOL_RATE / SAMPLES_PER_SYMBOL)
#define PHASE_DELTA_SPACE   (PHASE_MAX * FREQ_SPACE / SYMBOL_RATE / SAMPLES_PER_SYMBOL) 

static const uint8_t waveTable[WAVETABLE_SIZE] = {
    125, 129, 133, 137, 141, 145, 149, 153, 157, 161, 165, 169, 173, 177, 
    180, 184, 188, 191, 194, 198, 201, 204, 207, 210, 213, 216, 219, 222, 
    224, 227, 229, 231, 233, 235, 237, 239, 240, 242, 243, 245, 246, 247, 
    248, 248, 249, 249, 250, 250, 250, 250, 250, 249, 249, 248, 248, 247, 
    246, 245, 243, 242, 240, 239, 237, 235, 233, 231, 229, 227, 224, 222, 
    219, 216, 213, 210, 207, 204, 201, 198, 194, 191, 188, 184, 180, 177, 
    173, 169, 165, 161, 157, 153, 149, 145, 141, 137, 133, 129, 125, 121, 
    117, 113, 109, 105, 101, 97, 93, 89, 85, 81, 77, 73, 70, 66, 63, 59, 
    56, 52, 49, 46, 43, 40, 37, 34, 31, 28, 26, 23, 21, 19, 17, 15, 13, 
    11, 10, 8, 7, 5, 4, 3, 2, 2, 1, 1, 0, 0, 0, 0, 0, 1, 1, 2, 2, 3, 4, 
    5, 7, 8, 10, 11, 13, 15, 17, 19, 21, 23, 26, 28, 31, 34, 37, 40, 43, 
    46, 49, 52, 56, 59, 62, 66, 70, 73, 77, 81, 85, 89, 93, 97, 101, 105, 
    109, 113, 117, 121
};

static uint8_t *txBuffer;
static uint8_t  txBitMask;
static uint16_t txBitsToSend;
static uint16_t txSampleInSymbol;
static uint32_t txPhase;
static uint32_t txPhaseDelta;

void afsk_send(uint8_t *message, uint8_t lengthInBits)
{
    txBuffer = message;
    txBitsToSend = lengthInBits;

    txPhase = 0;
    txPhaseDelta = PHASE_DELTA_MARK;

    txBitMask = 1;
    txSampleInSymbol = 0;
    
    timer_enable_oc_output(TIM_PWM, TIM_OC1);
    timer_enable_counter(TIM_PWM);
}

void afsk_stop()
{
    timer_disable_counter(TIM_PWM);
    timer_disable_oc_output(TIM_PWM, TIM_OC1);
}

void afsk_setup()
{
    txBitsToSend = 0;

    /* Configure modulation pin (GPIO1 on SI446x) */
    rcc_periph_clock_enable(RCC_MOD);

    /* Setup pin for TIM14_CH1 function */
    gpio_mode_setup(PORT_MOD, GPIO_MODE_AF, GPIO_PUPD_NONE, PIN_MOD);
    gpio_set_output_options(PORT_MOD, GPIO_OTYPE_PP, GPIO_OSPEED_HIGH, PIN_MOD);
    gpio_set_af(PORT_MOD, AF_MOD, PIN_MOD);

    // Reset and configure timer
    rcc_periph_clock_enable(RCC_PWM);
    timer_reset(TIM_PWM);
    timer_set_mode(TIM_PWM, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_CENTER_1, TIM_CR1_DIR_UP);
    //timer_set_mode(TIM_PWM, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_set_prescaler(TIM_PWM, PWM_PRESCALER);
    timer_set_period(TIM_PWM, PWM_PERIOD - 1);
    timer_enable_break_main_output(TIM_PWM);

    timer_set_oc_mode(TIM_PWM, TIM_OC1, TIM_OCM_PWM1);
    timer_set_oc_value(TIM_PWM, TIM_OC1, PWM_PERIOD);

    timer_enable_irq(TIM_PWM, TIM_DIER_CC1IE);
    nvic_enable_irq(IRQ_PWM);
}

void tim14_isr(void)
{
    if (timer_get_flag(TIM_PWM, TIM_SR_CC1IF)) {
        timer_clear_flag(TIM_PWM, TIM_SR_CC1IF);

        if (txBitsToSend == 0) {
            afsk_stop();
            return;
        }

        timer_set_oc_value(TIM_PWM, TIM_OC1, waveTable[txPhase >> 8]);

        if (txSampleInSymbol == 0) {
            /* Load new symbol (bit) to transmit */

            if (*txBuffer & txBitMask) {
                //txPhaseDelta = PHASE_DELTA_MARK;   /* bit 1 */
            }
            else {
                //txPhaseDelta = PHASE_DELTA_SPACE;  /* bit 0 */
                /* Toggle the AFSK frequency */
                txPhaseDelta ^= (PHASE_DELTA_MARK ^ PHASE_DELTA_SPACE);
            }
            
            if (txBitMask == 0x80) {
                /* Whole byte was processed, move to the next one */
                txBitMask = 1;
                txBuffer++;
            } else {
                txBitMask <<= 1;
            }

            txBitsToSend--;
        }
        
        txSampleInSymbol++;
        if (txSampleInSymbol >= SAMPLES_PER_SYMBOL) {
            txSampleInSymbol = 0;
        }
        
        txPhase = (txPhase + txPhaseDelta) % PHASE_MAX;
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
