#include "usart.hh"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>

#include <errno.h>

#define FD_DEBUG    1

void usart_init(void)
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

	/* Enable USART Receive interrupt. */
	usart_enable_rx_interrupt(USART1);

	/* Finally enable the USART. */
	usart_enable(USART1);

	/* Enable the USART1 interrupt. */
    nvic_enable_irq(NVIC_USART1_IRQ);
}

Queue<uint8_t, 64> dbgTXQueue;

extern "C" {
    int _write(int file, char *ptr, int len)
    {
    	int ret = 0;

    	if (file == FD_DEBUG) {
            while (len) {
        		if (!dbgTXQueue.push((uint8_t)*ptr)) {
                    ret = -ret;
                    break;    		    
        		}
                ret++;
                ptr++;
                len--;
            }
    		if (dbgTXQueue.count() > 0)
                usart_enable_tx_interrupt(USART1);

    		return ret;
    	}

    	errno = EIO;
    	return -1;    
    }    
}

void usart1_isr(void) 
{
	/* Check if we were called because of RXNE. */
    if (usart_get_interrupt_source(USART1, USART_ISR_RXNE)) {
		/* Indicate that we got data. */
		gpio_toggle(GPIOA, GPIO12);

		/* Retrieve the data from the peripheral. */
		uint8_t data = usart_recv(USART1);
	}

	/* Check if we were called because of TXE. */
    if (usart_get_interrupt_source(USART1, USART_ISR_TXE)) {
        uint8_t data;
        
        if (dbgTXQueue.pop(data)) {
    		/* Put data into the transmit register. */
    		usart_send(USART1, data);
        }

		if (dbgTXQueue.count() == 0) {
            /* Disable the TXE interrupt as we don't need it anymore. */
            usart_disable_tx_interrupt(USART1);		    
		}
	}
};