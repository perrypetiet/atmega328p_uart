#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "avr_uart.h"

// Simple echo program

int main(void)
{
	DDRB |= (1<<DDB5);
  PORTB &= ~(1 << PORTB5);
  cli();
  init_uart(57600, 8, PARITY_NONE, 1);
  //Set the global interrupt flag.
  sei();

	while (1)
	{
    uint8_t byte;
    if (uart_read_byte(&byte))
    {
      uart_transmit(&byte, 1);
    } 
	}
}

