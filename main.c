/*
 * main.c
 *
 *  Created on: 4 Sep 2014
 *      Author: Marcin Twardak
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define LED_BLUE 0b01000000
#define LED_BAT_GREEN1 0b00010000
#define LED_BAT_GREEN2 0b00001000
#define LED_BAT_YELLOW 0b00000100
#define LED_BAT_RED	   0b00000010
#define LED_BLUETOOTH  0b00000001

void led_blinking_init();
void ADC_init();
void USART_init();
void USART_Transmit(unsigned char data);
unsigned char USART_Receive();

// ENGINES
void turn_off_all_engines();

uint8_t safety_variable; // turn off engines when bluetooth connection is broken

int main()
{
	safety_variable = 0;
	led_blinking_init();
	ADC_init();
	USART_init();
	sei();

	while(1)
	{
	}

	return 0;
}

void turn_off_all_engines()
{
	PORTB &= ~(LED_BLUETOOTH);
	return;
}

void led_blinking_init()
{
	DDRB |= LED_BLUE;

	// Timer 0 Configuration
	TCCR0 |= 0b101; // prescaler 1024
	TIMSK |= 1; // interrupt
}

void ADC_init()
{
	ADMUX = 0b11100101; // internal 2.56, ADC5, left adjustment
	ADCSRA = 0b11000111; // single mode, without interrupt, CLK / 128

	DDRB |= LED_BAT_GREEN1|LED_BAT_GREEN2|LED_BAT_RED|LED_BAT_YELLOW;
}

void USART_init()
{
	DDRD = 0b10;
	PORTD = 0;

	DDRB |= LED_BLUETOOTH;
	UBRRL = 12;
	UBRRH = 0;
	UCSRA = (1 << U2X);
	// Enable Receiver and Transmitter
	UCSRB = (1 << RXCIE) | (1 << RXEN) | (1 << TXEN);
	// 8 data, 1 stop bit
	UCSRC = (1 << URSEL) | (3 << UCSZ0);
}

void USART_Transmit( unsigned char data )
{
	/* Wait for empty transmit buffer */
	while ( !( UCSRA & (1<<UDRE)) )
	;
	/* Put data into buffer, sends the data */
	UDR = data;
}

unsigned char USART_Receive( void )
{
	/* Wait for data to be received */
	while ( !(UCSRA & (1<<RXC)) );
	/* Get and return received data from buffer */
	return UDR;
}

ISR (TIMER0_OVF_vect) // diode blinking + ADC read, safety_variable
{
	if (safety_variable >= 3)
			turn_off_all_engines();
	safety_variable++;

	PORTB ^= LED_BLUE;

	if (ADCH >= 118) // 13 V
		PORTB |= LED_BAT_GREEN1|LED_BAT_GREEN2|LED_BAT_RED|LED_BAT_YELLOW;
	else if (ADCH >= 113) // 12,5 V
	{
		PORTB |= LED_BAT_GREEN2 | LED_BAT_YELLOW | LED_BAT_RED;
		PORTB &= ~LED_BAT_GREEN1;
	}
	else if (ADCH >= 109) // 12 V
	{
		PORTB |= LED_BAT_YELLOW | LED_BAT_RED;
		PORTB &= ~(LED_BAT_GREEN1 | LED_BAT_GREEN2);
	}
	else
	{
		PORTB |= LED_BAT_RED;
		PORTB &= ~(LED_BAT_GREEN1 | LED_BAT_GREEN2 | LED_BAT_YELLOW);
	}

	ADCSRA |= 0b01000000;
}

ISR (USART_RXC_vect)
{
	PORTB |= LED_BLUETOOTH;
	unsigned char data = USART_Receive();
}
