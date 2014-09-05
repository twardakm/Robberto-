/*
 * main.c
 *
 *  Created on: 4 Sep 2014
 *      Author: Marcin Twardak
 */

#include <avr/io.h>
#include <avr/interrupt.h>

#define LED_BLUE 0b01000000
#define LED_BAT_GREEN1 0b00010000
#define LED_BAT_GREEN2 0b00001000
#define LED_BAT_YELLOW 0b00000100
#define LED_BAT_RED	   0b00000010
#define LED_BLUETOOTH  0b00000001

void led_blinking_init();
void ADC_init();

int main()
{
	led_blinking_init();
	ADC_init();
	sei();

	while(1) {}

	return 0;
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

ISR (TIMER0_OVF_vect) // diode blinking + ADC read
{
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
