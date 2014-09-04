/*
 * main.c
 *
 *  Created on: 4 Sep 2014
 *      Author: Marcin Twardak
 */

#include <avr/io.h>

int main()
{
	DDRB = 0xFF;
	PORTB = 0xFF;

	while(1) {}

	return 0;
}
