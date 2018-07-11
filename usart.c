#include "usart.h"

void usart_init( unsigned int ubrr) {
	// Set baud rate
	UBRR0H = (unsigned char)(ubrr>>8);
	UBRR0L = (unsigned char)ubrr;

	// Enable transmitter
	UCSR0B = (1<<TXEN0);

	// Set frame format: 8data
	UCSR0C = (1<<USBS0)|(3<<UCSZ00);
}

void usart_transmit( unsigned char data ) {
	// Wait for empty transmit buffer
	while ( !( UCSR0A & (1<<UDRE0)) );

	// Put data into buffer, sends the data
	UDR0 = data;
}

void usart_transmit_double(double value){
	unsigned char result[ 12];
	dtostrf(value, 7, 4, result);

	int i = 0;
	for (i = 0; i < sizeof(result); i++){
		usart_transmit(result[i]);
	}
}

void usart_transmit_int(int value){
	unsigned char result[ sizeof(value) ];
	itoa(value, result);

	int i = 0;
	for (i = 0; i < sizeof(result); i++){
		usart_transmit(result[i]);
	}
}

void usart_transmit_string(char* value){
	int i = 0;
	for (i = 0; i < sizeof(value); i++){
		usart_transmit(value[i]);
	}
}
