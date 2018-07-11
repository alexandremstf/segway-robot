#ifndef USART_H_
#define USART_H_

#include <avr/io.h>

void usart_init( unsigned int);
void usart_transmit( unsigned char);
void usart_transmit_double( double );
void usart_transmit_string(char*);
void usart_transmit_int(int);

#endif
