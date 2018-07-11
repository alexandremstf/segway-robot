#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <util/twi.h>
#include <stdlib.h>

void i2c_init();
int i2c_read_register(int, uint16_t);
int i2c_read_multiple_register(unsigned char*, int, unsigned char count, uint16_t);
int i2c_write_register(int, int, uint16_t);
