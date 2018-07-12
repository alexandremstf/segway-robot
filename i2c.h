#ifndef I2C_H_
#define I2C_H_

#include <util/twi.h>

void i2c_init();
int i2c_read_register(int, uint16_t);
int i2c_read_multiple_register(unsigned char*, int, unsigned char count, uint16_t);
int i2c_write_register(int, int, uint16_t);

#endif
