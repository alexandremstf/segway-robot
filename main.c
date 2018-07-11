#define BAUD 38400
#define MYUBRR 16000000U / 16 / BAUD-1

#include "i2c.h"										/* Include I2C file */
#include "usart.h"										/* Include USART file */
#include "mpu6050.h"									/* Include MPU6050 register define file */
#include <util/delay.h>									/* Include delay header file */

void mpu6050_init(int adress)	{
	_delay_ms(150);
	i2c_write_register(PWR_MGMT_1, 0x00, adress);
	i2c_write_register(CONFIG, 0x01, adress);
	i2c_write_register(INT_ENABLE, 0x00, adress);
	i2c_write_register(SIGNAL_PATH_RESET, 0x00, adress);
	i2c_write_register(SMPLRT_DIV, 0x00, adress);
}

double read_acceleration_z(int adress) {
	uint8_t buffer_reg[2];
	buffer_reg[0] = i2c_read_register(ACCEL_ZOUT_H, adress);
	buffer_reg[1] = i2c_read_register(ACCEL_ZOUT_L, adress);
	int16_t buffer = (buffer_reg[0] << 8) | (buffer_reg[1]);
	return ((double)buffer * 9.8 * 2 / 32768);
}

double read_gyro_y(int adress) {
	uint8_t buffer_reg[2];
	buffer_reg[0] = i2c_read_register(GYRO_YOUT_H, adress);
	buffer_reg[1] = i2c_read_register(GYRO_YOUT_L, adress);
	int16_t buffer = (buffer_reg[0] << 8) | (buffer_reg[1]);
	return ((double)buffer * 250 * 3.1415) / 32768 / 180;
}

void main() {

	i2c_init();
	usart_init(MYUBRR);
	mpu6050_init(0x68);
	mpu6050_init(0x69);

	while(1) {

		(read_acceleration_z(0x68) + read_acceleration_z(0x69)) / 2;
		usart_transmit_double( (read_acceleration_z(0x68) + read_acceleration_z(0x69)) / 2 );
		usart_transmit('\t');
		usart_transmit_double( (read_gyro_y(0x68) + read_gyro_y(0x69)) / 2 );
		usart_transmit('\n');

		_delay_ms(10);
	}
}
