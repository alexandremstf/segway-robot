#include "mpu6050.h"

void mpu6050_init(int adress)	{
	_delay_ms(150);
	i2c_write_register(PWR_MGMT_1, 0x00, adress);
	i2c_write_register(CONFIG, 0x01, adress);
	i2c_write_register(INT_ENABLE, 0x00, adress);
	i2c_write_register(SIGNAL_PATH_RESET, 0x00, adress);
	i2c_write_register(SMPLRT_DIV, 0x00, adress);
}

double read_acceleration_x(int adress) {
	uint8_t buffer_reg[2];
	buffer_reg[0] = i2c_read_register(ACCEL_XOUT_H, adress);
	buffer_reg[1] = i2c_read_register(ACCEL_XOUT_L, adress);
	int16_t buffer = (buffer_reg[0] << 8) | (buffer_reg[1]);
	return ((double)buffer * 2 / 32768);
}

double read_acceleration_y(int adress) {
	uint8_t buffer_reg[2];
	buffer_reg[0] = i2c_read_register(ACCEL_YOUT_H, adress);
	buffer_reg[1] = i2c_read_register(ACCEL_YOUT_L, adress);
	int16_t buffer = (buffer_reg[0] << 8) | (buffer_reg[1]);
	return ((double)buffer * 2 / 32768);
}

double read_acceleration_z(int adress) {
	uint8_t buffer_reg[2];
	buffer_reg[0] = i2c_read_register(ACCEL_ZOUT_H, adress);
	buffer_reg[1] = i2c_read_register(ACCEL_ZOUT_L, adress);
	int16_t buffer = (buffer_reg[0] << 8) | (buffer_reg[1]);
	return ((double)buffer * 2 / 32768);
}

double read_gyro_x(int adress) {
	uint8_t buffer_reg[2];
	buffer_reg[0] = i2c_read_register(GYRO_XOUT_H, adress);
	buffer_reg[1] = i2c_read_register(GYRO_XOUT_L, adress);
	int16_t buffer = (buffer_reg[0] << 8) | (buffer_reg[1]);
	return ((double)buffer * 250) / 32768;
}

double read_gyro_y(int adress) {
	uint8_t buffer_reg[2];
	buffer_reg[0] = i2c_read_register(GYRO_YOUT_H, adress);
	buffer_reg[1] = i2c_read_register(GYRO_YOUT_L, adress);
	int16_t buffer = (buffer_reg[0] << 8) | (buffer_reg[1]);
	return ((double)buffer * 250 ) / 32768;
}

double read_gyro_z(int adress) {
	uint8_t buffer_reg[2];
	buffer_reg[0] = i2c_read_register(GYRO_ZOUT_H, adress);
	buffer_reg[1] = i2c_read_register(GYRO_ZOUT_L, adress);
	int16_t buffer = (buffer_reg[0] << 8) | (buffer_reg[1]);
	return ((double)buffer * 250) / 32768;
}
