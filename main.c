#define BAUD 38400
#define MYUBRR 16000000U / 16 / BAUD-1

#include "i2c.h"										/* Include I2C file */
#include "usart.h"										/* Include USART file */
#include "kalman.h"										/* Include KALMAN file */
#include "mpu6050.h"									/* Include MPU6050 register define file */
#include <util/delay.h>									/* Include delay header file */

void main() {

	i2c_init();
	usart_init(MYUBRR);
	mpu6050_init(0x68);
	mpu6050_init(0x69);

    double compAngleY = 0;

	double dt = 0.01;

	while(1) {
		double accX = (read_acceleration_x(0x68) + read_acceleration_x(0x69)) / 2;
		double accY = (read_acceleration_y(0x68) + read_acceleration_y(0x69)) / 2;
		double accZ = (read_acceleration_z(0x68) + read_acceleration_z(0x69)) / 2;
		double gyroY = (read_gyro_y(0x68) + read_gyro_y(0x69)) / 2 ;

		double pitch = atan(-accZ / sqrt(accY * accY + accX * accX)) * (180.0 / M_PI);

	    compAngleY = 0.7 * (compAngleY + gyroY * dt) + 0.3 * pitch;

		usart_transmit_double(pitch);
  		usart_transmit('\t');
		usart_transmit_double(compAngleY);
  		usart_transmit('\n');

		_delay_ms(10);
	}
}
