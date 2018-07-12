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

	struct Kalman kalmanX = init_kalman(kalmanX);
	struct Kalman kalmanY = init_kalman(kalmanY);

	double kalAngleX = 0;
	double kalAngleY = 0;

    double gyroXangle = 0;
    double gyroYangle = 0;

	double compAngleX = 0;
    double compAngleY = 0;

	double dt = 0.05;

	while(1) {
		double accX = (read_acceleration_x(0x68) + read_acceleration_x(0x69)) / 2;
		double accY = (read_acceleration_y(0x68) + read_acceleration_y(0x69)) / 2;
		double accZ = (read_acceleration_z(0x68) + read_acceleration_z(0x69)) / 2;
		double gyroX = (read_gyro_x(0x68) + read_gyro_x(0x69)) / 2 ;
		double gyroY = (read_gyro_y(0x68) + read_gyro_y(0x69)) / 2 ;
		double gyroZ = (read_gyro_z(0x68) + read_gyro_z(0x69)) / 2 ;

		double roll  = atan2(accY, accZ) * (180.0 / M_PI);
		double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * (180.0 / M_PI);

		double gyroXrate = gyroX; // Convert to deg/s
	    double gyroYrate = gyroY; // Convert to deg/s

		kalAngleX = getAngle(kalmanX, roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
	    kalAngleY = getAngle(kalmanY, pitch, gyroYrate, dt);

	    gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
	    gyroYangle += gyroYrate * dt;

	    compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
	    compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

		usart_transmit_double(compAngleX);
  		usart_transmit('\t');
  		usart_transmit_double(compAngleX);
  		usart_transmit('\n');
/*
		usart_transmit_double( (read_acceleration_z(0x68) + read_acceleration_z(0x69)) / 2 );
		usart_transmit('\t');
		usart_transmit_double( (read_gyro_y(0x68) + read_gyro_y(0x69)) / 2 );
		usart_transmit('\n');
*/
		_delay_ms(50);
	}
}
