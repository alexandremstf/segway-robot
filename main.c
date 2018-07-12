#define BAUD 38400
#define MYUBRR 16000000U / 16 / BAUD-1

#include "i2c.h"										/* Include I2C file */
#include "usart.h"										/* Include USART file */
#include "mpu6050.h"									/* Include MPU6050 register define file */
#include <util/delay.h>									/* Include delay header file */
#include <avr/interrupt.h>

double dt = 0.01;
double angle_y = 0;

double get_angle();
void controller();
void config_timer_interrupt();

void main() {
	i2c_init();
	usart_init(MYUBRR);
	config_timer_interrupt();

	mpu6050_init(0x68);
	mpu6050_init(0x69);

	while(1) {}
}

void controller(){
	usart_transmit_double( get_angle() );
	usart_transmit('\n');
}

double get_angle(){
	double acceleration_x = (read_acceleration_x(0x68) + read_acceleration_x(0x69)) / 2;
	double acceleration_y = (read_acceleration_y(0x68) + read_acceleration_y(0x69)) / 2;
	double acceleration_z = (read_acceleration_z(0x68) + read_acceleration_z(0x69)) / 2;
	double gyro_y = (read_gyro_y(0x68) + read_gyro_y(0x69)) / 2 ;

	double pitch = atan(-acceleration_z / sqrt(acceleration_y * acceleration_y + acceleration_x * acceleration_x)) * (180.0 / M_PI);
	angle_y = 0.7 * (angle_y + gyro_y * dt) + 0.3 * pitch;

	return angle_y;
}

void config_timer_interrupt() {
	cli(); // stop interrupts
	TCCR1A = 0; // set entire TCCR1A register to 0
	TCCR1B = 0; // same for TCCR1B
	TCNT1  = 0; // initialize counter value to 0
	OCR1A = 19999; // = 16000000 / (8 * 100) - 1 (must be <65536)
	TCCR1B |= (1 << WGM12);
	TCCR1B |= (0 << CS12) | (1 << CS11) | (0 << CS10);
	TIMSK1 |= (1 << OCIE1A);
	sei(); // allow interrupts
}

ISR(TIMER1_COMPA_vect){
	controller();
}
