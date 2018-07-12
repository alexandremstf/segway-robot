#define BAUD 38400
#define MYUBRR 16000000U / 16 / BAUD-1
#define CONFIG_POTB DDRB |= 0x01
#define CONFIG_POTD DDRD |= 0xF0

#include "i2c.h"										/* Include I2C file */
#include "usart.h"										/* Include USART file */
#include "mpu6050.h"									/* Include MPU6050 register define file */
#include <util/delay.h>									/* Include delay header file */
#include <avr/interrupt.h>
#include <stdlib.h>

double dt = 0.01;
double angle_y = 0;

double get_angle();
void controller();
void config_timer_interrupt();
void configPWM();
void set_PWM(int16_t, uint8_t);

void main() {
	CONFIG_POTB;
	CONFIG_POTD;

	i2c_init();
	usart_init(MYUBRR);
	config_timer_interrupt();
	configPWM();

	mpu6050_init(0x68);
	mpu6050_init(0x69);

	while(1) {}
}

void controller(){
	/*
	if (get_angle() < 0) {
		set_PWM(120, 1);
		set_PWM(120, 2);
	} else {
		set_PWM(-120, 1);
		set_PWM(-120, 2);
	}
	*/

	usart_transmit_double( get_angle() );
	usart_transmit('\n');
}

double get_angle(){
	double acceleration_x = (read_acceleration_x(0x68) + read_acceleration_x(0x69)) / 2;
	double acceleration_y = (read_acceleration_y(0x68) + read_acceleration_y(0x69)) / 2;
	double acceleration_z = (read_acceleration_z(0x68) + read_acceleration_z(0x69)) / 2;
	double gyro_y = (read_gyro_y(0x68) + read_gyro_y(0x69)) / 2 ;

	double pitch = atan(-acceleration_z / sqrt(acceleration_y * acceleration_y + acceleration_x * acceleration_x)) * (180.0 / M_PI);
	angle_y = 0.8 * (angle_y + gyro_y * dt) + 0.2 * pitch;

	return angle_y;
}

//TIMER 2 for Fast PWM with 31.7kHz

void configPWM() {
	//Set ports PD3 and PB3 as output
	DDRB |= 1 << PB3;
	DDRD |= 1 << PD3;

	//Turn on Fast PWM mode
	TCCR2A |= (1 << COM2A1) | (1 << COM2B1) | (1 << WGM21) | (1 << WGM20);
	TCCR2B |= (1 << CS20);

	//STBY PIN TO HIGH
	PORTD |= 0x40;
}

void set_PWM(int16_t pwm, uint8_t pin) {
	//Output compare register set for PWM level
	//8bits (0 to 255)

	if (pin == 1) {
		if (pwm > 0) {
			PORTD |= 0x20;
			PORTD &= ~0x10;
		} else {
			PORTD &= ~0x20;
			PORTD |= 0x10;
		}
		OCR2B = abs(pwm);

	} else if (pin == 2) {
		if (pwm > 0) {
			PORTD |= 0x80;
			PORTB &= ~0x01;
		} else {
			PORTD &= ~0x80;
			PORTB |= 0x01;
		}
		OCR2A = abs(pwm);
	}
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
