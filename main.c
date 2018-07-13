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

double dt = 0.003;
double angle_y = 0;
double erro_anterior = 0;
double integral = 0;

double get_angle();
double controller();
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

double controller(){
	double erro = (0 - get_angle()) * (M_PI / 180);
	double derivada = (erro - erro_anterior) / dt;
	integral = integral + erro * dt;
	erro_anterior = erro;

	double output = 200 * erro + 0 * derivada + 0 * integral;

	if (output > 255) output = 255;
	if (output < -255) output = -255;
	return output;
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
	// set compare match register for 300.0018750117188 Hz increments
	OCR1A = 53332; // = 16000000 / (1 * 300.0018750117188) - 1 (must be <65536)
	// turn on CTC mode
	TCCR1B |= (1 << WGM12);
	// Set CS12, CS11 and CS10 bits for 1 prescaler
	TCCR1B |= (0 << CS12) | (0 << CS11) | (1 << CS10);
	// enable timer compare interrupt
	TIMSK1 |= (1 << OCIE1A);
	sei(); // allow interrupts
}

ISR(TIMER1_COMPA_vect){
	double c = controller();

	set_PWM(c, 1);
	set_PWM(c, 2);
}
