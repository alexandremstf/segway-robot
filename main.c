//DEFINE BAUDRATE FOR USART COMMUNICATION
#define BAUD 38400
//CONFIGURES THE CLOCK FREQUENCY FOR USART COMMUNICATION
#define MYUBRR 16000000U / 16 / BAUD-1

//PRE-COMPILATION DIRECTIVES TO CONFIGURE PORTB AND PORTD
#define CONFIG_POTB DDRB |= 0x01 //SETS PORTB1 AS OUTPUT AND THE REST REMAINS AS THEY WERE (IN THIS CASE, ALL OTHERS AS INPUTS)
#define CONFIG_POTD DDRD |= 0xF0 //SETS PORTD7,6,5,4 AS OUTPUTS AND THE REST REMAINS AS THEY WERE (IN THIS CASE, ALL OTHERS AS INPUTS)

#include "i2c.h"										/* Include I2C file */
#include "usart.h"										/* Include USART file */
#include "mpu6050.h"									/* Include MPU6050 register define file */
#include <util/delay.h>									/* Include delay header file */
#include <avr/interrupt.h>								/* Include interrupt header file */
#include <stdlib.h>										/* Include stdlib header file */

//THIS LAST LIB IS ADD TO USE THE abs() FUNCTION, USED LATER ON.

//THE #INCLUDE <AVR/IO.H> IS CALLED AT "usart.h" AS IT IS NEEDED TO CONFIGURE THE USART

//VARIABLES USED FOR THE CONTROL EQUATION
volatile double dt = 0.01;
volatile double angle_y = 0;
volatile double erro_anterior = 0;
volatile double integral = 0;

//FUNCTION SCOPES DEFINED LATER ON
double get_angle();  //RETURNS THE RESULT OF THE COMPLEMENTARY FILTER APPLIED TO THE SENSORS READINGS
double controller(); //FUNCTION THAT RETURNS THE PWM TO APPLY TO THE MOTORS FROM THE CONTROL EQUATION
void config_timer_interrupt();  //FUNCTION USED TO CONFIGURE THE TIMER1 AS INTERRUPT FOR THE CONTROL EVENT
void configPWM(); //FUNCTION TO CONFIGURE THE PWM OUTPUT PINS OC2B AND OC2A - PB3 AND PD3
void set_PWM(int16_t, uint8_t); //FUNCTION THAT SETS THE PWM TO THE OUTPUT (PD3 OR PB3)

void main() {
	CONFIG_POTB;  //CONFIGURE PORTB input/outputs
	CONFIG_POTD;  //CONFIGURE PORTD input/outputs

	i2c_init();   //FUNCTION THAT SETS THE INITIAL CONFIGURATION FOR THE I2C COMMUNICATION
	usart_init(MYUBRR);  //SET THE CONFIGURATION FOR THE USART COMMUNICATION FOR THE MYUBRR CLK FREQUENCY - NOTE THAT USART IS USED ONLY FOR DEBBUGING
	config_timer_interrupt(); //FUNCTION THAT SETS THE TIMER1 CONFIGURATION FOR INTERRUPTION
	configPWM();  //FUNCTION THAT SETS THE CONFIGURATION FOR THE PWM

	//INICIALIZE BOTH SENSORS
	mpu6050_init(0x68);
	mpu6050_init(0x69);

	//INFINITE "WAIT" LOOP
	while(1) {}
	//AFTER SET UP THE MCU ACTS ONLY DURING TIMER1 ISR, CALLING THE CONTROL FUNCTION
}

double controller(){
	double erro = (-3 - get_angle()) * (M_PI / 180);

	double derivada = (erro - erro_anterior) / dt;
	integral = integral + erro * dt;
	erro_anterior = erro;

	double output = 200 * erro + 0.8 * derivada + 0 * integral;

	if (output > 255) output = 255;
	if (output < -255) output = -255;

	//usart_transmit_double(output);
	//usart_transmit('\n');

	return output;
}

double get_angle(){
	//THIS FUNCTION RETURNS THE COMPLEMENTAR FILTER OUTPUT FOR THE 2 SENSORS USED
	double acceleration_x = (read_acceleration_x(0x68) + read_acceleration_x(0x69)) / 2;
	double acceleration_y = (read_acceleration_y(0x68) + read_acceleration_y(0x69)) / 2;
	double acceleration_z = (read_acceleration_z(0x68) + read_acceleration_z(0x69)) / 2;
	double gyro_y = (read_gyro_y(0x68) + read_gyro_y(0x69)) / 2 ;

	//AFTER COLLECTING THE NECESSARY READING TO CALCULATE THE  DESIRED ANGULAR POSITION
	//THE EQUATION FOR THE COMPLEMENTARY FILTER IS APPLIED
	double pitch = atan(-acceleration_z / sqrt(acceleration_y * acceleration_y + acceleration_x * acceleration_x)) * (180.0 / M_PI);
	angle_y = 0.8 * (angle_y + gyro_y * dt) + 0.2 * pitch;
	//RETURNS THE CALCULATED ANGLE
	return angle_y;
}

//TIMER 2 for Fast PWM with 31.7kHz

void configPWM() {
	//Set ports PD3 and PB3 as output AND THE OTHER PINS REMAIN AS THEY WHERE
	DDRB |= 1 << PB3;
	DDRD |= 1 << PD3;
	//AS PD3 IS OC2B
	//AND PB3 OC2A

	//Turn on Fast PWM mode
	TCCR2A |= (1 << COM2A1) | (1 << COM2B1) | (1 << WGM21) | (1 << WGM20);
	TCCR2B |= (1 << CS20);

	//STBY PIN TO HIGH - CONNECTED TO PD4
	PORTD |= 0x40;
}

void set_PWM(int16_t pwm, uint8_t pin) {
	//Output compare register set for PWM level
	//8bits (0 to 255)
	//CHECK WHICH PIN TO SET THE PWM VALUE PD3 OR PB3
	if (pin == 1) {
		if (pwm > 0) {
			//WHEN ITS GREATER THAN 0, ROTATES THE MOTOR IN CLOCKWISE DIRECTION
			PORTD |= 0x20;
			PORTD &= ~0x10;
		} else {
			//WHEN ITS GREATER THAN 0, ROTATES THE MOTOR IN COUNTERCLOCKWISE DIRECTION
			PORTD &= ~0x20;
			PORTD |= 0x10;
		}
		OCR2B = abs(pwm); //UPDATES THE PWM VALUE (O TO 255) TO THE COMPARE REGISTER B

	} else if (pin == 2) {
		if (pwm > 0) {
			//WHEN ITS GREATER THAN 0, ROTATES THE MOTOR IN CLOCKWISE DIRECTION
			PORTD |= 0x80;
			PORTB &= ~0x01;
		} else {
			//WHEN ITS GREATER THAN 0, ROTATES THE MOTOR IN COUNTERCLOCKWISE DIRECTION
			PORTD &= ~0x80;
			PORTB |= 0x01;
		}
		OCR2A = abs(pwm); //UPDATES THE PWM VALUE (O TO 255) TO THE COMPARE REGISTER A
	}
}

void config_timer_interrupt() {
	cli(); // stop interrupts
	TCCR1A = 0; // set entire TCCR1A register to 0
	TCCR1B = 0; // same for TCCR1B
	TCNT1  = 0; // initialize counter value to 0
	// set compare match register for 300.0018750117188 Hz increments
	OCR1A = 19999; // = 16000000 / (8 * 100) - 1 (must be <65536)
	// turn on CTC mode
	//Configures the Timer1 as Counter with OCR1A as TOP (Operation Mode 4)
	TCCR1B |= (1 << WGM12);
	// Set CS12, CS11 and CS10 bits for 1 prescaler - no MCU frequency division
	TCCR1B |= (0 << CS12) | (1 << CS11) | (0 << CS10);
	// enable timer compare interrupt when TCNT1 matches OCR1A
	TIMSK1 |= (1 << OCIE1A);
	sei(); // allow global interrupts
}

ISR(TIMER1_COMPA_vect){
	//ROUTINE FOR TREATING THE TIMER1 INTERRUPT EVENT - THE CONTROL FUNCTION CALL
	double c = controller();
	//SET BOTH MOTORS WITH THE SAME PWM
	set_PWM(c, 1);
	set_PWM(c, 2);
}
