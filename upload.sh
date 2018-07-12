#!/bin/bash

avr-gcc -Os -DF_CPU=16000000UL -mmcu=atmega328p -c -o main.o main.c
avr-gcc -Os -DF_CPU=16000000UL -mmcu=atmega328p -c -o i2c.o i2c.c
avr-gcc -Os -DF_CPU=16000000UL -mmcu=atmega328p -c -o usart.o usart.c
avr-gcc -Os -DF_CPU=16000000UL -mmcu=atmega328p -c -o kalman.o kalman.c
avr-gcc -Os -DF_CPU=16000000UL -mmcu=atmega328p -c -o mpu6050.o mpu6050.c
avr-gcc -mmcu=atmega328p main.o i2c.o usart.o kalman.o mpu6050.o -o main
avr-objcopy -O ihex -R .eeprom main main.hex
avrdude -v -p m328p -c arduino -P /dev/ttyUSB0 -b 57600 -D -U flash:w:main.hex:i

rm *.o
rm *.hex
