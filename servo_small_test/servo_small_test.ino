/*
 * This ESP32 code is created by esp32io.com
 *
 * This ESP32 code is released in the public domain
 *
 * For more detail (instruction and wiring diagram), visit https://esp32io.com/tutorials/esp32-servo-motor
 */

#include <ESP32Servo.h>

Servo servo_left, servo_right;

void setup() {
	servo_left.attach(12);  // attaches the servo on ESP32 pin
	servo_right.attach(14);  // attaches the servo on ESP32 pin
	
	servo_left.write(90);
	servo_right.write(90);
	delay(1000);
}


void loop() {
	
	for (int i = 0; i < 40; i++) {
		servo_left.write(i);
		delay(40);
	}

	delay(1000);
	
	for (int i = 80; i < 180; i++) {
		servo_right.write(i);
		delay(40);
	}

	delay(1000);
	
}
