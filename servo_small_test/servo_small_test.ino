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
	servo_left.attach(19);  // attaches the servo on ESP32 pin
	servo_right.attach(18);  // attaches the servo on ESP32 pin

	// right servo motor goes from standing position at 150 degrees down to 80 degrees (pulling position)
	// left servo motor goes from standing position at 80 degrees up to 150 degrees (pulling position)
	
	servo_left.write(100);
	servo_right.write(100);
	delay(1000);
}


void loop() {
	
	for (int i = 80; i < 150; i++) {
		servo_left.write(i);
		delay(30);
	}

	//delay(1000);
	
	for (int i = 150; i > 80; i--) {
		servo_right.write(i);
		delay(30);
	}

	//delay(1000);
	
}
