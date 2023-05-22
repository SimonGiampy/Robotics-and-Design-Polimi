/*
 * This ESP32 code is created by esp32io.com
 *
 * This ESP32 code is released in the public domain
 *
 * For more detail (instruction and wiring diagram), visit https://esp32io.com/tutorials/esp32-servo-motor
 */

#include <ESP32Servo.h>
#include <string>

Servo servo_left, servo_right;

void setup() {
	Serial.begin(115200);
	servo_left.attach(19);  // attaches the servo on ESP32 pin
	servo_right.attach(18);  // attaches the servo on ESP32 pin

	// right servo motor goes from standing position at 150 degrees down to 80 degrees (pulling position)
	// left servo motor goes from standing position at 80 degrees up to 150 degrees (pulling position)
	delay(1000);
}


void loop() {
	
	// read string from serial monitor
	std::string input = "";
	while (Serial.available()) {
		input = std::string(Serial.readString().c_str());
		if (input[0] == 'l') {
			// left: up from 160, down at 95 degrees
			servo_left.write(std::stoi(input.substr(1, 3)));
		} else if (input[0] == 'r') {
			// right: up from 90 degrees, down at 160 degrees
			servo_right.write(std::stoi(input.substr(1, 3)));
		}
		delay(10);
	}
	
}
