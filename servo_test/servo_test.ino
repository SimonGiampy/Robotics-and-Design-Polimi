
#include <ESP32Servo.h>

#define PIN_NECK_BASE 21
#define PIN_NECK_SPHERE_A 22
#define PIN_NECK_SPHERE_B 23

Servo servo_1, servo_2, servo_3;

void setup() {
	servo_3.attach(PIN_NECK_BASE);  // attaches the servo on ESP32 pin
	servo_1.attach(PIN_NECK_SPHERE_A);  // attaches the servo on ESP32 pin
	servo_2.attach(PIN_NECK_SPHERE_B);  // attaches the servo on ESP32 pin

	servo_1.write(110); // servo for spherical joint 1 = pitch angle x axis
	servo_2.write(55); // servo for spherical joint 2 = roll angle y axis 
	servo_3.write(90); // external servo for yaw rotation on z axis

	// servo 1: mid point at 110, oscillates backwards down to 40, oscillates forwards up to 170
	// servo 2: mid point at 55, oscillates on the right up to 0 degrees, oscillates left up to 125 degress
	// (these servo angles are fixed like this if the internal gears do not skip a rotation)
	// servo 3: mid point at 90

	delay(2000);
}


void loop() {

	// first servo
	for (int pos = 40; pos <= 170; pos += 1) {
		servo_1.write(pos);
		delay(50);
	}

	// rotates from 180 degrees to 0 degrees the second servo motor
	for (int pos = 170; pos >= 40; pos -= 1) {
		servo_1.write(pos);
		delay(50); 
	}
	
	servo_1.write(110);
	delay(1000);
	
	
	// second servo
	for (int pos = 0; pos <= 125; pos += 1) {
		servo_2.write(pos);
		delay(50);
	}

	for (int pos = 125; pos >= 0; pos -= 1) {
		servo_2.write(pos);
		delay(50); 
	}

	servo_2.write(55);
	delay(1000);

	// third servo
	for (int pos = 0; pos <= 180; pos += 1) {
		servo_3.write(pos);
		delay(50);
	}

	for (int pos = 180; pos >= 0; pos -= 1) {
		servo_3.write(pos);
		delay(50); 
	}
	
	servo_3.write(90);
	delay(1000);
	

}
