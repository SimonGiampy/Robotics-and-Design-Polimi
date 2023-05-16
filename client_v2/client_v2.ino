
#include "emotions.h"

// data incoming from server or for debugging via serial
String incomingData;

TaskHandle_t Task1;
TaskHandle_t Task2;

//bool has_finished = true;


// Network details
const char* ssid_test = "test";
const char* password_test = "test1234567!";
const char* server_ip_test = "192.168.1.5";
int server_port_test = 800;

const char* ssid = "192.168.56.1";
const char* password = "test1234567!";
const char* server_ip = "192.168.1.3";
int server_port = 800;

static WiFiClient client;

unsigned int from;
char emotion;
unsigned int to;
unsigned int intensity;


char currentEmotion = 'i';

void emote(MovementStruct movements, uint8_t eyes_animation[][2][16]);
// emote functions list
void emote_relaxed();
void emote_happy();
void emote_sad();
void emote_angry();
void emote_surprised();
void emote_afraid();
void emote_annoyed();
void emote_anxious();



void initMouth(){
	pinMode(PIN_MOUTH_RED,  OUTPUT);
	pinMode(PIN_MOUTH_GREEN,  OUTPUT);
	pinMode(PIN_MOUTH_BLUE,  OUTPUT);
	analogWrite(PIN_MOUTH_RED, 255);
	analogWrite(PIN_MOUTH_GREEN, 0);
	analogWrite(PIN_MOUTH_BLUE, 0);
}

void setMouthColor(uint8_t R, uint8_t G, uint8_t B){
	analogWrite(PIN_MOUTH_RED, R);
	analogWrite(PIN_MOUTH_GREEN, G);
	analogWrite(PIN_MOUTH_BLUE, B);
}

void initEars(){
	leftEarServo.attach(PIN_LEFT_EAR);
	rightEarServo.attach(PIN_RIGHT_EAR);
	leftEarServo.write(LEFT_EAR_START_POS);
	rightEarServo.write(RIGHT_EAR_START_POS);
}

void initEyebrows(){
    leftEyebrowServo.attach(PIN_LEFT_EYEBROW);
    rightEyebrowServo.attach(PIN_RIGHT_EYEBROW);
    leftEyebrowServo.write(LEFT_EYEBROW_START_POS);
    rightEyebrowServo.write(RIGHT_EYEBROW_START_POS);
}

void initNeck(){
    neckBaseZ.attach(PIN_NECK_BASE);
    neckSphereX.attach(PIN_NECK_SPHERE_X);
    neckSphereY.attach(PIN_NECK_SPHERE_Y);
    neckBaseZ.write(NECK_BASE_START_POS);
    neckSphereX.write(NECK_SPHERE_X_START_POS);
    neckSphereY.write(NECK_SPHERE_Y_START_POS);
}

void initEyes(){
    leftEye.begin();
    rightEye.begin();
    leftEye.clear();
    rightEye.clear();
    leftEye.control(MD_MAX72XX::INTENSITY, EYES_INTENSITY);
    rightEye.control(MD_MAX72XX::INTENSITY, EYES_INTENSITY);

	uint8_t eye_frame[2][16] = {{0x0, 0x0, 0x7E, 0xFF, 0xFF, 0xFF, 0xE1, 0xE1, 0xE1, 0xE1, 0xFF, 0xFF, 0xFF, 0x7E, 0x0, 0x0},{0x0, 0x0, 0x7E, 0xFF, 0xFF, 0xFF, 0xE1, 0xE1, 0xE1, 0xE1, 0xFF, 0xFF, 0xFF, 0x7E, 0x0, 0x0}};
	for(int i=0; i < 16; i++){
		rightEye.setColumn(i, eye_frame[0][i]);
		leftEye.setColumn(i, eye_frame[1][i]);
	}
}

// RXD2 pin = GPIO 15
// TXD2 pin = GPIO 5

// wirings:
// https://github.com/NicolasBrondin/flower-player/raw/master/schema.jpg
// tx pin mp3 -> gpio 5 esp32
// rx pin mp3 -> gpio 15 esp32
// from left to right, with mp3 player sd card side on the left, on the top:
// spk2 mp3 -> speaker negative
// gnd mp3 -> gnd
// spk1 mp3 -> speaker positive
// skip x2 pins
// tx mp3
// rx mp3
// vcc mp3

void initSpeaker() {
	mySoftwareSerial.begin(9600, SERIAL_8N1, PIN_Tx, PIN_Rx);  // speed, type, TX, RX

	Serial.println();
	Serial.println("Initializing DFPlayer");
	
	while(!myDFPlayer.begin(mySoftwareSerial)) {
		Serial.println("Unable to begin");
	}
	Serial.println("DFPlayer Mini online.");

	songsCount = myDFPlayer.readFileCountsInFolder(2);
	myDFPlayer.volume(20);
	myDFPlayer.EQ(DFPLAYER_EQ_NORMAL);
	myDFPlayer.outputDevice(DFPLAYER_DEVICE_SD);
	myDFPlayer.playFolder(1, 1);  
	myDFPlayer.start();

	/*
	if (myDFPlayer.available()) {
		uint8_t type = myDFPlayer.readType();
		if(type == DFPlayerPlayFinished){
			myDFPlayer.playFolder(1, 1);  
		}
	}
	*/
}


void handle(String incomingData) {
	Serial.println("handling " + incomingData);
	// Debugging the emotions and motors via serial
	if (incomingData.indexOf("m") == 0) { // move motors for debugging purposes
		int angle;
		if (incomingData.indexOf("mx") == 0) { // motor x axis
			angle = incomingData.substring(2, 5).toInt();
			neckSphereX.write(angle);
		} else if (incomingData.indexOf("my") == 0) { // motor y axis
			angle = incomingData.substring(2, 5).toInt();
			neckSphereY.write(angle);
		} else if (incomingData.indexOf("mz") == 0) { // motor z axis
			angle = incomingData.substring(2, 5).toInt();
			neckBaseZ.write(angle);
		}
		
	} else if (incomingData.indexOf("happy_") == 0) {
		emote_happy();
	} else if (incomingData.indexOf("angry_") == 0) {
		emote_angry();
	} else if (incomingData.indexOf("anxious_") == 0) {
		emote_anxious();
	} else if (incomingData.indexOf("sad_") == 0) {
		emote_idle();
	} else if (incomingData.indexOf("relaxed_") == 0) {
		emote_relaxed();
	} else if (incomingData.indexOf("surprised_") == 0) {
		emote_surprised();
	} else if (incomingData.indexOf("afraid_") == 0) {
		emote_afraid();
	} else if (incomingData.compareTo("idle_") == 0) {
		emote_idle();
	} else if (incomingData.compareTo("G1") == 0) { // Rocco message
		lookAt(1);
	} else if (incomingData.compareTo("G2") == 0) { // Rocco message
		lookAt(1);
	} else if (incomingData.compareTo("G3") == 0) { // Eva message
		
	} else if (incomingData.compareTo("G4") == 0) { // Eva message
		
	} else if (incomingData.compareTo("G5") == 0) { // Lele message
		lookAt(3);
	} else if (incomingData.compareTo("G6") == 0) { // Lele message
		lookAt(3);
	} else if (incomingData.compareTo("G7") == 0) { // Carlotta Message
		lookAt(4);
	} else if (incomingData.compareTo("G8") == 0) { // Carlotta Message
		lookAt(4);
	} else if (incomingData.compareTo("G9") == 0) {	// Peppe message
		lookAt(5);
	} else if (incomingData.compareTo("GA") == 0) { // Peppe message
		lookAt(5);
	} else if (incomingData.compareTo("GB") == 0) { // Bianca message
		lookAt(6);
	} else if (incomingData.compareTo("GC") == 0) { // Bianca message
		lookAt(6);
	} else if (incomingData.compareTo("GD") == 0) { // Cosimo message
		lookAt(7);	
	} else if (incomingData.compareTo("GE") == 0) { // Cosimo message
		lookAt(7);
	} else if (incomingData.compareTo("GF") == 0) {
		//god kills any sequence of reactions
		//set flag to prevent any reaction and clear the queue of reactions
		
	} else if (incomingData.compareTo("GG") == 0) {
		//presentation finished
		//should_handle = false;
	} else if (incomingData.length() == 4) { // reactions of the other robots
		
		from = incomingData[0] - '0';
		emotion = incomingData[1];
		to = incomingData[2] - '0';
		intensity = incomingData[3] - '0';

		// the neck rotates in the direction of the robot who is making the emotion
		lookAt(from);

		// reactions
		// A = idle, B = happy, C = angry, D = shocked, E = sad, F = relaxed, G = afraid, H = cautious,
		// I = surprised, J = annoyed, K = embarrassed, L = anxious

		// send the server the robot reaction to the other robot
		// client.print("2A51") // eva is happy towards group 5 at intensity 1

		//client.flush(); // ?
		
	}
}

void main_loop(void* pvParameters) {
	// Handle emotions
	while (true) {
		//Read from serial interface to bypass WiFi for debugging
		if (Serial.available()) {
			incomingData = Serial.readStringUntil('\n');
			Serial.println("Serial data = " + incomingData);
			handle(incomingData);
		}
		delay(10);
	}
}

void network_loop(void* pvParameters) {
  // Handle connection and receive from WiFi (updating state)
  while (true) {

	if (WiFi.status() != WL_CONNECTED) {
		initConnection();
	}

	if (client.available()) {
		incomingData = client.readString();
		Serial.println("Server says: " + incomingData);
		/*if(should_handle) {
			if(!has_finished){
			queue.enqueue(incomingData);
		} else {
			while(queue.size() > 0) {
			incomingData = queue.dequeue();
			handle(incomingData);
			}
		} 
		}*/
		handle(incomingData);
	}

	delay(10);
  }
}

void initConnection() {

	Serial.println();
	Serial.print("Connecting to ");
	Serial.println(ssid_test);

	// WiFi connection
	WiFi.begin(ssid_test, password_test);

	while (WiFi.status() != WL_CONNECTED) {
		delay(100);
	}
	Serial.println();
	Serial.println("WiFi connected");
	Serial.println("IP address: ");
	Serial.println(WiFi.localIP());

	// Server connection
	Serial.println();
	Serial.println("Connecting to Server...");
	while (!client.connect(server_ip_test, server_port_test)) {
		delay(100);
	}
	Serial.println("Server connected");
	Serial.println();
}

void setup() {
	Serial.begin(115200);

	initMouth();
	initEars();
	initEyebrows();
	initNeck();
	initEyes();
	initSpeaker();
	delay(500);

	//Wifi and Server Init
	initConnection();

	//Parallel functions

	// Create a task that will be executed in the main_loop() function,
	// with name "main", stack size 10000, NULL as parameter, priority 1,
	// handled by Task1 and executed on core 0
	xTaskCreatePinnedToCore(main_loop, "main", 10000, NULL, 1, &Task1, 0);
	delay(500);

	xTaskCreatePinnedToCore(network_loop, "network", 10000, NULL, 1, &Task2, 1);
	delay(500);


}

void loop()  {

}

// look at the robot who is making the emotion with its number
void lookAt(int robotNumber) {
	if (robotNumber == 1) {
		neckBaseZ.write(180); // robot sits on its left
	} else if (robotNumber == 3) {
		neckBaseZ.write(0); // robot sits on its right
	} else if (robotNumber == 4) {
		neckBaseZ.write(30);
	} else if (robotNumber == 5) {
		neckBaseZ.write(60);
	} else if (robotNumber == 6) {
		neckBaseZ.write(90);
	} else if (robotNumber == 7) {
		neckBaseZ.write(120); 
	}
}

void tiltX_Up() {
	neckSphereX.write(90); // adjust angle and velocity
}

void tiltX_Down() {
	neckSphereX.write(90); // adjust angle
}

void tiltY_Left() {
	neckSphereY.write(90); // adjust angle
}

void tiltY_Right() {
	neckSphereY.write(90); // adjust angle
}

void tiltZ_Left() {
	neckBaseZ.write(90); // adjust angle
}

void tiltZ_Right() {
	neckBaseZ.write(90); // adjust angle
}

void emote(MovementStruct movements, uint8_t eyes_animation[][2][16]) {
	// compute length of eyes animation
	int eyes_animation_length = sizeof(*eyes_animation) / sizeof(eyes_animation[0]);

	//iterate over every frame of the animation
	for(int frame = 0; frame < movements.maxFrames; frame++){
		neckBaseZ.write(movements.neckBaseAngles[frame % movements.neckBaseFrames]);
		neckSphereX.write(movements.neckSphereXAngles[frame % movements.neckSphereXFrames]);
		neckSphereY.write(movements.neckSphereYAngles[frame % movements.neckSphereYFrames]);
		leftEarServo.write(movements.leftEarAngles[frame % movements.leftEarFrames]);
		rightEarServo.write(movements.rightEarAngles[frame % movements.rightEarFrames]);
		// iterate over every column of the eyes matrices
		for(int i=0; i < 16; i++) {
			rightEye.setColumn(i, eyes_animation[frame % eyes_animation_length][0][i]);
			leftEye.setColumn(i, eyes_animation[frame % eyes_animation_length][1][i]);
		}
		// delay for the duration of the frame
		unsigned long time_now = millis();
		while (millis() < time_now + 75){
			//wait approx. 100 ms
		}
	}

}

void emote_relaxed() {
	emote(relaxedMovements, relaxed);
}

void emote_idle() {
	emote(idleMovements, relaxed);
}

void emote_happy() {
	emote(happyMovements, happy);
}

void emote_sad() {
	emote(sadMovements, sad);
}

void emote_angry() {
	emote(angryMovements, angry);
}

void emote_surprised() {
	emote(surprisedMovements, surprised);
}

void emote_annoyed() {
	emote(annoyedMovements, annoyed);
}

void emote_anxious() {
	emote(anxiousMovements, anxious);
}

void emote_afraid() {
	emote(afraidMovements, afraid);
}


