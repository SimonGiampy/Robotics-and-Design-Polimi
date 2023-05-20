#include "emotions.h"

// data incoming from server or for debugging via serial
std::string incomingData;

TaskHandle_t Task1;
TaskHandle_t Task2;

//bool has_finished = true;


// Network details
const char* ssid_test = "test";
const char* password_test = "test1234567!";
const char* server_ip_test = "192.168.1.5";
int server_port_test = 800;

const char* ssid = "Doki Doki Literature Club";
const char* password = "SimonGiampy";
const char* server_ip = "192.168.90.244";
int server_port = 800;

static WiFiClient client;

unsigned int from;
char emotion;
unsigned int to;
unsigned int intensity;
std::string god_message =  "G0";

// create queue for messages received from server in cpp
std::deque<std::string> messages;
bool kill_signal = false;

void emote(MovementStruct movements, std::vector<std::vector<std::vector<uint8_t>>> &emotion, int audio);
// emote functions list
void emote_idle();
void emote_happy();
void emote_sad();
void emote_angry();
void emote_surprised();
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

void initSpeaker() {
	mySoftwareSerial.begin(9600, SERIAL_8N1, PIN_Tx, PIN_Rx);  // speed, type, TX, RX

	Serial.println();
	Serial.println("Initializing DFPlayer");
	
	while(!myDFPlayer.begin(mySoftwareSerial)) {
		Serial.println("Unable to begin");
	}
	Serial.println("DFPlayer Mini online.");

	int mp3Count = myDFPlayer.readFileCounts();
	Serial.print("mp3 files count: " + String(mp3Count));
	myDFPlayer.volume(20);
	myDFPlayer.EQ(DFPLAYER_EQ_NORMAL);
	myDFPlayer.outputDevice(DFPLAYER_DEVICE_SD);

	// audio file names
	// 1.mp3 -> tts message G4
	// 2.mp3 -> tts message G3
	// 3.mp3 -> angry
	// 4.mp3 -> happy
	// 5.mp3 -> sad
	// 6.mp3 -> surprised
	// 7.mp3 -> annoyed
	// 8.mp3 -> anxious
	// 9.mp3 -> idle
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

	//Parallel functions

	// Create a task that will be executed in the main_loop() function,
	// with name "main", stack size 10000, NULL as parameter, priority 1,
	// handled by Task1 and executed on core 0
	//xTaskCreatePinnedToCore(main_loop, "main", 10000, NULL, 1, &Task1, 0);
	//delay(500);

	xTaskCreatePinnedToCore(network_loop, "network", 10000, NULL, 1, &Task2, 1);
	delay(500);

}

void loop() {
	// Handle emotions
	while (true) {
		//Read from serial interface to bypass WiFi for debugging
		if (Serial.available()) {
			incomingData = std::string(Serial.readStringUntil('\n').c_str());
			Serial.print("Serial data = ");
			Serial.println(incomingData.c_str());
			messages.push_front(incomingData);
		} 
		if (!messages.empty()) {
			incomingData = std::string(messages.back());
			messages.pop_back();
			handle(incomingData);
		} else {
			// no message, go to idle
			emote_idle();
		}
		//delay(10);
	}
}


void network_loop(void* pvParameters) {
  // Handle connection and receive from WiFi (updating state)
  	while (true) {

		if (WiFi.status() != WL_CONNECTED) {
			initConnection();
		}

		if (client.available()) {
			incomingData = std::string(client.readString().c_str());
			Serial.print("Server says: ");
			Serial.println(incomingData.c_str());

			if (incomingData == "GF") {
				//god kills any sequence of reactions
				//set flag to prevent any reaction and clear the queue of reactions
				kill_signal = true;
			}
			// add to queue
			messages.push_front(incomingData);
		}
  }
}


void initConnection() {

	Serial.println();
	Serial.print("Connecting to ");
	Serial.println(ssid);

	// WiFi connection
	WiFi.begin(ssid, password);

	while (WiFi.status() != WL_CONNECTED) {
		//delay(100);
		// use millis() function to wait for 100ms
		unsigned long time_now = millis();
		while (millis() - time_now < 100) {
			// do nothing
		}
		
	}
	Serial.println();
	Serial.println("WiFi connected");
	Serial.println("IP address: ");
	Serial.println(WiFi.localIP());

	// Server connection
	Serial.println();
	Serial.println("Connecting to Server...");
	while (!client.connect(server_ip, server_port)) {
		unsigned long time_now = millis();
		while (millis() - time_now < 100) {
			// do nothing
		}
	}
	Serial.println("Server connected");
	Serial.println();
}



void handle(std::string incomingData) {
	Serial.print("handling ");
	Serial.println(incomingData.c_str());
	// Debugging the emotions and motors via serial
	if (incomingData[0] == 'm') { // move motors for debugging purposes
		int angle;
		if (incomingData[1] == 'x') { // motor x axis
			angle = std::stoi(incomingData.substr(2, 3));
			neckSphereX.write(angle);
		} else if (incomingData[1] == 'y') { // motor y axis
			angle = std::stoi(incomingData.substr(2, 3));
			neckSphereY.write(angle);
		} else if (incomingData[1] == 'z') { // motor z axis
			angle = std::stoi(incomingData.substr(2, 3));
			neckBaseZ.write(angle);
		}
		unsigned long time_now = millis();
		while (millis() - time_now < 3000) {
			// do nothing
		}
	} else if (incomingData.substr(0, 4) == "play") { // play audio file
		myDFPlayer.play(std::stoi(incomingData.substr(4, 1)));
	} else if (incomingData == "happy_") {
		emote_happy();
	} else if (incomingData == "angry_") {
		emote_angry();
	} else if (incomingData == "annoyed_") {
		emote_annoyed();
	} else if (incomingData == "anxious_") {
		emote_anxious();
	} else if (incomingData == "sad_") {
		emote_sad();
	} else if (incomingData == "surprised_") {
		emote_surprised();
	} else if (incomingData == "idle_") {
		emote_idle();
	} else if (incomingData == "G1") { // Rocco message
		lookAt(1);
		god_message = "G1";
	} else if (incomingData == "G2") { // Rocco message
		lookAt(1);
		god_message = "G2";
	} else if (incomingData == "G3") { // Eva message
		god_message = "G3";
		// read tts message with audio
		myDFPlayer.play(2);
		// send message to server with GG after audio is finished playing
		while (myDFPlayer.readState() != 0) {
			// spin wait
		}
		client.println("GG");
		// send message with reaction
		client.println("2L13");
		// emote emotion anxoious
		emote_anxious();
	} else if (incomingData == "G4") { // Eva message
		god_message = "G4";
		// read tts message with audio
		myDFPlayer.play(1);
		// send message to server with GG after audio is finished playing
		while (myDFPlayer.readState() != 0) {
			// spin wait
		}
		client.println("GG");
		// send message with reaction
		client.println("2C13");
		// emote emotion angry
		emote_angry();
	} else if (incomingData == "G5") { // Lele message
		lookAt(3);
		god_message = "G5";
	} else if (incomingData == "G6") { // Lele message
		lookAt(3);
		god_message = "G6";
	} else if (incomingData == "G7") { // Carlotta Message
		lookAt(4);
		god_message = "G7";
	} else if (incomingData == "G8") { // Carlotta Message
		lookAt(4);
		god_message = "G8";
	} else if (incomingData == "G9") {	// Peppe message
		lookAt(5);
		god_message = "G9";
	} else if (incomingData == "GA") { // Peppe message
		lookAt(5);
		god_message = "GA";
	} else if (incomingData == "GB") { // Bianca message
		lookAt(6);
		god_message = "GB";
	} else if (incomingData == "GC") { // Bianca message
		lookAt(6);
		god_message = "GC";
	} else if (incomingData == "GD") { // Cosimo message
		lookAt(7);
		god_message = "GD";
	} else if (incomingData == "GE") { // Cosimo message
		lookAt(7);
		god_message = "GE";
	} else if (incomingData == "GF") { // God kills
		// set messages deque to empty
		messages.clear();
		kill_signal = false;
		emote_idle();
	} else if (incomingData == "GG") { // react to the God messages
		if (god_message == "G1") {
			emote_happy();
		} else if (god_message == "G2") {
			emote_angry();
		} else if (god_message == "G5") {
			emote_happy();
		} else if (god_message == "G6") {
			emote_angry();
		} else if (god_message == "G7") {
			emote_happy();
		} else if (god_message == "G8") {
			emote_angry();
		} else if (god_message == "G9") {
			emote_happy();
		} else if (god_message == "GA") {
			emote_angry();
		} else if (god_message == "GB") {
			emote_happy();
		} else if (god_message == "GC") {
			emote_angry();
		} else if (god_message == "GD") {
			emote_happy();
		} else if (god_message == "GE") {
			emote_angry();
		}
		god_message == "G0";
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
		if (from == 1) {
			if (emotion == 'C' || emotion == 'J') {// angry or annoyed
				if (to == 2) {
					emote_surprised();
				} else {
					emote_angry();
				}
			} else if (emotion == 'B') { // happy
				emote_happy();
			} else if (emotion == 'G') { // afraid
				emote_surprised();
			} else if (emotion == 'E') { // sad
				emote_sad();
			}

		} else if (from == 3) {
			if (emotion == 'C') { // angry
				emote_annoyed();
			} else if (emotion == 'H') { // cautious
				emote_angry();
			} else if (emotion == 'D') { // shocked
				emote_surprised();
			}
			
		} else if (from == 4) {
			if (emotion == 'D') { // shocked
				emote_surprised();
			} else if (emotion == 'C') { // angry
				emote_angry();
			} else if (emotion == 'L') { // anxious
				emote_anxious();
			} else if (emotion == 'G') { // afraid
				emote_sad();
			}
		} else if (from == 5) {
			if (emotion == 'C') { // angry
				emote_happy();
			} else if (emotion == 'H') { // cautious
				emote_surprised();
			} else if (emotion == 'B') { // happy
				emote_happy();
			}
		} else if (from == 6) {
 			if (emotion == 'E') { // sad
				emote_sad();
			} else if (emotion == 'B') { // happy
				emote_happy();
			} else if (emotion == 'D') { // shocked
				emote_sad();
			}
		} else if (from == 7) {
			if (emotion == 'H') { // cautious
				emote_annoyed();
			} else if (emotion == 'G') { // afraid
				emote_surprised();
			} else if (emotion == 'E') { // sad
				emote_sad();
			} else if (emotion == 'B') { // happy
				emote_happy();
			} else if (emotion == 'D') { // shocked
				emote_sad();
			}
		}

		// send the server the robot reaction to the other robot
		// client.print("2A51") // eva is happy towards group 5 at intensity 1

		//client.flush(); // ?
		
	}
	//emote_idle(); // after emotion finishes, go back to idle

}


// look at the robot who is making the emotion with its number
void lookAt(int robotNumber) {
	if (robotNumber == 1) {
		neckBaseZ.write(180); // robot sits on its left
	} else if (robotNumber == 3) {
		neckBaseZ.write(0); // robot sits on its right
	} else if (robotNumber == 4) {
		neckBaseZ.write(22);
	} else if (robotNumber == 5) {
		neckBaseZ.write(44);
	} else if (robotNumber == 6) {
		neckBaseZ.write(66);
	} else if (robotNumber == 7) {
		neckBaseZ.write(90); 
	}
}


void emote(MovementStruct movements, std::vector<std::vector<std::vector<uint8_t>>> &eyes_animation, int audio) {
	// compute length of eyes animation
	int eyes_animation_length = eyes_animation.size();
	if (audio != 9) myDFPlayer.play(audio);

	//iterate over every frame of the animation
	for(int frame = 0; frame < movements.maxFrames; frame++){
		neckBaseZ.write(movements.neckBaseAngles[frame % movements.neckBaseFrames]);
		neckSphereX.write(movements.neckSphereXAngles[frame % movements.neckSphereXFrames]);
		neckSphereY.write(movements.neckSphereYAngles[frame % movements.neckSphereYFrames]);
		leftEarServo.write(movements.leftEarAngles[frame % movements.leftEarFrames]);
		rightEarServo.write(movements.rightEarAngles[frame % movements.rightEarFrames]);
		leftEyebrowServo.write(movements.leftEyebrowAngles[frame % movements.leftEyebrowFrames]);
		rightEyebrowServo.write(movements.rightEyebrowAngles[frame % movements.rightEyebrowFrames]);
		// iterate over every column of the eyes matrices
		for(int i=0; i < 16; i++) {
			rightEye.setColumn(i, eyes_animation[frame % eyes_animation_length][0][i]);
			leftEye.setColumn(i, eyes_animation[frame % eyes_animation_length][1][i]);
		}
		if (!kill_signal) {
			if (frame == movements.maxFrames / 2) {
				if (audio != 9) myDFPlayer.play(audio);
			}
			// delay for the duration of the frame
			unsigned long time_now = millis();
			while (millis() < time_now + 75){
				//wait approx. 100 ms
			}
		} else return;
	}

}


void emote_idle() {
	emote(idleMovements, idle, 9);
}

void emote_happy() {
  emote(happyMovements, happy, 4);
}

void emote_sad() {
	emote(sadMovements, sad, 5);
}

void emote_angry() {
	emote(angryMovements, angry, 3);
}

void emote_surprised() {
	emote(surprisedMovements, surprised, 6);
}

void emote_annoyed() {
	//(annoyedMovements, annoyed, 7);
}

void emote_anxious() {
	emote(anxiousMovements, anxious, 8);
}




