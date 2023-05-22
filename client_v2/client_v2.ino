#include "emotions.h"

// data incoming from server or for debugging via serial
std::string incomingData;

TaskHandle_t Task1;
TaskHandle_t Task2;


// Network details
const char* ssid_test = "test";
const char* password_test = "test1234567!";
const char* server_ip_test = "192.168.1.5";
int server_port_test = 800;

const char* ssid = "Triskarone";
const char* password = "triskarone";
const char* server_ip = "192.168.1.101";
int server_port = 8090;

static WiFiClient client;

unsigned int from;
char emotion;
unsigned int to;
unsigned int intensity;
std::string god_message =  "G0";

// create queue for messages received from server in cpp
std::deque<std::string> messages;
bool kill_signal = false;

void emote(MovementStruct movements, std::vector<std::vector<std::vector<uint8_t>>> &emotion, int audio, int lookAtRobot);
void reset_state(); // used to reset to initial angles
int getAngle(int lookAtRobot);
void lookAt(int robotNumber);
// emote functions list
void emote_idle();
void emote_happy(int lookAtRobot);
void emote_sad(int lookAtRobot);
void emote_angry(int lookAtRobot);
void emote_surprised(int lookAtRobot);
void emote_annoyed(int lookAtRobot);
void emote_anxious(int lookAtRobot);


void initMouth(){
	pinMode(PIN_MOUTH_RED,  OUTPUT);
	pinMode(PIN_MOUTH_GREEN,  OUTPUT);
	pinMode(PIN_MOUTH_BLUE,  OUTPUT);
	analogWrite(PIN_MOUTH_RED, 0);
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
	myDFPlayer.volume(30);
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
			incomingData = std::string(client.readStringUntil('\n').c_str());
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
		reset_state();
	} else if (incomingData.substr(0, 4) == "play") { // play audio file for debugging purposes
		myDFPlayer.play(std::stoi(incomingData.substr(4, 1)));
	} else if (incomingData.substr(0, 6) == "happy_") {
		emote_happy(std::stoi(incomingData.substr(6, 1)));
		reset_state();
	} else if (incomingData.substr(0, 6) == "angry_") {
		emote_angry(std::stoi(incomingData.substr(6, 1)));
		reset_state();
	} else if (incomingData.substr(0, 8) == "annoyed_") {
		emote_annoyed(std::stoi(incomingData.substr(8, 1)));
		reset_state();
	} else if (incomingData.substr(0, 8) == "anxious_") {
		emote_anxious(std::stoi(incomingData.substr(8, 1)));
		reset_state();
	} else if (incomingData.substr(0, 4) == "sad_") {
		emote_sad(std::stoi(incomingData.substr(4, 1)));
		reset_state();
	} else if (incomingData.substr(0, 10) == "surprised_") {
		emote_surprised(std::stoi(incomingData.substr(10, 1)));
		reset_state();
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
			// blinking mouth when the robot talks
			setMouthColor(255, 100, 0);
			unsigned long time_now = millis();
			while (millis() - time_now < 250) {
				// do nothing
			}
			setMouthColor(0, 0, 0);
			time_now = millis();
			while (millis() - time_now < 250) {
				// do nothing
			}
		}
		client.print("GG\n");
		// send message with reaction
		client.print("2L13\n");
		// emote emotion anxoious
		emote_anxious(1);
		reset_state();
	} else if (incomingData == "G4") { // Eva message
		god_message = "G4";
		// read tts message with audio
		myDFPlayer.play(1);
		// send message to server with GG after audio is finished playing
		while (myDFPlayer.readState() != 0) {
			// blinking mouth when the robot talks
			setMouthColor(255, 100, 0);
			unsigned long time_now = millis();
			while (millis() - time_now < 250) {
				// do nothing
			}
			setMouthColor(0, 0, 0);
			time_now = millis();
			while (millis() - time_now < 250) {
				// do nothing
			}
		}
		client.print("GG\n");
		// send message with reaction
		client.print("2C13\n");
		// emote emotion angry
		emote_angry(1);
		reset_state();
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
			client.print("2B13\n");
			emote_happy(1);
		} else if (god_message == "G2") {
			client.print("2C13\n");
			emote_angry(1);
		} else if (god_message == "G5") {
			client.print("2I33\n");
			emote_surprised(3);
		} else if (god_message == "G6") {
			client.print("2B33\n");
			emote_happy(3);
		} else if (god_message == "G7") {
			client.print("2I43\n");
			emote_surprised(4);
		} else if (god_message == "G8") {
			client.print("2C43\n");
			emote_angry(4);
		} else if (god_message == "G9") {
			client.print("2B53\n");
			emote_happy(5);
		} else if (god_message == "GA") {
			client.print("2C53\n");
			emote_angry(5);
		} else if (god_message == "GB") {
			client.print("2I63\n");
			emote_surprised(6);
		} else if (god_message == "GC") {
			client.print("2C63\n");
			emote_happy(6);
		} else if (god_message == "GD") {
			client.print("2L73\n");
			emote_anxious(7);
		} else if (god_message == "GE") {
			client.print("2E73\n");
			emote_sad(7);
		}
		god_message == "G0";
		reset_state();
	} else if (incomingData.length() == 4) { // reactions of the other robots
		
		from = incomingData[0] - '0';
		emotion = incomingData[1];
		to = incomingData[2] - '0';
		intensity = incomingData[3] - '0';

		// the neck rotates in the direction of the robot who is making the emotion
		//lookAt(from);

		// reactions
		// A = idle, B = happy, C = angry, D = shocked, E = sad, F = relaxed, G = afraid, H = cautious,
		// I = surprised, J = annoyed, K = embarrassed, L = anxious
		if (from == 1) {
			if (emotion == 'C' || emotion == 'J') {// angry or annoyed
				if (to == 2) {
					client.print("2I13\n");
					emote_surprised(1);
				} else {
					client.print("2C11\n");
					emote_angry(1);
				}
			} else if (emotion == 'B') { // happy
				client.print("2B13\n");
				emote_happy(1);
			} else if (emotion == 'G') { // afraid
				client.print("2I13\n");
				emote_surprised(1);
			} else if (emotion == 'E') { // sad
				client.print("2E13\n");
				emote_sad(1);
			} else if (emotion == 'K') { // embarassed
				client.print("2J13\n");
				emote_annoyed(1);
			}

		} else if (from == 3) {
			if (emotion == 'C') { // angry
				client.print("2J33\n");
				emote_annoyed(3);
			} else if (emotion == 'H') { // cautious
				client.print("2C33\n");
				emote_angry(3);
			} else if (emotion == 'D') { // shocked
				client.print("2I33\n");
				emote_surprised(3);
			}
			
		} else if (from == 4) {
			if (emotion == 'D') { // shocked
				client.print("2I43\n");
				emote_surprised(4);
			} else if (emotion == 'C') { // angry
				client.print("2C43\n");
				emote_angry(4);
			} else if (emotion == 'L') { // anxious
				client.print("2L43\n");
				emote_anxious(4);
			} else if (emotion == 'G') { // afraid
				client.print("2E43\n");
				emote_sad(4);
			}
		} else if (from == 5) {
			if (emotion == 'C') { // angry
				client.print("2B53\n");
				emote_happy(5);
			} else if (emotion == 'H') { // cautious
				client.print("2I53\n");
				emote_surprised(5);
			} else if (emotion == 'B') { // happy
				client.print("2B53\n");
				emote_happy(5);
			} else if (emotion == 'K') { // embarrassed
				client.print("2B53\n");
				emote_happy(5);
			}
		} else if (from == 6) {
 			if (emotion == 'E') { // sad
				client.print("2E63\n");
				emote_sad(6);
			} else if (emotion == 'B') { // happy
				client.print("2B63\n");
				emote_happy(6);
			} else if (emotion == 'D') { // shocked
				client.print("2E63\n");
				emote_sad(6);
			} else if (emotion == 'K') { // embarrassed
				client.print("2L63\n");
				emote_anxious(6);

			}
		} else if (from == 7) {
			if (emotion == 'H') { // cautious
				client.print("2J73\n");
				emote_annoyed(7);
			} else if (emotion == 'G') { // afraid
				client.print("2J73\n");
				emote_annoyed(7);
			} else if (emotion == 'E') { // sad
				client.print("2E73\n");
				emote_sad(7);
			} else if (emotion == 'B') { // happy
				client.print("2B73\n");
				emote_happy(7);
			} else if (emotion == 'D') { // shocked
				client.print("2E73\n");
				emote_sad(7);
			}
		}

		// send the server the robot reaction to the other robot
		// client.print("2A51") // eva is happy towards group 5 at intensity 1

		//client.flush(); // ?
		reset_state();
	}
}


// look at the robot who is making the emotion with its number
void lookAt(int robotNumber) {
	neckBaseZ.write(getAngle(robotNumber));
}

int getAngle(int lookAtRobot) {
	if (lookAtRobot == 1) {
		return 180;  // robot sits on its left
	} else if (lookAtRobot == 3) {
		return 0; // robot sits on its right
	} else if (lookAtRobot == 4) {
		return 22;
	} else if (lookAtRobot == 5) {
		return 44;
	} else if (lookAtRobot == 6) {
		return 66;
	} else if (lookAtRobot == 7) {
		return 88;
	}
	return 0;
}


void emote(MovementStruct movements, std::vector<std::vector<std::vector<uint8_t>>> &eyes_animation, int audio, int lookAtRobot) {
	// compute length of eyes animation
	int eyes_animation_length = eyes_animation.size();
	if (audio != 9) myDFPlayer.play(audio); // idle audio doesn't play

	// if idle, blink the eyes only once in a while
	// if any other emotion, always play the eyes animation
	if (audio != 9 || (rand() % 300 + 1) == 1) {
		setMouthColor(255, 0, 0);
		
		//iterate over every frame of the animation
		for(int frame = 0; frame < movements.maxFrames; frame++) {
			// apply the movements to the servos
			neckSphereX.write(movements.neckSphereXAngles[frame % movements.neckSphereXFrames]);
			neckSphereY.write(movements.neckSphereYAngles[frame % movements.neckSphereYFrames]);
			leftEarServo.write(movements.leftEarAngles[frame % movements.leftEarFrames]);
			rightEarServo.write(movements.rightEarAngles[frame % movements.rightEarFrames]);
			leftEyebrowServo.write(movements.leftEyebrowAngles[frame % movements.leftEyebrowFrames]);
			rightEyebrowServo.write(movements.rightEyebrowAngles[frame % movements.rightEyebrowFrames]);


			// neck base rotation on z axis needs to be adjusted relative to the robot towards which the robot is looking
			if (audio != 5 && audio != 8 && audio != 9) { //if it's not sad or anxious emotion:
				neckBaseZ.write(movements.neckBaseAngles[frame % movements.neckBaseFrames] - 90 + getAngle(lookAtRobot));
			} else if (audio == 8 || audio == 9) { // anxious emotion or in idle
				neckBaseZ.write(movements.neckBaseAngles[frame % movements.neckBaseFrames]);
			} else if (audio == 5) { // sad emotion needs calibrated angles
				if (getAngle(lookAtRobot) == 180) {
					neckBaseZ.write(movements.neckBaseAngles[frame % movements.neckBaseFrames] - 90 + 150);
				} else if (getAngle(lookAtRobot) == 0 || getAngle(lookAtRobot) == 22) {
					neckBaseZ.write(movements.neckBaseAngles[frame % movements.neckBaseFrames] - 90 + 30);
				} else {
					neckBaseZ.write(movements.neckBaseAngles[frame % movements.neckBaseFrames] - 90 + getAngle(lookAtRobot));
				}
			}
			
			
			for(int i=0; i < 16; i++) {
				// iterate over every column of the eyes matrices
				rightEye.setColumn(i, eyes_animation[frame % eyes_animation_length][0][i]);
				leftEye.setColumn(i, eyes_animation[frame % eyes_animation_length][1][i]);
			}
		
			
			if (!kill_signal) {
				if (frame == movements.maxFrames / 2) { // at half of the animation, replays the audio
					if (audio != 9) myDFPlayer.play(audio); // idle audio doesn't play
				}
				// delay for the duration of the frame
				unsigned long time_now = millis();
				while (millis() < time_now + 75){
				}
			} else return; // if GF signal is received, stop the animation
		}
	} else {
		// if it skips idle animation, then wait for 75ms anyway
		// delay for the duration of the frame
		unsigned long time_now = millis();
		while (millis() < time_now + 75){
		}
	}
}

void reset_state() {
	// reset the servos to their initial position
	neckBaseZ.write(NECK_BASE_START_POS);
	neckSphereX.write(NECK_SPHERE_X_START_POS);
	neckSphereY.write(NECK_SPHERE_Y_START_POS);
	leftEarServo.write(LEFT_EAR_START_POS);
	rightEarServo.write(RIGHT_EAR_START_POS);
	leftEyebrowServo.write(LEFT_EYEBROW_START_POS);
	rightEyebrowServo.write(RIGHT_EYEBROW_START_POS);
	
	// reset the eyes to their initial position
	
	for(int i=0; i < 16; i++) {
		// iterate over every column of the eyes matrices
		rightEye.setColumn(i, idle[0][0][i]);
		leftEye.setColumn(i, idle[0][1][i]);
	}

	setMouthColor(0, 0, 0);

}

void emote_idle() {
	emote(idleMovements, idle, 9, 0);
	setMouthColor(0, 0, 0);
}

void emote_happy(int lookAtRobot) {
  emote(happyMovements, happy, 4, lookAtRobot);
}

void emote_sad(int lookAtRobot) {
	emote(sadMovements, sad, 5, lookAtRobot);
}

void emote_angry(int lookAtRobot) {
	emote(angryMovements, angry, 3, lookAtRobot);
}

void emote_surprised(int lookAtRobot) {
	emote(surprisedMovements, surprised, 6, lookAtRobot);
}

void emote_annoyed(int lookAtRobot) {
	emote(annoyedMovements, annoyed, 7, lookAtRobot);
}

void emote_anxious(int lookAtRobot) {
	emote(anxiousMovements, anxious, 8, lookAtRobot);
}




