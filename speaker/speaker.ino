// RXD2 pin = GPIO 16
// TXD2 pin = GPIO 17

// wirings:
// https://github.com/NicolasBrondin/flower-player/raw/master/schema.jpg
// tx pin mp3 -> gpio 17 esp32
// rx pin mp3 -> gpio 16 esp32
// from left to right, with mp3 player sd card side on the left, on the top:
// spk2 mp3 -> speaker negative
// gnd mp3 -> gnd
// spk1 mp3 -> speaker positive
// skip x2 pins
// tx mp3
// rx mp3
// vcc mp3


#include <DFRobotDFPlayerMini.h>
#include <HardwareSerial.h>

HardwareSerial mySoftwareSerial(1);

DFRobotDFPlayerMini myDFPlayer;

int songNumber = 1;
int songsCount = 1;

void setup() {
	
	mySoftwareSerial.begin(9600, SERIAL_8N1, 17, 16);  // speed, type, TX, RX
	
	Serial.begin(115200);
	
	Serial.println();
	Serial.println("DFRobot DFPlayer Mini");
	Serial.println("Initializing DFPlayer");
	
	while(!myDFPlayer.begin(mySoftwareSerial)) {
		Serial.println("Unable to begin");
	}
	Serial.println("DFPlayer Mini online.");

	songsCount = myDFPlayer.readFileCountsInFolder(2);
	myDFPlayer.volume(10);
	myDFPlayer.EQ(DFPLAYER_EQ_NORMAL);
	myDFPlayer.outputDevice(DFPLAYER_DEVICE_SD);
	myDFPlayer.playFolder(1, 1);  
	myDFPlayer.start();
}

void loop() {
	if (myDFPlayer.available()) {
		uint8_t type = myDFPlayer.readType();
		if(type == DFPlayerPlayFinished){
			myDFPlayer.playFolder(1, 1);  
		}
	}
	
}

void nextSong(){
	songNumber = ((songNumber)%(songsCount-1))+1; //Needs to be checked, might jump songs
	myDFPlayer.playFolder(2, songNumber);
}
