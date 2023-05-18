#include <MD_MAX72xx.h> // led matrices libraries
#include <ESP32Servo.h>
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <DFRobotDFPlayerMini.h> // mp3 player library
#include <HardwareSerial.h> // communication with mp3 player
// standard cpp libraries
#include <vector>
#include <deque>
#include <string>

// initializations
void initConnection();
void initEars();
void initEyebrows();
void initMouth();
void initNeck();
void initEyes();
void initSpeaker();
void setMouthColor(uint8_t R, uint8_t G, uint8_t B);

// handling messages and data
void handle(std::string incomingData);

//Task Functions
void main_loop(void* pvParameters);
void network_loop(void* pvParameters);

// --- START MOUTH ---
#define PIN_MOUTH_RED    2
#define PIN_MOUTH_GREEN  4
#define PIN_MOUTH_BLUE   12
// --- END MOUTH ---

// --- START EYES ---
#define PIN_LEFT_EYE_DATA 25
#define PIN_LEFT_EYE_CS 33
#define PIN_LEFT_EYE_CLK 32
#define PIN_RIGHT_EYE_DATA 14
#define PIN_RIGHT_EYE_CS 27
#define PIN_RIGHT_EYE_CLK 26
#define LED_MATRIX_TYPE MD_MAX72XX::FC16_HW
#define MATRIX_PER_EYE 2
#define EYES_INTENSITY 8

MD_MAX72XX leftEye = MD_MAX72XX(LED_MATRIX_TYPE, PIN_LEFT_EYE_DATA, PIN_LEFT_EYE_CLK, PIN_LEFT_EYE_CS, MATRIX_PER_EYE);
MD_MAX72XX rightEye = MD_MAX72XX(LED_MATRIX_TYPE, PIN_RIGHT_EYE_DATA, PIN_RIGHT_EYE_CLK, PIN_RIGHT_EYE_CS, MATRIX_PER_EYE);
// --- END EYES ---

// --- START NECK ---
#define PIN_NECK_BASE 21
#define PIN_NECK_SPHERE_X 22
#define PIN_NECK_SPHERE_Y 23
#define NECK_BASE_START_POS 90 // rotates left to 0 degrees, right to 180 degrees
#define NECK_SPHERE_X_START_POS 75 // tilt upwards down to 20 degrees (for simmetry), downwards up to 130 degrees
#define NECK_SPHERE_Y_START_POS 35 // tilt left down to 0 degrees (strange problem), right up to 70 (for simmetry) degrees

Servo neckBaseZ;
Servo neckSphereX;
Servo neckSphereY;
// --- END NECK ---

// --- START EARS ---
#define PIN_LEFT_EAR 19
#define PIN_RIGHT_EAR 18
#define LEFT_EAR_START_POS 90
#define RIGHT_EAR_START_POS 150

Servo leftEarServo;
Servo rightEarServo;
// --- END EARS ---

// --- START EYEBROWS ---
#define PIN_LEFT_EYEBROW 17
#define PIN_RIGHT_EYEBROW 16
#define LEFT_EYEBROW_START_POS 100
#define RIGHT_EYEBROW_START_POS 100

Servo leftEyebrowServo;
Servo rightEyebrowServo;
// --- END EYEBROWS ---

// --- START SPEAKER ---
HardwareSerial mySoftwareSerial(1); // serial communication with speaker
DFRobotDFPlayerMini myDFPlayer; // mp3 player object

#define PIN_Tx 5
#define PIN_Rx 15
// --- END SPEAKER ---

// definition of the structure for animating the robot
typedef struct {
	int maxFrames;
	int neckBaseFrames;
	int neckBaseAngles[100];
	int neckSphereXFrames;
	int neckSphereXAngles[100];
	int neckSphereYFrames;
	int neckSphereYAngles[100];
	int leftEarFrames;
	int leftEarAngles[100];
	int rightEarFrames;
	int rightEarAngles[100];
	int leftEyebrowFrames;
	int leftEyebrowAngles[100];
	int rightEyebrowFrames;
	int rightEyebrowAngles[100];
} MovementStruct;


// basic animation for demo purpose
MovementStruct relaxedMovements = {
	.maxFrames = 60,
	.neckBaseFrames = 60,
	.neckBaseAngles = {40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100, 105, 110, 115, 120, 125, 130, 135, 140, 140, 140, 140, 140, 140, 140, 140, 140, 140, 140, 135, 130, 125, 120, 115, 110, 105, 100, 95, 90, 85, 80, 75, 70, 65, 60, 55, 50, 45},
	.neckSphereXFrames = 1,
	.neckSphereXAngles = {75},
	.neckSphereYFrames = 54,
	.neckSphereYAngles = {55, 55, 55, 57, 59, 61, 63, 65, 67, 69, 69, 69, 67, 65, 63, 61, 59, 57, 55, 53, 51, 49, 47, 45, 43, 41, 41, 41, 41, 41, 41, 41, 41, 43, 45, 47, 49, 51, 53, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55},
	.leftEarFrames = 27,
	.leftEarAngles = {80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 150, 150, 150, 150, 80, 80, 80, 80, 150, 150, 150, 150, 80, 80, 80, 80},
	.rightEarFrames = 27,
	.rightEarAngles = {150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 80, 80, 80, 80, 150, 150, 150, 150, 80, 80, 80, 80, 150, 150, 150, 150},
	.leftEyebrowFrames = 1,
	.leftEyebrowAngles = {90},
	.rightEyebrowFrames = 1,
	.rightEyebrowAngles = {90}
};

// basic animation for demo purpose
MovementStruct idleMovements = {
	.maxFrames = 60,
	.neckBaseFrames = 1,
	.neckBaseAngles = {90},
	.neckSphereXFrames = 1,
	.neckSphereXAngles = {75},
	.neckSphereYFrames = 1,
	.neckSphereYAngles = {35},
	.leftEarFrames = 1,
	.leftEarAngles = {90},
	.rightEarFrames = 1,
	.rightEarAngles = {150},
	.leftEyebrowFrames = 1,
	.leftEyebrowAngles = {90},
	.rightEyebrowFrames = 1,
	.rightEyebrowAngles = {90}
};


// basic emotion for being calm
std::vector<std::vector<std::vector<unsigned int>>> idle = {
	{{0x0, 0x0, 0x7E, 0xFF, 0xFF, 0xFF, 0xE1, 0xE1, 0xE1, 0xE1, 0xFF, 0xFF, 0xFF, 0x7E, 0x0, 0x0},{0x0, 0x0, 0x7E, 0xFF, 0xFF, 0xFF, 0xE1, 0xE1, 0xE1, 0xE1, 0xFF, 0xFF, 0xFF, 0x7E, 0x0, 0x0}},
	{{0x0, 0x0, 0x7E, 0xFF, 0xFF, 0xFF, 0xE1, 0xE1, 0xE1, 0xE1, 0xFF, 0xFF, 0xFF, 0x7E, 0x0, 0x0},{0x0, 0x0, 0x7E, 0xFF, 0xFF, 0xFF, 0xE1, 0xE1, 0xE1, 0xE1, 0xFF, 0xFF, 0xFF, 0x7E, 0x0, 0x0}},
	{{0x0, 0x0, 0x3C, 0x7E, 0x7E, 0x7E, 0x62, 0x62, 0x62, 0x62, 0x7E, 0x7E, 0x7E, 0x3C, 0x0, 0x0},{0x0, 0x0, 0x3C, 0x7E, 0x7E, 0x7E, 0x62, 0x62, 0x62, 0x62, 0x7E, 0x7E, 0x7E, 0x3C, 0x0, 0x0}},
	{{0x0, 0x0, 0x3C, 0x7E, 0x7E, 0x7E, 0x62, 0x62, 0x62, 0x62, 0x7E, 0x7E, 0x7E, 0x3C, 0x0, 0x0},{0x0, 0x0, 0x3C, 0x7E, 0x7E, 0x7E, 0x62, 0x62, 0x62, 0x62, 0x7E, 0x7E, 0x7E, 0x3C, 0x0, 0x0}},
	{{0x0, 0x0, 0x18, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x18, 0x0, 0x0},{0x0, 0x0, 0x18, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x18, 0x0, 0x0}},
	{{0x0, 0x0, 0x18, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x18, 0x0, 0x0},{0x0, 0x0, 0x18, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x18, 0x0, 0x0}},
	{{0x0, 0x0, 0x3C, 0x7E, 0x7E, 0x7E, 0x62, 0x62, 0x62, 0x62, 0x7E, 0x7E, 0x7E, 0x3C, 0x0, 0x0},{0x0, 0x0, 0x3C, 0x7E, 0x7E, 0x7E, 0x62, 0x62, 0x62, 0x62, 0x7E, 0x7E, 0x7E, 0x3C, 0x0, 0x0}},
	{{0x0, 0x0, 0x3C, 0x7E, 0x7E, 0x7E, 0x62, 0x62, 0x62, 0x62, 0x7E, 0x7E, 0x7E, 0x3C, 0x0, 0x0},{0x0, 0x0, 0x3C, 0x7E, 0x7E, 0x7E, 0x62, 0x62, 0x62, 0x62, 0x7E, 0x7E, 0x7E, 0x3C, 0x0, 0x0}},
	{{0x0, 0x0, 0x7E, 0xFF, 0xFF, 0xFF, 0xE1, 0xE1, 0xE1, 0xE1, 0xFF, 0xFF, 0xFF, 0x7E, 0x0, 0x0},{0x0, 0x0, 0x7E, 0xFF, 0xFF, 0xFF, 0xE1, 0xE1, 0xE1, 0xE1, 0xFF, 0xFF, 0xFF, 0x7E, 0x0, 0x0}},
	{{0x0, 0x0, 0x7E, 0xFF, 0xFF, 0xFF, 0xE1, 0xE1, 0xE1, 0xE1, 0xFF, 0xFF, 0xFF, 0x7E, 0x0, 0x0},{0x0, 0x0, 0x7E, 0xFF, 0xFF, 0xFF, 0xE1, 0xE1, 0xE1, 0xE1, 0xFF, 0xFF, 0xFF, 0x7E, 0x0, 0x0}},
	{{0x0, 0x0, 0x3C, 0x7E, 0x7E, 0x7E, 0x62, 0x62, 0x62, 0x62, 0x7E, 0x7E, 0x7E, 0x3C, 0x0, 0x0},{0x0, 0x0, 0x3C, 0x7E, 0x7E, 0x7E, 0x62, 0x62, 0x62, 0x62, 0x7E, 0x7E, 0x7E, 0x3C, 0x0, 0x0}},
	{{0x0, 0x0, 0x3C, 0x7E, 0x7E, 0x7E, 0x62, 0x62, 0x62, 0x62, 0x7E, 0x7E, 0x7E, 0x3C, 0x0, 0x0},{0x0, 0x0, 0x3C, 0x7E, 0x7E, 0x7E, 0x62, 0x62, 0x62, 0x62, 0x7E, 0x7E, 0x7E, 0x3C, 0x0, 0x0}},
	{{0x0, 0x0, 0x18, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x18, 0x0, 0x0},{0x0, 0x0, 0x18, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x18, 0x0, 0x0}},
	{{0x0, 0x0, 0x18, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x18, 0x0, 0x0},{0x0, 0x0, 0x18, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x18, 0x0, 0x0}},
	{{0x0, 0x0, 0x3C, 0x7E, 0x7E, 0x7E, 0x62, 0x62, 0x62, 0x62, 0x7E, 0x7E, 0x7E, 0x3C, 0x0, 0x0},{0x0, 0x0, 0x3C, 0x7E, 0x7E, 0x7E, 0x62, 0x62, 0x62, 0x62, 0x7E, 0x7E, 0x7E, 0x3C, 0x0, 0x0}},
	{{0x0, 0x0, 0x3C, 0x7E, 0x7E, 0x7E, 0x62, 0x62, 0x62, 0x62, 0x7E, 0x7E, 0x7E, 0x3C, 0x0, 0x0},{0x0, 0x0, 0x3C, 0x7E, 0x7E, 0x7E, 0x62, 0x62, 0x62, 0x62, 0x7E, 0x7E, 0x7E, 0x3C, 0x0, 0x0}},
	{{0x0, 0x0, 0x7E, 0xFF, 0xFF, 0xFF, 0xE1, 0xE1, 0xE1, 0xE1, 0xFF, 0xFF, 0xFF, 0x7E, 0x0, 0x0},{0x0, 0x0, 0x7E, 0xFF, 0xFF, 0xFF, 0xE1, 0xE1, 0xE1, 0xE1, 0xFF, 0xFF, 0xFF, 0x7E, 0x0, 0x0}},
	{{0x0, 0x0, 0x7E, 0xFF, 0xFF, 0xFF, 0xE1, 0xE1, 0xE1, 0xE1, 0xFF, 0xFF, 0xFF, 0x7E, 0x0, 0x0},{0x0, 0x0, 0x7E, 0xFF, 0xFF, 0xFF, 0xE1, 0xE1, 0xE1, 0xE1, 0xFF, 0xFF, 0xFF, 0x7E, 0x0, 0x0}},
	{{0x0, 0x0, 0x7E, 0xFF, 0xFF, 0xFF, 0xE1, 0xE1, 0xE1, 0xE1, 0xFF, 0xFF, 0xFF, 0x7E, 0x0, 0x0},{0x0, 0x0, 0x7E, 0xFF, 0xFF, 0xFF, 0xE1, 0xE1, 0xE1, 0xE1, 0xFF, 0xFF, 0xFF, 0x7E, 0x0, 0x0}},
	{{0x0, 0x0, 0x7E, 0xFF, 0xFF, 0xFF, 0xE1, 0xE1, 0xE1, 0xE1, 0xFF, 0xFF, 0xFF, 0x7E, 0x0, 0x0},{0x0, 0x0, 0x7E, 0xFF, 0xFF, 0xFF, 0xE1, 0xE1, 0xE1, 0xE1, 0xFF, 0xFF, 0xFF, 0x7E, 0x0, 0x0}},
	{{0x0, 0x0, 0x7E, 0xFF, 0xFF, 0xFF, 0xE1, 0xE1, 0xE1, 0xE1, 0xFF, 0xFF, 0xFF, 0x7E, 0x0, 0x0},{0x0, 0x0, 0x7E, 0xFF, 0xFF, 0xFF, 0xE1, 0xE1, 0xE1, 0xE1, 0xFF, 0xFF, 0xFF, 0x7E, 0x0, 0x0}},
	{{0x0, 0x0, 0x7E, 0xFF, 0xFF, 0xFF, 0xE1, 0xE1, 0xE1, 0xE1, 0xFF, 0xFF, 0xFF, 0x7E, 0x0, 0x0},{0x0, 0x0, 0x7E, 0xFF, 0xFF, 0xFF, 0xE1, 0xE1, 0xE1, 0xE1, 0xFF, 0xFF, 0xFF, 0x7E, 0x0, 0x0}},
	{{0x0, 0x0, 0x7E, 0xFF, 0xFF, 0xFF, 0xE1, 0xE1, 0xE1, 0xE1, 0xFF, 0xFF, 0xFF, 0x7E, 0x0, 0x0},{0x0, 0x0, 0x7E, 0xFF, 0xFF, 0xFF, 0xE1, 0xE1, 0xE1, 0xE1, 0xFF, 0xFF, 0xFF, 0x7E, 0x0, 0x0}},
	{{0x0, 0x0, 0x7E, 0xFF, 0xFF, 0xFF, 0xE1, 0xE1, 0xE1, 0xE1, 0xFF, 0xFF, 0xFF, 0x7E, 0x0, 0x0},{0x0, 0x0, 0x7E, 0xFF, 0xFF, 0xFF, 0xE1, 0xE1, 0xE1, 0xE1, 0xFF, 0xFF, 0xFF, 0x7E, 0x0, 0x0}},
	{{0x0, 0x0, 0x7E, 0xFF, 0xFF, 0xFF, 0xE1, 0xE1, 0xE1, 0xE1, 0xFF, 0xFF, 0xFF, 0x7E, 0x0, 0x0},{0x0, 0x0, 0x7E, 0xFF, 0xFF, 0xFF, 0xE1, 0xE1, 0xE1, 0xE1, 0xFF, 0xFF, 0xFF, 0x7E, 0x0, 0x0}},
	{{0x0, 0x0, 0x7E, 0xFF, 0xFF, 0xFF, 0xE1, 0xE1, 0xE1, 0xE1, 0xFF, 0xFF, 0xFF, 0x7E, 0x0, 0x0},{0x0, 0x0, 0x7E, 0xFF, 0xFF, 0xFF, 0xE1, 0xE1, 0xE1, 0xE1, 0xFF, 0xFF, 0xFF, 0x7E, 0x0, 0x0}},
	{{0x0, 0x0, 0x7E, 0xFF, 0xFF, 0xFF, 0xE1, 0xE1, 0xE1, 0xE1, 0xFF, 0xFF, 0xFF, 0x7E, 0x0, 0x0},{0x0, 0x0, 0x7E, 0xFF, 0xFF, 0xFF, 0xE1, 0xE1, 0xE1, 0xE1, 0xFF, 0xFF, 0xFF, 0x7E, 0x0, 0x0}},
	{{0x0, 0x0, 0x7E, 0xFF, 0xFF, 0xFF, 0xE1, 0xE1, 0xE1, 0xE1, 0xFF, 0xFF, 0xFF, 0x7E, 0x0, 0x0},{0x0, 0x0, 0x7E, 0xFF, 0xFF, 0xFF, 0xE1, 0xE1, 0xE1, 0xE1, 0xFF, 0xFF, 0xFF, 0x7E, 0x0, 0x0}},
	{{0x0, 0x0, 0x7E, 0xFF, 0xFF, 0xFF, 0xE1, 0xE1, 0xE1, 0xE1, 0xFF, 0xFF, 0xFF, 0x7E, 0x0, 0x0},{0x0, 0x0, 0x7E, 0xFF, 0xFF, 0xFF, 0xE1, 0xE1, 0xE1, 0xE1, 0xFF, 0xFF, 0xFF, 0x7E, 0x0, 0x0}},
	{{0x0, 0x0, 0x7E, 0xFF, 0xFF, 0xFF, 0xE1, 0xE1, 0xE1, 0xE1, 0xFF, 0xFF, 0xFF, 0x7E, 0x0, 0x0},{0x0, 0x0, 0x7E, 0xFF, 0xFF, 0xFF, 0xE1, 0xE1, 0xE1, 0xE1, 0xFF, 0xFF, 0xFF, 0x7E, 0x0, 0x0}},
	{{0x0, 0x0, 0x7E, 0xFF, 0xFF, 0xFF, 0xE1, 0xE1, 0xE1, 0xE1, 0xFF, 0xFF, 0xFF, 0x7E, 0x0, 0x0},{0x0, 0x0, 0x7E, 0xFF, 0xFF, 0xFF, 0xE1, 0xE1, 0xE1, 0xE1, 0xFF, 0xFF, 0xFF, 0x7E, 0x0, 0x0}},
	{{0x0, 0x0, 0x7E, 0xFF, 0xFF, 0xFF, 0xE1, 0xE1, 0xE1, 0xE1, 0xFF, 0xFF, 0xFF, 0x7E, 0x0, 0x0},{0x0, 0x0, 0x7E, 0xFF, 0xFF, 0xFF, 0xE1, 0xE1, 0xE1, 0xE1, 0xFF, 0xFF, 0xFF, 0x7E, 0x0, 0x0}},
	{{0x0, 0x0, 0x7E, 0xFF, 0xFF, 0xFF, 0xE1, 0xE1, 0xE1, 0xE1, 0xFF, 0xFF, 0xFF, 0x7E, 0x0, 0x0},{0x0, 0x0, 0x7E, 0xFF, 0xFF, 0xFF, 0xE1, 0xE1, 0xE1, 0xE1, 0xFF, 0xFF, 0xFF, 0x7E, 0x0, 0x0}},
	{{0x0, 0x0, 0x7E, 0xFF, 0xFF, 0xFF, 0xE1, 0xE1, 0xE1, 0xE1, 0xFF, 0xFF, 0xFF, 0x7E, 0x0, 0x0},{0x0, 0x0, 0x7E, 0xFF, 0xFF, 0xFF, 0xE1, 0xE1, 0xE1, 0xE1, 0xFF, 0xFF, 0xFF, 0x7E, 0x0, 0x0}},
	{{0x0, 0x0, 0x7E, 0xFF, 0xFF, 0xFF, 0xE1, 0xE1, 0xE1, 0xE1, 0xFF, 0xFF, 0xFF, 0x7E, 0x0, 0x0},{0x0, 0x0, 0x7E, 0xFF, 0xFF, 0xFF, 0xE1, 0xE1, 0xE1, 0xE1, 0xFF, 0xFF, 0xFF, 0x7E, 0x0, 0x0}},
	{{0x0, 0x0, 0x7E, 0xFF, 0xFF, 0xFF, 0xE1, 0xE1, 0xE1, 0xE1, 0xFF, 0xFF, 0xFF, 0x7E, 0x0, 0x0},{0x0, 0x0, 0x7E, 0xFF, 0xFF, 0xFF, 0xE1, 0xE1, 0xE1, 0xE1, 0xFF, 0xFF, 0xFF, 0x7E, 0x0, 0x0}},
	{{0x0, 0x0, 0x3C, 0x7E, 0x7E, 0x7E, 0x62, 0x62, 0x62, 0x62, 0x7E, 0x7E, 0x7E, 0x3C, 0x0, 0x0},{0x0, 0x0, 0x3C, 0x7E, 0x7E, 0x7E, 0x62, 0x62, 0x62, 0x62, 0x7E, 0x7E, 0x7E, 0x3C, 0x0, 0x0}},
	{{0x0, 0x0, 0x3C, 0x7E, 0x7E, 0x7E, 0x62, 0x62, 0x62, 0x62, 0x7E, 0x7E, 0x7E, 0x3C, 0x0, 0x0},{0x0, 0x0, 0x3C, 0x7E, 0x7E, 0x7E, 0x62, 0x62, 0x62, 0x62, 0x7E, 0x7E, 0x7E, 0x3C, 0x0, 0x0}},
	{{0x0, 0x0, 0x18, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x18, 0x0, 0x0},{0x0, 0x0, 0x18, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x18, 0x0, 0x0}},
	{{0x0, 0x0, 0x18, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x18, 0x0, 0x0},{0x0, 0x0, 0x18, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x18, 0x0, 0x0}},
	{{0x0, 0x0, 0x3C, 0x7E, 0x7E, 0x7E, 0x62, 0x62, 0x62, 0x62, 0x7E, 0x7E, 0x7E, 0x3C, 0x0, 0x0},{0x0, 0x0, 0x3C, 0x7E, 0x7E, 0x7E, 0x62, 0x62, 0x62, 0x62, 0x7E, 0x7E, 0x7E, 0x3C, 0x0, 0x0}},
	{{0x0, 0x0, 0x3C, 0x7E, 0x7E, 0x7E, 0x62, 0x62, 0x62, 0x62, 0x7E, 0x7E, 0x7E, 0x3C, 0x0, 0x0},{0x0, 0x0, 0x3C, 0x7E, 0x7E, 0x7E, 0x62, 0x62, 0x62, 0x62, 0x7E, 0x7E, 0x7E, 0x3C, 0x0, 0x0}},
	{{0x0, 0x0, 0x7E, 0xFF, 0xFF, 0xFF, 0xE1, 0xE1, 0xE1, 0xE1, 0xFF, 0xFF, 0xFF, 0x7E, 0x0, 0x0},{0x0, 0x0, 0x7E, 0xFF, 0xFF, 0xFF, 0xE1, 0xE1, 0xE1, 0xE1, 0xFF, 0xFF, 0xFF, 0x7E, 0x0, 0x0}},
	{{0x0, 0x0, 0x7E, 0xFF, 0xFF, 0xFF, 0xE1, 0xE1, 0xE1, 0xE1, 0xFF, 0xFF, 0xFF, 0x7E, 0x0, 0x0},{0x0, 0x0, 0x7E, 0xFF, 0xFF, 0xFF, 0xE1, 0xE1, 0xE1, 0xE1, 0xFF, 0xFF, 0xFF, 0x7E, 0x0, 0x0}},
	{{0x0, 0x0, 0x7E, 0xFF, 0xFF, 0xFF, 0xE1, 0xE1, 0xE1, 0xE1, 0xFF, 0xFF, 0xFF, 0x7E, 0x0, 0x0},{0x0, 0x0, 0x7E, 0xFF, 0xFF, 0xFF, 0xE1, 0xE1, 0xE1, 0xE1, 0xFF, 0xFF, 0xFF, 0x7E, 0x0, 0x0}},
	{{0x0, 0x0, 0x7E, 0xFF, 0xFF, 0xFF, 0xE1, 0xE1, 0xE1, 0xE1, 0xFF, 0xFF, 0xFF, 0x7E, 0x0, 0x0},{0x0, 0x0, 0x7E, 0xFF, 0xFF, 0xFF, 0xE1, 0xE1, 0xE1, 0xE1, 0xFF, 0xFF, 0xFF, 0x7E, 0x0, 0x0}},
	{{0x0, 0x0, 0x7E, 0xFF, 0xFF, 0xFF, 0xE1, 0xE1, 0xE1, 0xE1, 0xFF, 0xFF, 0xFF, 0x7E, 0x0, 0x0},{0x0, 0x0, 0x7E, 0xFF, 0xFF, 0xFF, 0xE1, 0xE1, 0xE1, 0xE1, 0xFF, 0xFF, 0xFF, 0x7E, 0x0, 0x0}},
	{{0x0, 0x0, 0x7E, 0xFF, 0xFF, 0xFF, 0xE1, 0xE1, 0xE1, 0xE1, 0xFF, 0xFF, 0xFF, 0x7E, 0x0, 0x0},{0x0, 0x0, 0x7E, 0xFF, 0xFF, 0xFF, 0xE1, 0xE1, 0xE1, 0xE1, 0xFF, 0xFF, 0xFF, 0x7E, 0x0, 0x0}},
	{{0x0, 0x0, 0x7E, 0xFF, 0xFF, 0xFF, 0xE1, 0xE1, 0xE1, 0xE1, 0xFF, 0xFF, 0xFF, 0x7E, 0x0, 0x0},{0x0, 0x0, 0x7E, 0xFF, 0xFF, 0xFF, 0xE1, 0xE1, 0xE1, 0xE1, 0xFF, 0xFF, 0xFF, 0x7E, 0x0, 0x0}},
	{{0x0, 0x0, 0x7E, 0xFF, 0xFF, 0xFF, 0xE1, 0xE1, 0xE1, 0xE1, 0xFF, 0xFF, 0xFF, 0x7E, 0x0, 0x0},{0x0, 0x0, 0x7E, 0xFF, 0xFF, 0xFF, 0xE1, 0xE1, 0xE1, 0xE1, 0xFF, 0xFF, 0xFF, 0x7E, 0x0, 0x0}},
	{{0x0, 0x0, 0x7E, 0xFF, 0xFF, 0xFF, 0xE1, 0xE1, 0xE1, 0xE1, 0xFF, 0xFF, 0xFF, 0x7E, 0x0, 0x0},{0x0, 0x0, 0x7E, 0xFF, 0xFF, 0xFF, 0xE1, 0xE1, 0xE1, 0xE1, 0xFF, 0xFF, 0xFF, 0x7E, 0x0, 0x0}},
	{{0x0, 0x0, 0x7E, 0xFF, 0xFF, 0xFF, 0xE1, 0xE1, 0xE1, 0xE1, 0xFF, 0xFF, 0xFF, 0x7E, 0x0, 0x0},{0x0, 0x0, 0x7E, 0xFF, 0xFF, 0xFF, 0xE1, 0xE1, 0xE1, 0xE1, 0xFF, 0xFF, 0xFF, 0x7E, 0x0, 0x0}},
	{{0x0, 0x0, 0x7E, 0xFF, 0xFF, 0xFF, 0xE1, 0xE1, 0xE1, 0xE1, 0xFF, 0xFF, 0xFF, 0x7E, 0x0, 0x0},{0x0, 0x0, 0x7E, 0xFF, 0xFF, 0xFF, 0xE1, 0xE1, 0xE1, 0xE1, 0xFF, 0xFF, 0xFF, 0x7E, 0x0, 0x0}},
	{{0x0, 0x0, 0x7E, 0xFF, 0xFF, 0xFF, 0xE1, 0xE1, 0xE1, 0xE1, 0xFF, 0xFF, 0xFF, 0x7E, 0x0, 0x0},{0x0, 0x0, 0x7E, 0xFF, 0xFF, 0xFF, 0xE1, 0xE1, 0xE1, 0xE1, 0xFF, 0xFF, 0xFF, 0x7E, 0x0, 0x0}},
	{{0x0, 0x0, 0x7E, 0xFF, 0xFF, 0xFF, 0xE1, 0xE1, 0xE1, 0xE1, 0xFF, 0xFF, 0xFF, 0x7E, 0x0, 0x0},{0x0, 0x0, 0x7E, 0xFF, 0xFF, 0xFF, 0xE1, 0xE1, 0xE1, 0xE1, 0xFF, 0xFF, 0xFF, 0x7E, 0x0, 0x0}},
	{{0x0, 0x0, 0x7E, 0xFF, 0xFF, 0xFF, 0xE1, 0xE1, 0xE1, 0xE1, 0xFF, 0xFF, 0xFF, 0x7E, 0x0, 0x0},{0x0, 0x0, 0x7E, 0xFF, 0xFF, 0xFF, 0xE1, 0xE1, 0xE1, 0xE1, 0xFF, 0xFF, 0xFF, 0x7E, 0x0, 0x0}},
	{{0x0, 0x0, 0x7E, 0xFF, 0xFF, 0xFF, 0xE1, 0xE1, 0xE1, 0xE1, 0xFF, 0xFF, 0xFF, 0x7E, 0x0, 0x0},{0x0, 0x0, 0x7E, 0xFF, 0xFF, 0xFF, 0xE1, 0xE1, 0xE1, 0xE1, 0xFF, 0xFF, 0xFF, 0x7E, 0x0, 0x0}},
	{{0x0, 0x0, 0x7E, 0xFF, 0xFF, 0xFF, 0xE1, 0xE1, 0xE1, 0xE1, 0xFF, 0xFF, 0xFF, 0x7E, 0x0, 0x0},{0x0, 0x0, 0x7E, 0xFF, 0xFF, 0xFF, 0xE1, 0xE1, 0xE1, 0xE1, 0xFF, 0xFF, 0xFF, 0x7E, 0x0, 0x0}}
};

MovementStruct sadMovements = {
  .maxFrames = 80,
  .neckBaseFrames = 80,
  .neckBaseAngles = {90, 87, 84, 81, 78, 75, 72, 69, 66, 63, 60, 63, 66, 69, 72, 75, 78, 81, 84, 87, 90, 93, 96, 99, 102, 105, 108, 111, 114, 117, 120, 117, 114, 111, 108, 105, 102, 99, 96, 93, 90, 87, 84, 81, 78, 75, 72, 69, 66, 63, 60, 63, 66, 69, 72, 75, 78, 81, 84, 87, 90, 93, 96, 99, 102, 105, 108, 111, 114, 117, 120, 117, 114, 111, 108, 105, 102, 99, 96, 93},
  .neckSphereXFrames = 80,
  .neckSphereXAngles = {75, 80, 85, 90, 95, 100, 105, 110, 110, 110, 110, 110, 110, 110, 100, 100, 110, 110, 110, 110, 110, 110, 110, 100, 100, 110, 110, 100, 100, 110, 110, 110, 110, 110, 110, 110, 100, 110, 110, 110, 110, 110, 110, 100, 110, 110, 110, 100, 100, 110, 110, 110, 110, 110, 110, 100, 100, 110, 110, 110, 110, 110, 110, 100, 100, 110, 110, 110, 110, 110, 110, 100, 100, 110, 110, 110, 110, 110, 110, 110},
  .neckSphereYFrames = 80,
  .neckSphereYAngles = {40, 45, 50, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55},
  .leftEarFrames = 80,
  .leftEarAngles = {90, 95, 100, 105, 110, 115, 120, 125, 130, 135, 140, 145, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, },
  .rightEarFrames = 80,
  .rightEarAngles = {150, 145, 140, 135, 130, 125, 120, 115, 110, 105, 100, 95, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90},
  .leftEyebrowFrames = 80,
  .leftEyebrowAngles = {130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130},
  .rightEyebrowFrames = 80,
  .rightEyebrowAngles = {70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70},
};

std::vector<std::vector<std::vector<unsigned int>>> sad = {
    {
        {0x0, 0x0, 0x0, 0x3C, 0x7E, 0xFF, 0xE3, 0xE3, 0xE3, 0xE3, 0xFF, 0x7E, 0x3C, 0x0, 0x0, 0x0},
        {0x0, 0x0, 0x0, 0x3C, 0x7E, 0xFF, 0xE3, 0xE3, 0xE3, 0xE3, 0xFF, 0x7E, 0x3C, 0x0, 0x0, 0x0}
    },
    {
        {0x0, 0x0, 0x0, 0x3C, 0x7E, 0x7F, 0x63, 0x63, 0x63, 0x63, 0x7F, 0x7E, 0x3C, 0x0, 0x0, 0x0},
        {0x0, 0x0, 0x0, 0x3C, 0x7E, 0x7F, 0x63, 0x63, 0x63, 0x63, 0x7F, 0x7E, 0x3C, 0x0, 0x0, 0x0}
    },
    {
        {0x0, 0x0, 0x0, 0x3C, 0x7E, 0x7F, 0x73, 0x73, 0x73, 0x73, 0x7F, 0x7E, 0x3C, 0x0, 0x0, 0x0},
        {0x0, 0x0, 0x0, 0x3C, 0x7E, 0x7F, 0x73, 0x73, 0x73, 0x73, 0x7F, 0x7E, 0x3C, 0x0, 0x0, 0x0}
    },
    {
        {0x0, 0x0, 0x0, 0x3C, 0x7E, 0x7F, 0x7F, 0x73, 0x73, 0x73, 0x73, 0x7E, 0x3C, 0x0, 0x0, 0x0},
        {0x0, 0x0, 0x0, 0x3C, 0x7E, 0x73, 0x73, 0x73, 0x73, 0x7F, 0x7F, 0x7E, 0x3C, 0x0, 0x0, 0x0}
    },
    {
        {0x0, 0x0, 0x0, 0x3C, 0x7E, 0x7F, 0x7F, 0x7F, 0x73, 0x73, 0x73, 0x72, 0x3C, 0x0, 0x0, 0x0},
        {0x0, 0x0, 0x0, 0x3C, 0x72, 0x73, 0x73, 0x73, 0x7F, 0x7F, 0x7F, 0x7E, 0x3C, 0x0, 0x0, 0x0}
    },
    {
        {0x0, 0x0, 0x0, 0x3C, 0x7E, 0x7F, 0x7F, 0x73, 0x73, 0x73, 0x73, 0x7E, 0x3C, 0x0, 0x0, 0x0},
        {0x0, 0x0, 0x0, 0x3C, 0x7E, 0x73, 0x73, 0x73, 0x73, 0x7F, 0x7F, 0x7E, 0x3C, 0x0, 0x0, 0x0}
    },
    {
        {0x0, 0x0, 0x0, 0x3C, 0x7E, 0x7F, 0x73, 0x73, 0x73, 0x73, 0x7F, 0x7E, 0x3C, 0x0, 0x0, 0x0},
        {0x0, 0x0, 0x0, 0x3C, 0x7E, 0x7F, 0x73, 0x73, 0x73, 0x73, 0x7F, 0x7E, 0x3C, 0x0, 0x0, 0x0}
    },
    {
        {0x0, 0x0, 0x0, 0x3C, 0x7E, 0x7F, 0x63, 0x63, 0x63, 0x63, 0x7F, 0x7E, 0x3C, 0x0, 0x0, 0x0},
        {0x0, 0x0, 0x0, 0x3C, 0x7E, 0x7F, 0x63, 0x63, 0x63, 0x63, 0x7F, 0x7E, 0x3C, 0x0, 0x0, 0x0}
    },
    {
        {0x0, 0x0, 0x0, 0x3C, 0x7E, 0xFF, 0xE3, 0xE3, 0xE3, 0xE3, 0xFF, 0x7E, 0x3C, 0x0, 0x0, 0x0},
        {0x0, 0x0, 0x0, 0x3C, 0x7E, 0xFF, 0xE3, 0xE3, 0xE3, 0xE3, 0xFF, 0x7E, 0x3C, 0x0, 0x0, 0x0}
    }
}; 