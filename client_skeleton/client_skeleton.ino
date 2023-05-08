#include <ESP32Servo.h>
#include <WiFi.h>
#include <WiFiClient.h>

// Network details
const char* ssid = "test";
const char* password = "test1234567!";
const char* server_ip = "192.168.1.3";
int server_port = 800;

static WiFiClient client;

// Global variables
String incomingData;

TaskHandle_t Task1;
TaskHandle_t Task2;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  delay(10);
  Serial.println();

  // Create a task that will be executed in the main_loop() function,
  // with name "main", stack size 10000, NULL as parameter, priority 1,
  // handled by Task1 and executed on core 0
  xTaskCreatePinnedToCore(main_loop, "main", 10000, NULL, 1, &Task1, 0);                 
  delay(500); 

  xTaskCreatePinnedToCore(network_loop, "network", 10000, NULL, 1, &Task2, 1);
  delay(500);
}

void main_loop(void * pvParameters) {
  // Handle your emotions stuff
  while(true) {
  /* Read from serial interface to bypass WiFi for debugging
    if(Serial.available())  {
      incomingData = Serial.readStringUntil('\n');
      Serial.println(incomingData);
      // handle(incomingData);
    }
  */
    delay(10);  
  }
}

void network_loop(void * pvParameters) {
  // Handle connection and receive from WiFi (updating state)
  while(true) {

    if(WiFi.status() != WL_CONNECTED) {
      initConnection();     
    }

    if(Serial.available()) {
      String send = Serial.readString();
      client.print(send);
    }

    if(client.available())  {
      incomingData = client.readString();
      Serial.println(incomingData);
      // handle(incomingData);
    }
    
    delay(10);  
  }
}

void initConnection() {

  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  // WiFi connection
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    //Serial.print(".");
  }
  
  Serial.println();
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  // Server connection
  Serial.println();
  Serial.println("Connecting to Server...");
  while (!client.connect(server_ip, server_port)) {
    delay(1000);
    //Serial.print(".");
  }
  Serial.println("Server connected");
  Serial.println();
}

void loop() {
  // put your main code here, to run repeatedly:

}
