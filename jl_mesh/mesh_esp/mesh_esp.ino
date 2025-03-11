#include <painlessMesh.h>
#include <M5StickCPlus.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <WiFi.h>
#include <PubSubClient.h>

/* Wi-Fi & MQTT  Settings */
const char* ssid = "SSID";
const char* password = "password";
const char* mqtt_server = "broker.hivemq.com"; // Use your MQTT broker

/* Bluetooth Structure */
#define SCAN_TIME 5
#define PHONE_BLUETOOTH_NAME "JL"
BLEScan* pBLEScan;
int minimumDeviceThreshold = -60;
bool phoneDetected = false; 


/** Mesh Identification **/
#define   MESH_SSID       "csc2106meshy"
#define   MESH_PASSWORD   "meshpotatoes"
#define   MESH_PORT       3000

/** Structure for Message **/
#define MAX_HEAP_SIZE 10  
#define MAX_MESSAGE_LENGTH 50
#define MAX_CHECKSUM_LENGTH 20
#define sourceID 1 
#define broadCastID 255

struct Message {
  int id;
  int priority; 
  char message[MAX_MESSAGE_LENGTH];
  char checksum[MAX_CHECKSUM_LENGTH];
  int destinationId;
} messageBuffer;


// messageBuffer heap[MAX_HEAP_SIZE];
// int heapSize = 0;

/* General Setup */
unsigned long nextMedicationTime = 0;
int durationSec = 20; 

/** Function Prototypes **/
void turnOnLEDandBuzzer();
void sendMeshMessage();
void receivedmeshCallback(uint32_t from, String &msg);
void newConnectionCallback(uint32_t nodeId);
void changedConnectionCallback();
void nodeTimeAdjustedCallback(int32_t offset);
void delayReceivedCallback(uint32_t from, int32_t delay);
void handle_buzzer_on();
void handle_buzzer_off();

/**Base Setup for Scheduler, Mesh and MQTT**/
Scheduler userScheduler;
painlessMesh mesh;

WiFiClient espClient;
PubSubClient client(espClient);

// Bluetooth Class
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks{
  void onResult(BLEAdvertisedDevice advertisedDevice){
    int rssi = advertisedDevice.getRSSI();
    if(rssi > minimumDeviceThreshold){
      phoneDetected = true;
    }else {
      phoneDetected = false;
    }
  }
};


void setup() {
  Serial.begin(115200);

  /* M5 Setup */
  int x = M5.IMU.Init(); //return 0 is ok, return -1 is unknown
  if(x!=0)
    Serial.println("IMU initialisation fail!");  
  M5.begin();
  M5.Lcd.setRotation(3);
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setCursor(0, 0, 2);
  M5.Lcd.printf("Start Initialization");

  
  pinMode(M5_LED, OUTPUT);
  pinMode(M5_BUTTON_HOME, INPUT);
  pinMode(BUTTON_B_PIN, INPUT);
  digitalWrite(M5_LED, 1);


  /* Wi-Fi Setup */
  //WiFi.begin(ssid,password);

  /* Mesh Setup */
  // mesh.setDebugMsgTypes(ERROR | DEBUG); 

  // mesh.init(MESH_SSID, MESH_PASSWORD, &userScheduler, MESH_PORT);
  // mesh.onReceive(&receivedCallback);
  // mesh.onNewConnection(&newConnectionCallback);
  // mesh.onChangedConnections(&changedConnectionCallback);
  // mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);
  // mesh.onNodeDelayReceived(&delayReceivedCallback);

  /* Bluetooth setup */

  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan(); 
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true); 

  nextMedicationTime = millis();

}

void loop() {
  // mesh.update();

  BLEScanResults foundDevices = pBLEScan->start(SCAN_TIME);

  unsigned long elapsedTime = millis() - nextMedicationTime;

  if (phoneDetected && elapsedTime >= durationSec) {
    handle_buzzer_on();
    nextMedicationTime = millis();
  }

  if (digitalRead(M5_BUTTON_HOME) == LOW) {
    handle_buzzer_off();
    while(digitalRead(M5_BUTTON_HOME) == LOW);
  }

  delay(1000);

}


void handle_buzzer_on(){
  digitalWrite(M5_LED, 0);

  M5.Beep.tone(4000, 300);
  M5.update();
}


void handle_buzzer_off(){
  digitalWrite(M5_LED, 1);

  M5.Beep.tone(0, 0);
  M5.update();
}

void sendMeshMessage() {
}


void receivedmeshCallback(uint32_t from, String &msg) {
}

void newConnectionCallback(uint32_t nodeId) {
}

void changedConnectionCallback() {
}

void nodeTimeAdjustedCallback(int32_t offset) {
}

void delayReceivedCallback(uint32_t from, int32_t delay) {
}