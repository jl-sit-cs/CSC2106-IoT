#include <painlessMesh.h>
#include <M5StickCPlus.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <WiFi.h>
#include <PubSubClient.h>

/* Wi-Fi & MQTT  Settings */
const char* ssid = "SSID";
const char* password = "password";
const char* mqtt_server = "broker.hivemq.com"; 

/* Bluetooth Structure */
#define SCAN_TIME 5
#define TARGET_PHONE_BLUETOOTH_NAME "JL" // change accordingly
BLEScan* pBLEScan;
int minimumDeviceThreshold = -60;
bool phoneDetected = false; 


/** Mesh Identification **/
#define   MESH_SSID       "medicationadherence"
#define   MESH_PASSWORD   "medicationadherencepassword"
#define   MESH_PORT       3000

/** Structure for Message **/
#define MAX_HEAP_SIZE 10  
#define MAX_CHECKSUM_LENGTH 20
#define MAX_MESSAGE_LENGTH 30

#define DETECT_PHONE 1
#define SAVE_DATA 2 
#define PHONE_DETECTED 3
#define PHONE_NOT_DETECTED 4
#define CLEAR_DATA 5

struct messageBuffer {
  int id;
  int priority; 
  int messageType;
  char checksum[MAX_CHECKSUM_LENGTH];
  char medicationType[MAX_MESSAGE_LENGTH];
};

messageBuffer heap[MAX_HEAP_SIZE];
int heapSize = 0;

/* General Setup */
#define MAX_ATTEMPTS 2

unsigned long nextMedicationTime = 0;
int durationSec = 20; 
int attempts = 0;

/** Function Prototypes **/
void sendMeshMessage();
void receivedmeshCallback(uint32_t from, String &msg);
void newConnectionCallback(uint32_t nodeId);
void changedConnectionCallback();
void nodeTimeAdjustedCallback(int32_t offset);
void delayReceivedCallback(uint32_t from, int32_t delay);
void handle_buzzer_on();
void handle_buzzer_off();
void printMessages(int attempts);

/**Base Setup for Scheduler, Mesh and MQTT**/
Scheduler userScheduler;
painlessMesh mesh;

WiFiClient espClient;
PubSubClient client(espClient);

// Bluetooth Class
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks{
  void onResult(BLEAdvertisedDevice advertisedDevice){
    Serial.printf("Found device: %s\n", advertisedDevice.getName().c_str());
    Serial.printf("advertised Name: %s\n",advertisedDevice.haveName());

    if(advertisedDevice.getName() == TARGET_PHONE_BLUETOOTH_NAME){
      phoneDetected = true;
    }else{
      phoneDetected=false;
    }

    // int rssi = advertisedDevice.getRSSI();
    // if(rssi > minimumDeviceThreshold){
    //   phoneDetected = true;
    // }else {
    //   phoneDetected = false;
    // }
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
  mesh.setDebugMsgTypes(ERROR | DEBUG); 

  mesh.init(MESH_SSID, MESH_PASSWORD, &userScheduler, MESH_PORT);
  // mesh.onReceive(&receivedCallback);
  // mesh.onNewConnection(&newConnectionCallback);

  /* Bluetooth setup */
  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan(); 
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true); 

  nextMedicationTime = millis();
}

void loop() {
  mesh.update();

  unsigned long elapsedTime = millis() - nextMedicationTime;

  if(elapsedTime >= durationSec){

    while(attempts <= MAX_ATTEMPTS){

      pBLEScan->start(SCAN_TIME);
      
      if (phoneDetected) {
        handle_buzzer_on();
        nextMedicationTime = millis();
        // maybe send an MQTT message here?
      
        attempts = 0;
        break;
      }

      printMessages(attempts);

      attempts++;
      delay(1000);
    }
  }  

  if (digitalRead(M5_BUTTON_HOME) == LOW) {
    handle_buzzer_off();
    while(digitalRead(M5_BUTTON_HOME) == LOW);
  }

  if(attempts >= MAX_ATTEMPTS){
    //broadcast an mesh message here 
    Serial.println("MAX_ATTEMPTS MAX");
    attempts = 0;  
  }
}

void printMessages(int attempts){
  Serial.printf("Attempts #%d: Failed\n", (attempts+1));

  if (attempts == 0){
    M5.Lcd.setRotation(3);
    M5.Lcd.fillScreen(BLACK);
  }

  int position = attempts * 20;

  M5.Lcd.setCursor(0, (attempts * 20), 2);
  M5.Lcd.printf("Attempt #%d: Failed", (attempts+1));
  M5.update();
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

