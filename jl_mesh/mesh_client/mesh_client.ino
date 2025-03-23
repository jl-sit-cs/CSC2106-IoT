#include <M5StickCPlus.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <esp_now.h>

/* ESP_NOW STRUCTURE*/
uint8_t broadcastAddress[] = {0xE8, 0x9F, 0x6D, 0x09, 0x39, 0x70};
esp_now_peer_info_t peerInfo;

/* Wi-Fi & MQTT  Settings */
const char* WIFI_SSID        = "JL";
const char* WIFI_PASSWORD    = "helloworld2024";
const char* mqtt_server = "172.20.10.4";

/* Bluetooth Structure */
#define SCAN_TIME 5
#define TARGET_PHONE_BLUETOOTH_NAME "JL" // change accordingly
BLEScan* pBLEScan;
int minimumDeviceThreshold = -60;
bool phoneDetected = false; 
const unsigned long bleInterval = 5000;
unsigned long lastBLEScan = 0;

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
  int messageType;
  char checksum[MAX_CHECKSUM_LENGTH];
  char medicationMessage[MAX_MESSAGE_LENGTH];
};

/* Creates a structure for any leave out message */
messageBuffer heap[MAX_HEAP_SIZE];
int heapSize = 0;

/* General Setup */
#define MAX_ATTEMPTS 2

unsigned long nextMedicationTime = 0;
int warningSec = 5;
int attempts = 3;

/** Function Prototypes **/
void handle_buzzer_on();
void handle_buzzer_off();
void printMessages(int attempts);
void setupWifi();
void computeCheckSum();
void init_esp_peer();
void onReceive(const uint8_t *mac, const uint8_t *data, int len);

// Bluetooth Class
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks{
  void onResult(BLEAdvertisedDevice advertisedDevice){
    int rssi = advertisedDevice.getRSSI();

    phoneDetected = (rssi > minimumDeviceThreshold);
  }
};

void readMacAddress(){
 uint8_t mac[6];
  WiFi.macAddress(mac);
  Serial.print("MAC Address: ");
  for (int i = 0; i < 6; i++) {
      Serial.print(mac[i], HEX);
      if (i < 5) Serial.print(":");
  }
  Serial.println();
}


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


  setupWifi();
  readMacAddress();
  

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  init_esp_peer();
  esp_now_register_recv_cb(onReceive);


  // /* Bluetooth setup */
  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan(); 
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
}

void loop() {
  if (digitalRead(M5_BUTTON_HOME) == LOW) {
    handle_buzzer_off();
    while(digitalRead(M5_BUTTON_HOME) == LOW);
  }
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

void computeCheckSum(struct messageBuffer *myMessage){
  unsigned char checksum = myMessage->id ^ myMessage->messageType;
  
  for (int i = 0; i < strlen(myMessage->medicationMessage); i++) {
    checksum ^= myMessage->medicationMessage[i];
  }

  snprintf(myMessage->checksum, MAX_CHECKSUM_LENGTH, "%02X", checksum);
}

void setupWifi() {
    delay(10);
    M5.Lcd.printf("Connecting to %s", WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);  

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        M5.Lcd.print(".");
    }
    M5.Lcd.printf("\nSuccess\n");
}

void init_esp_peer() {
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}

void onReceive(const uint8_t *mac, const uint8_t *data, int len) {
    if (len == sizeof(messageBuffer)) {
        messageBuffer *receivedMessage = (messageBuffer*)data;
        
        Serial.print("Received message ID: ");
        Serial.println(receivedMessage->id);
        // Handle other fields...
    } else {
        Serial.println("Data length mismatch, printing raw data:");
        for (int i = 0; i < len; i++) {
            Serial.print(data[i], HEX);  // Print each byte in hexadecimal
            Serial.print(" ");
        }
        Serial.println();
    }
}