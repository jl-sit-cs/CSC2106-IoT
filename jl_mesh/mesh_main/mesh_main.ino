#include <M5StickCPlus.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <esp_now.h>

/* ESP_NOW STRUCTURE*/
uint8_t broadcastAddress[] = {0x4C, 0x75, 0x25, 0xCB, 0x98, 0x84};
esp_now_peer_info_t peerInfo;

/* Wi-Fi & MQTT  Settings */
const char* WIFI_SSID        = "";
const char* WIFI_PASSWORD    = "";
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
messageBuffer currentMessage;
messageBuffer heap[MAX_HEAP_SIZE];
int heapSize = 0;

/* General Setup */
#define MAX_ATTEMPTS 2

unsigned long nextMedicationTime = 0;
int warningSec = 5;
int attempts = 3;
int retryCount = 0;


/** Function Prototypes **/
void handle_buzzer_on();
void handle_buzzer_off();
void printMessages(int attempts);
void setupWifi();
void callback(char* topic, byte* payload, unsigned int length);
void setupMQTT();
void computeCheckSum();
void sendMessage();
void init_esp_peer();
void onReceive(const uint8_t *mac, const uint8_t *data, int len);

WiFiClient mqttClient;
PubSubClient client(mqttClient);

// Bluetooth Class
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks{
  void onResult(BLEAdvertisedDevice advertisedDevice){
    int rssi = advertisedDevice.getRSSI();

    phoneDetected = (rssi > minimumDeviceThreshold);
  }
};

void OnSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nSend message status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Sent Successfully" : "Sent Failed");

  if (status != ESP_NOW_SEND_SUCCESS) {
    if (retryCount < MAX_ATTEMPTS) {
      retryCount++;
      Serial.printf("Retry attempt #%d...\n", retryCount);
      esp_err_t result = esp_now_send(mac_addr, (uint8_t *)&currentMessage, sizeof(currentMessage));
      if (result == ESP_OK) {
        Serial.println("Retry sent successfully.");
      } else {
        Serial.printf("Retry failed with error code: %d\n", result);
      }
    } else {
      Serial.println("Max retries reached. Giving up.");
    }
  } else {
    // Reset the retry count on successful send
    retryCount = 0;
  }
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

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  client.setServer(mqtt_server, 1883);  // Sets the server details.  
  client.setCallback(callback);  

  init_esp_peer();
  esp_now_register_send_cb(OnSent);
  esp_now_register_recv_cb(onReceive);

  // /* Bluetooth setup */
  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan(); 
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);

  nextMedicationTime = millis();
}

void loop() {
  if (!client.connected()) {
        reconnect();
  }
  client.loop(); 
  M5.update();

  // unsigned long elapsedTime = millis() - nextMedicationTime;
  // unsigned long lastScan = millis() - lastBLEScan;

  // if(elapsedTime >= warningSec * 1000 && lastScan >= bleInterval){
  //   lastBLEScan = millis();
    
  //   pBLEScan->start(SCAN_TIME, false);
        
  //     if (phoneDetected) {
  //       //handle_buzzer_on();
  //       // nextMedicationTime = millis();      
  //       // attempts = 0;
  //       // return;
  //     }

  //     printMessages(attempts);
  //     attempts++;
  //   }  

  if(attempts >= MAX_ATTEMPTS){
    client.publish("medication_adherence/not_taken_medication", "Hello World");  

    sendMessage(1,DETECT_PHONE,"Hello World");
      
    attempts = 0;  
  }

  if (digitalRead(M5_BUTTON_HOME) == LOW) {
    handle_buzzer_off();
    while(digitalRead(M5_BUTTON_HOME) == LOW);
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

void sendMessage(int id, int messageType, String message){
  currentMessage.id = id; 
  currentMessage.messageType = messageType;
  snprintf(currentMessage.medicationMessage, MAX_MESSAGE_LENGTH, "%s", message);

  computeCheckSum();

  // Print the results
  printf("Medication Message: %s\n", currentMessage.medicationMessage);
  printf("Checksum: %s\n", currentMessage.checksum);

  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &currentMessage, sizeof(currentMessage));
  
  if (result == ESP_OK) {
    Serial.println("The message was sent successfully.");
    return;
  } else {
    Serial.printf("Error sending message, error code: %d\n", result);
  }

}

void computeCheckSum(){
  unsigned char checksum = currentMessage.id ^ currentMessage.messageType;
  
  for (int i = 0; i < strlen(currentMessage.medicationMessage); i++) {
    checksum ^= currentMessage.medicationMessage[i];
  }

  snprintf(currentMessage.checksum, MAX_CHECKSUM_LENGTH, "%02X", checksum);
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


void callback(char* topic, byte* payload, unsigned int length) {
    if (strcmp(topic, "medication_adherence/taken_medication") == 0){
      nextMedicationTime = millis();
    }
}

void reconnect() {
    while (!client.connected()) {
        M5.Lcd.print("Attempting MQTT connection...");
        // Create a random client ID.  
        String clientId = "M5Stick-";
        clientId += String(random(0xffff), HEX);
        // Attempt to connect.  
        if (client.connect(clientId.c_str())) {
            M5.Lcd.printf("\nSuccess\n");
            // Once connected, publish an announcement to the topic.
            Serial.println("Success");
            client.subscribe("medication_adherence/taken_medication");
        } else {
            M5.Lcd.print(client.state());
            M5.Lcd.println("try again in 5 seconds");
            delay(5000);
        }
    }
}

void init_esp_peer(){
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK){
      Serial.println("Failed to add peer");
      return;
    }
}

void onReceive(const uint8_t *mac, const uint8_t *data, int len) {
  Serial.print("Received message: ");
  for (int i = 0; i < len; i++) {
    Serial.print((char)data[i]);
  }
  Serial.println();
}