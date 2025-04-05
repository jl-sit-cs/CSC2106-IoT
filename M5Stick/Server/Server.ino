#include <M5StickCPlus.h>
#include <esp_now.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>


extern "C" {
  #include "esp_wifi.h"
}

#define HEARTBEAT_MSG 3
#define SERVER_MSG 4
#define MAX_ATTEMPTS 3
#define MAX_DEVICES 5
#define LAST_HEARTBEAT_MSG 10000
#define MAX_CHECKSUM_LENGTH 5
#define MQTT_TOPIC "espnow/host"

const char* WIFI_SSID = "JL";
const char* WIFI_PASSWORD = "helloworld2024";
const char* MQTT_SERVER = "172.20.10.13";

WiFiClient espClient;
PubSubClient mqttClient(espClient);
TaskHandle_t mqttTaskHandle = NULL;

unsigned long lastHeartbeatTime = 0;
esp_now_peer_info_t peerInfo[MAX_DEVICES];
int retryCount = 0;

uint8_t serverAddress[MAX_DEVICES][6] = {
  {0x0C, 0x8B, 0x95, 0xA8, 0x27, 0xBC},
  {0x4C, 0x75, 0x25, 0xCB, 0x88, 0x60},
  {0x4C, 0x75, 0x25, 0xCB, 0x98, 0x84},
  {0xE8, 0x9F, 0x6D, 0x09, 0x39, 0x70}

};

bool isAlive[MAX_DEVICES] = {false};

struct generalMsgStruct {
  uint8_t id_mac[6];
  int messageType;
  unsigned long medicationTime;
  char checksum[MAX_CHECKSUM_LENGTH];
};

generalMsgStruct msg;
unsigned long nextMedication = 0;

void computeCheckSum() {
  unsigned char checksum = 0;
  for (int i = 0; i < 6; i++) checksum ^= msg.id_mac[i];
  checksum ^= msg.messageType;
  snprintf(msg.checksum, MAX_CHECKSUM_LENGTH, "%02X", checksum);
}

void onReceive(const uint8_t *mac, const uint8_t *data, int len) {
  if (len == sizeof(generalMsgStruct)) {
    memcpy(&msg, data, len);
    lastHeartbeatTime = millis();

    char expectedChecksum[MAX_CHECKSUM_LENGTH];
    unsigned char checksum = 0;
    for (int i = 0; i < 6; i++) checksum ^= msg.id_mac[i];
    checksum ^= msg.messageType;
    snprintf(expectedChecksum, MAX_CHECKSUM_LENGTH, "%02X", checksum);

    if (strcmp(expectedChecksum, msg.checksum) == 0) {
      Serial.print("\nHeartbeat from MAC: ");
      for (int i = 0; i < 6; i++) {
        Serial.printf("%02X", mac[i]);
        if (i < 5) Serial.print(":");
      }
      Serial.println();

      for (int i = 0; i < MAX_DEVICES; i++) {
        bool match = true;
        for (int j = 0; j < 6; j++) {
          if (mac[j] != serverAddress[i][j]) {
            match = false;
          }
        }
        if (match) {
          isAlive[i] = true;
        }
      }

      // Respond if the message is a heartbeat
      if (msg.messageType == HEARTBEAT_MSG) {

        msg.messageType = SERVER_MSG;
        msg.medicationTime = nextMedication;
        computeCheckSum();
        esp_err_t result = esp_now_send(msg.id_mac, (uint8_t*)&msg, sizeof(msg));

        if (result == ESP_OK) Serial.println("Reply sent successfully.");
        else Serial.printf("Failed to send reply. Code: %d\n", result);
      }
    } else {
      Serial.println("Checksum mismatch!");
    }
  } else {
    Serial.println("Invalid message length");
  }
}

void OnSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("ESP-NOW Send status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");

  if (status != ESP_NOW_SEND_SUCCESS && retryCount < MAX_ATTEMPTS) {
    retryCount++;
    esp_now_send(msg.id_mac, (uint8_t*)&msg, sizeof(msg));
  } else {
    retryCount = 0;
  }
}

void mqttTask(void* param) {
  for (;;) {
    if (!mqttClient.connected()) {
      if (mqttClient.connect("ESP32Server","admin","Qs2NZ3q6")) {
        mqttClient.subscribe("medication_adherence/medication");
      }
    } else {
      mqttClient.loop();
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void init_mass_esp_peer() {
  for (int i = 0; i < MAX_DEVICES; i++) {
    memcpy(peerInfo[i].peer_addr, serverAddress[i], 6);
    peerInfo[i].channel = 0;
    peerInfo[i].encrypt = false;
    if (esp_now_add_peer(&peerInfo[i]) == ESP_OK) {
      Serial.println("ESP-NOW peer added.");
    }
  }
}

void sendMedicationStatus(int deviceIndex) {
  WiFi.macAddress(msg.id_mac);
  msg.messageType = SERVER_MSG;
  msg.medicationTime = nextMedication;
  computeCheckSum(); 

  esp_err_t result = esp_now_send(serverAddress[deviceIndex], (uint8_t*)&msg, sizeof(msg));
  
  if (result == ESP_OK) {
    Serial.printf("Sent message to device %d: %s\n", deviceIndex);
  } else {
    Serial.printf("Failed to send message to device %d. Error: %d\n", deviceIndex);
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  char *payloadString = (char*)payload; 
  payloadString[length] = '\0'; 

  if (strcmp(topic, "medication_adherence/medication") == 0) {
    if (strstr(payloadString, "medication_taken") != NULL) {
      nextMedication = millis() + 60000;

      for (int i = 0; i < MAX_DEVICES; i ++){
        if(isAlive[i]){
          sendMedicationStatus(i);
        }
      }
    } else {
        DynamicJsonDocument doc(1024);  // Create a document with enough space for your JSON

        DeserializationError error = deserializeJson(doc, payloadString);

        if (error) {
          Serial.print("Failed to deserialize JSON: ");
          Serial.println(error.f_str());
          return;
        }

        // Access the values in the JSON document
        long weight_change = doc["weight_change"];
        unsigned long time = doc["time"].as<unsigned long long>() % 4294967296UL;

        nextMedication = time;
    }
  }
}

void connectWiFi() {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) delay(500);
  Serial.printf("Wi-Fi Connected on Channel: %d\n", WiFi.channel());

  esp_wifi_set_channel(WiFi.channel(), WIFI_SECOND_CHAN_NONE);
  WiFi.setSleep(false);
}

void setup() {
  M5.begin();
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  connectWiFi();
  mqttClient.setServer(MQTT_SERVER, 1883);
  mqttClient.setCallback(callback);

  uint8_t mac[6];  // Array to store MAC address
  esp_wifi_get_mac(WIFI_IF_STA, mac);  // Get MAC address of Wi-Fi interface

  // Print the MAC address
  Serial.print("MAC Address: ");
  for (int i = 0; i < 6; i++) {
    if (mac[i] < 0x10) {
      Serial.print("0");
    }
    Serial.print(mac[i], HEX);
    if (i < 5) {
      Serial.print(":");
    }
  }
  Serial.println();

  xTaskCreatePinnedToCore(mqttTask, "MQTTTask", 16384, NULL, 1, &mqttTaskHandle, 1);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }

  esp_now_register_recv_cb(onReceive);
  esp_now_register_send_cb(OnSent);
  init_mass_esp_peer();

  Serial.printf("Free Heap: %d bytes\n", ESP.getFreeHeap());

  lastHeartbeatTime = millis();
  nextMedication = millis() + 150000;
}

unsigned long checkTime = 0;
unsigned long lastResetTime = 0;  


void loop() {
  unsigned long now = millis();

  // Publish heartbeat message if enough time has passed
  if (now - lastHeartbeatTime >= LAST_HEARTBEAT_MSG) {
    if (mqttClient.connected()) {
      mqttClient.publish(MQTT_TOPIC, "HeartbeatMsg from server");
    }
    lastHeartbeatTime = now;
  }

  // Check if 2 minutes have passed for medication adherence check
  if ((now - nextMedication) >= 120000 && (now - checkTime) >= 120000) {
    if (mqttClient.connected()) {

      mqttClient.publish("medication_adherence/medication", "medication_not_taken");
    }
    checkTime = now;
  }

  // Print the alive list every 60 seconds
  if (now - lastResetTime >= 60000) {
    // Reset all devices to not alive
    memset(isAlive, 0, sizeof(isAlive));  // Set all to false (0)
    lastResetTime = now;  // Update the last reset time
    Serial.println("Resetting alive devices list.");
  }

}