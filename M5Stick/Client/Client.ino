#include <M5StickCPlus.h>
#include <esp_now.h>
#include <WiFi.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "esp_wifi.h"
#include "esp_bt.h"

#define CHANNEL 1
#define MAX_CHECKSUM_LENGTH 5
#define HEARTBEAT_MSG 3
#define SERVER_MSG 4

TaskHandle_t bleScanTaskHandle;
TaskHandle_t espNowTaskHandle;

SemaphoreHandle_t bleEspMutex;

struct generalMsgStruct {
  uint8_t id_mac[6];
  int messageType;
  unsigned long medicationTime;
  char checksum[MAX_CHECKSUM_LENGTH];
};


generalMsgStruct currentMessage;
unsigned long medicationTime = 0;

bool phoneDetected = false;

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) override {
    Serial.print("BLE Advertised Device found: ");
    Serial.println(advertisedDevice.toString().c_str());

    int rssiThreshold = -60;

    if (advertisedDevice.getRSSI() > rssiThreshold) {
      phoneDetected = true;
      Serial.println("Phone detected with sufficient signal strength (RSSI > threshold).");
      BLEDevice::getScan()->stop();
    }
  }
};

void handle_buzzer_on() {
  digitalWrite(M5_LED, 0);
  M5.Beep.tone(4000, 300);
  M5.update();
}

void handle_buzzer_off() {
  digitalWrite(M5_LED, 1);
  M5.Beep.tone(0, 0);
  M5.update();
}

void enableCoexistence() {
  if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_IDLE) {
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if (esp_bt_controller_init(&bt_cfg) != ESP_OK) return;
    if (esp_bt_controller_enable(ESP_BT_MODE_BLE) != ESP_OK) return;
  }

  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_channel(6, WIFI_SECOND_CHAN_NONE);
}

void computeCheckSum() {
  unsigned char checksum = 0;
  for (int i = 0; i < 6; i++) checksum ^= currentMessage.id_mac[i];
  checksum ^= currentMessage.messageType;
  snprintf(currentMessage.checksum, MAX_CHECKSUM_LENGTH, "%02X", checksum);
}

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status != ESP_NOW_SEND_SUCCESS) {
    Serial.println("❌ ESP-NOW send failed.");
  }
}

void onReceive(const uint8_t *mac, const uint8_t *data, int len) {
  if (len == sizeof(generalMsgStruct)) {
    memcpy(&currentMessage, data, len);

    char expectedChecksum[MAX_CHECKSUM_LENGTH];
    unsigned char checksum = 0;
    for (int i = 0; i < 6; i++) checksum ^= currentMessage.id_mac[i];
    checksum ^= currentMessage.messageType;
    snprintf(expectedChecksum, MAX_CHECKSUM_LENGTH, "%02X", checksum);

    if (strcmp(expectedChecksum, currentMessage.checksum) == 0) {
      if (currentMessage.messageType == SERVER_MSG) {
        medicationTime = currentMessage.medicationTime;
        Serial.println("Server message received.");
      }
    } else {
      Serial.println("Checksum mismatch.");
    }
  }
}

void espNowTask(void *pvParameters) {
  uint8_t peerMac[] = {0x4C, 0x75, 0x25, 0xCB, 0x89, 0x98}; 
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, peerMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    vTaskDelete(NULL);
  }


  while (1) {
    if (xSemaphoreTake(bleEspMutex, pdMS_TO_TICKS(2000)) == pdTRUE) {
      WiFi.macAddress(currentMessage.id_mac);
      currentMessage.messageType = HEARTBEAT_MSG;
      currentMessage.medicationTime = millis();
      computeCheckSum();

      esp_now_send(peerMac, (uint8_t*)&currentMessage, sizeof(currentMessage));

      xSemaphoreGive(bleEspMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(3000));
  }
}

void bleScanTask(void *pvParameters) {
  BLEDevice::init("ESP32_BLE_Scanner");
  BLEScan *pBLEScan = BLEDevice::getScan();

  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);

  MyAdvertisedDeviceCallbacks scanCallbacks;
  pBLEScan->setAdvertisedDeviceCallbacks(&scanCallbacks);
  pBLEScan->start(3, false);

  while (1) {
    phoneDetected = false;
    if (xSemaphoreTake(bleEspMutex, pdMS_TO_TICKS(2000)) == pdTRUE) {
      Serial.println("Starting BLE scan...");
      pBLEScan->start(3, false);
      xSemaphoreGive(bleEspMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(10000));
  }
}

void setup() {
  Serial.begin(115200);
  int x = M5.IMU.Init(); 
  if (x != 0) Serial.println("IMU initialisation fail!");  
  M5.begin();
  M5.Lcd.setRotation(3);
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setCursor(0, 0, 2);
  M5.Lcd.printf("Start Initialization");

  pinMode(M5_LED, OUTPUT);
  pinMode(M5_BUTTON_HOME, INPUT);
  pinMode(BUTTON_B_PIN, INPUT);
  digitalWrite(M5_LED, 1); 

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Initialization Failed.");
    return;
  }

  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onReceive);

  bleEspMutex = xSemaphoreCreateMutex();
  enableCoexistence();

  xTaskCreatePinnedToCore(espNowTask, "ESP_NOW_TASK", 4096, NULL, 1, &espNowTaskHandle, 0);
  xTaskCreatePinnedToCore(bleScanTask, "BLE_SCAN_TASK", 4096, NULL, 1, &bleScanTaskHandle, 1);
}

#define BUTTON_DEBOUNCE_TIME 300
unsigned long lastButtonPress = 0;
unsigned long lastSendTime = 0;

void loop() {
  unsigned long now = millis();
  if (phoneDetected && now - medicationTime >= 1000 && now - lastSendTime >= 8000) {
    handle_buzzer_on();
    lastSendTime = now;
  }

  if (digitalRead(M5_BUTTON_HOME) == LOW && (now - lastButtonPress) > BUTTON_DEBOUNCE_TIME) {
    lastButtonPress = now;
    handle_buzzer_off();
    while (digitalRead(M5_BUTTON_HOME) == LOW);
  }
  delay(100);
  yield();
}