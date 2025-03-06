# CSC2106-IoT

## Prerequisite 
Download an MQTT broker
https://mosquitto.org/

Edit the mosquitto.conf to include
```
# mosquitto.conf

listener 1883 0.0.0.0
allow_anonymous true
```

## Start
Edit all relvant parts to include your own laptop's information
```
#define MQTT_SERVER_HOST "192.168.137.1"  // Replace with your laptop's IP
#define MQTT_SERVER_PORT 1883
#define MQTT_TLS 0

#define WIFI_SSID "Wifi SSID"
#define WIFI_PASSWORD "Wifi password"

// Set static IP for local MQTT broker
void set_mqtt_server_ip(MQTT_CLIENT_T *state) {
    IP4_ADDR(&state->remote_addr, 192, 168, 137, 1);  // Replace with your laptop's IP
}
```

Run you mosquitto broker (Change the file path to your mosquitto.conf)
```
 mosquitto -v -c "C:\Program Files\mosquitto\mosquitto.conf"
```

Compile and upload the file to pico



