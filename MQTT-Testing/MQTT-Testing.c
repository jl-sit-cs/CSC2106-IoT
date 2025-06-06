#include <string.h>
#include <time.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

#include "lwip/pbuf.h"
#include "lwip/tcp.h"
#include "lwip/dns.h"
#include "lwip/apps/mqtt.h"

#include <stdio.h>
#include <math.h>

#define DT_PIN 2  // Data pin
#define SCK_PIN 3 // Clock pin

#define CALIBRATION_FACTOR 1000 // Calibration factor
#define NUM_ZERO_READINGS 100

#define DEBUG_printf printf

// MQTT server configuration
#define MQTT_SERVER_PORT 1883
#define MQTT_TLS 1
#define MQTT_CLIENT_ID "PicoW"
#define MQTT_USER "admin" //Replace with the dashboard pass
#define MQTT_PASS "Qs2NZ3q6" //Replace with the dashboard pass

// Wi-Fi configuration
#define WIFI_SSID "JL"
#define WIFI_PASSWORD "helloworld2024"

#define WEIGHT_THRESHOLD 20         // Minimum weight change to consider significant
#define MIN_PICKUP_DURATION 10000   // Minimum pickup duration in milliseconds (5 seconds)
#define MAX_PICKUP_DURATION 60000  // Maximum pickup duration in milliseconds (60 seconds)
#define RETURN_THRESHOLD 20        // Weight threshold for returning the bottle

int weight_change = 0; // Global variable to store weight change

typedef struct MQTT_CLIENT_T_ {
    ip_addr_t remote_addr;
    mqtt_client_t *mqtt_client;
    u32_t received;
    u32_t counter;
} MQTT_CLIENT_T;

typedef struct {
    int initial_weight;
    int previous_weight;
    uint32_t pickup_start_time;
    bool is_picked_up;
    bool medication_taken_reported;
    bool is_start;
} MedicationTracker;

err_t mqtt_test_publish(MQTT_CLIENT_T *state);
err_t mqtt_test_connect(MQTT_CLIENT_T *state);

// Memory allocation and initialization
static MQTT_CLIENT_T* mqtt_client_init(void) {
    MQTT_CLIENT_T *state = (MQTT_CLIENT_T *)calloc(1, sizeof(MQTT_CLIENT_T));
    if (!state) {
        DEBUG_printf("failed to allocate state\n");
        return NULL;
    }
    state->received = 0;
    state->counter = 0;
    return state;
}

void init_medication_tracker(MedicationTracker *tracker, int initial_weight) {
    tracker->initial_weight = initial_weight;
    tracker->previous_weight = initial_weight;
    tracker->pickup_start_time = 0;
    tracker->is_picked_up = false;
    tracker->medication_taken_reported = false;
    tracker->is_start = false;
}

bool detect_medication_taking(MedicationTracker *tracker, int current_weight, uint32_t current_time) 
{
    weight_change = abs(current_weight - tracker->previous_weight);

    // Step 1: Bottle placed on the tracker
    if (!tracker->is_start && weight_change > WEIGHT_THRESHOLD) {
        tracker->is_start = true;
        tracker->initial_weight = current_weight;
        tracker->previous_weight = current_weight;
        DEBUG_printf("Bottle Placed\n");
        return false;
    }

    // Step 2: Bottle picked up
    if (tracker->is_start && !tracker->is_picked_up && 
        current_weight < 1) 
    {
        tracker->pickup_start_time = current_time;
        tracker->is_picked_up = true;
        tracker->medication_taken_reported = false;
        DEBUG_printf("Bottle picked up. Initial weight: %d, Current weight: %d\n", tracker->initial_weight, current_weight);
        return false;
    }

    // Step 3: Check for medication taken
    if (tracker->is_start && tracker->is_picked_up && !tracker->medication_taken_reported) {
        uint32_t duration = current_time - tracker->pickup_start_time;
        if (duration >= MAX_PICKUP_DURATION) {
            tracker->medication_taken_reported = true;
            tracker->is_picked_up = false;
            DEBUG_printf("Medication likely taken. Duration: %d ms\n", duration);
            return true;
        }
    }

    // Step 4: Bottle returned (regardless of report status)
    if (tracker->is_start && tracker->is_picked_up && 
        weight_change >= RETURN_THRESHOLD && 
        (current_time - tracker->pickup_start_time) >= MIN_PICKUP_DURATION)
    
    {
        DEBUG_printf("Bottle returned. Initial: %d, Current: %d, Δ: %d\n", 
                     tracker->initial_weight, current_weight, abs(current_weight - tracker->previous_weight));

        // Reset tracker
        tracker->is_picked_up = false;

        return true; // Return true if meds were actually taken
    }

    return false;
}



// Set static IP for local MQTT broker
void set_mqtt_server_ip(MQTT_CLIENT_T *state) {
    IP4_ADDR(&state->remote_addr, 172, 20, 10, 13);  // Replace with your laptop's IP
}


u32_t data_in = 0;
u8_t buffer[1025];
u8_t data_len = 0;

// Function to read raw data from HX711
int32_t hx711_read(void) {
    int32_t value = 0;
    uint8_t i;

    // Wait for HX711 to be ready (DT goes LOW)
    while (gpio_get(DT_PIN));

    // Read 24-bit data from HX711
    for (i = 0; i < 24; i++) {
        gpio_put(SCK_PIN, 1);
        sleep_us(1);
        value = (value << 1) | gpio_get(DT_PIN);
        gpio_put(SCK_PIN, 0);
        sleep_us(1);
    }

    // Send extra clock pulse to set gain (128 by default)
    gpio_put(SCK_PIN, 1);
    sleep_us(1);
    gpio_put(SCK_PIN, 0);
    sleep_us(1);

    // Convert to signed 32-bit
    if (value & 0x800000) { // If MSB is 1 (negative number)
        value |= 0xFF000000; // Sign-extend to 32-bit
    }

    return value;
}

int32_t calibrate_zero_offset(void) {
   int32_t sum = 0;
   for (int i = 0; i < NUM_ZERO_READINGS; i++) {
       sum += hx711_read();
       sleep_ms(100);  // Wait a bit between readings
   }
   return sum / NUM_ZERO_READINGS;  // Average value
}


// Callback functions
static void mqtt_pub_start_cb(void *arg, const char *topic, u32_t tot_len) {
    DEBUG_printf("mqtt_pub_start_cb: topic %s\n", topic);
    if (tot_len > 1024) {
        DEBUG_printf("Message length exceeds buffer size, discarding\n");
    } else {
        data_in = tot_len;
        data_len = 0;
    }
}

static void mqtt_pub_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags) {
    if (data_in > 0) {
        data_in -= len;
        memcpy(&buffer[data_len], data, len);
        data_len += len;

        if (data_in == 0) {
            buffer[data_len] = 0;
            DEBUG_printf("Message received: %s\n", buffer);
        }
    }
}

static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status) {
    MQTT_CLIENT_T *state = (MQTT_CLIENT_T *)arg;

    if (status != 0) {
        DEBUG_printf("Error during connection: err %d.\n", status);
    } else {
        DEBUG_printf("MQTT connected.\n");

        // Publish a message immediately upon successful connection
        if (mqtt_test_publish(state) == ERR_OK) {
            DEBUG_printf("Initial message published after connection.\n");
        }
    }
}

void mqtt_pub_request_cb(void *arg, err_t err) {
    MQTT_CLIENT_T *state = (MQTT_CLIENT_T *)arg;
    DEBUG_printf("mqtt_pub_request_cb: err %d\n", err);
    state->received++;
}

void mqtt_sub_request_cb(void *arg, err_t err) {
    DEBUG_printf("mqtt_sub_request_cb: err %d\n", err);
}

err_t mqtt_test_publish(MQTT_CLIENT_T *state) {
    char buffer[128];
    sprintf(buffer, "{\"message\":\"hello from picow %d / %d\"}", state->received, state->counter);

    err_t err;
    u8_t qos = 0;
    u8_t retain = 0;

    cyw43_arch_lwip_begin();
    err = mqtt_publish(state->mqtt_client, "pico_w/test", buffer, strlen(buffer), qos, retain, mqtt_pub_request_cb, state);
    cyw43_arch_lwip_end();

    if (err != ERR_OK) {
        DEBUG_printf("Publish err: %d\n", err);
    }

    return err;
}

err_t mqtt_test_connect(MQTT_CLIENT_T *state) {
    struct mqtt_connect_client_info_t ci;
    err_t err;

    memset(&ci, 0, sizeof(ci));

    ci.client_id = "PicoW";
    ci.client_user = MQTT_USER;
    ci.client_pass = MQTT_PASS;
    ci.keep_alive = 60;

    err = mqtt_client_connect(state->mqtt_client, 
        &(state->remote_addr), MQTT_SERVER_PORT, 
        mqtt_connection_cb, state, &ci);
    if (err != ERR_OK) {
        DEBUG_printf("mqtt_connect return %d\n", err);
    }

    return err;
}

void connect_to_mqtt(MQTT_CLIENT_T *state) {
    while (mqtt_test_connect(state) != ERR_OK) {
        DEBUG_printf("MQTT connection failed. Retrying in 5 seconds...\n");
        sleep_ms(5000);
    }
    DEBUG_printf("Connected to MQTT!\n");
}

void connect_to_wifi() {
    DEBUG_printf("Connecting to WiFi...\n");
    while (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        DEBUG_printf("WiFi connection failed. Retrying in 5 seconds...\n");
        sleep_ms(5000);
    }
    DEBUG_printf("Connected to WiFi!\n");
}

void mqtt_run_test(MQTT_CLIENT_T *state) {
    state->mqtt_client = mqtt_client_new();
    if (!state->mqtt_client) {
        DEBUG_printf("Failed to create new MQTT client\n");
        return;
    }

    connect_to_mqtt(state);

    absolute_time_t timeout = nil_time;
    bool subscribed = false;

    mqtt_set_inpub_callback(state->mqtt_client, mqtt_pub_start_cb, mqtt_pub_data_cb, 0);

    while (true) {
        cyw43_arch_poll();
        absolute_time_t now = get_absolute_time();

        // Reconnect if MQTT is disconnected
        if (!mqtt_client_is_connected(state->mqtt_client)) {
            DEBUG_printf("MQTT lost. Reconnecting...\n");
            break;
            //connect_to_mqtt(state);
        }

        // Ensure Wi-Fi is still connected
        if (!cyw43_wifi_link_status(&cyw43_state, CYW43_ITF_STA)) {
            DEBUG_printf("WiFi lost. Reconnecting...\n");
            connect_to_wifi();
            connect_to_mqtt(state); // Ensure MQTT reconnects after Wi-Fi is back
        }

        if (is_nil_time(timeout) || absolute_time_diff_us(now, timeout) <= 0) {
            if (mqtt_client_is_connected(state->mqtt_client)) {
                cyw43_arch_lwip_begin();

                // Subscribe to topic if not already done
                if (!subscribed) {
                    mqtt_sub_unsub(state->mqtt_client, "pico_w/recv", 1, mqtt_sub_request_cb, 0, 1);
                    subscribed = true;
                }

                if (mqtt_test_publish(state) == ERR_OK) {
                    if (state->counter != 0) {
                        DEBUG_printf("Published %d\n", state->counter);
                    }
                    timeout = make_timeout_time_ms(5000);
                    state->counter++;
                }

                cyw43_arch_lwip_end();
            }
        }
    }
}

int main() {
    stdio_init_all();

    if (cyw43_arch_init()) {
        DEBUG_printf("failed to initialise\n");
        return 1;
    }
    cyw43_arch_enable_sta_mode();

    connect_to_wifi();

    MQTT_CLIENT_T *state = mqtt_client_init();
    if (!state) {
        DEBUG_printf("Failed to initialize MQTT client state\n");
        return 1;
    }

    set_mqtt_server_ip(state);  // Set the IP manually
    mqtt_run_test(state);

    gpio_init(DT_PIN);
    gpio_set_dir(DT_PIN, GPIO_IN);

    gpio_init(SCK_PIN);
    gpio_set_dir(SCK_PIN, GPIO_OUT);

   int32_t zero_offset = calibrate_zero_offset();
   printf("Zero Offset: %ld\n", zero_offset);

   printf("HX711 Initialized!\n");

    // Initialize medication tracker
    MedicationTracker medication_tracker;

    init_medication_tracker(&medication_tracker, 0);

    while (1) {
        // Get current time
        uint32_t current_time = to_ms_since_boot(get_absolute_time());

        // Read current weight
        int32_t raw_value = hx711_read();
        float weight = (float)(raw_value - zero_offset) / CALIBRATION_FACTOR;
        int rounded_weight = (int)round(weight);
        
        // DEBUG_printf("rounded_weight: %d\n", rounded_weight);
        // Detect medication taking
        bool medication_taken = detect_medication_taking(&medication_tracker,  abs(rounded_weight), current_time);

        // Publish medication status if taken
        if (medication_taken) {
            char weight_message[128];
            medication_tracker.previous_weight = abs(rounded_weight);
            
            sprintf(weight_message, "{\"status\":\"medication_taken\", \"weight\":%d, \"weight_change\":%d}", 
                abs(rounded_weight), abs(weight_change));
            
            printf("Weight: %d\n", abs(rounded_weight));
            printf("Weight Change: %d\n", abs(weight_change));
            cyw43_arch_lwip_begin();

            err_t err = mqtt_publish(state->mqtt_client, "medication_adherence/medication_weight", 
                                    weight_message, strlen(weight_message), 
                                        0, 0, mqtt_pub_request_cb, state);
            cyw43_arch_lwip_end();
            

            if (err != ERR_OK) {
                DEBUG_printf("Failed to publish medication status or weight\n");
            } else {
                DEBUG_printf("Medication taken status published\n");
            }
        }

        sleep_ms(1000);  // Reduced sleep time for more responsive tracking
    }
    
    // Free memory and deinitialize Wi-Fi
    free(state);
    cyw43_arch_deinit();
    
    return 0;
}
