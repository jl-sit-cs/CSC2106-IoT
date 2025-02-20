 #include "pico/stdlib.h"
 #include <stdio.h>
 #include <math.h>
 
 #define DT_PIN 2  // Data pin
 #define SCK_PIN 3 // Clock pin
 
 #define CALIBRATION_FACTOR 1000 // Calibration factor
 #define NUM_ZERO_READINGS 100
 
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
 
 int main() {
     stdio_init_all();
     gpio_init(DT_PIN);
     gpio_set_dir(DT_PIN, GPIO_IN);
 
     gpio_init(SCK_PIN);
     gpio_set_dir(SCK_PIN, GPIO_OUT);

    int32_t zero_offset = calibrate_zero_offset();
    printf("Zero Offset: %ld\n", zero_offset);
 
    printf("HX711 Initialized!\n");
 
    while (1) {
        int32_t raw_value = hx711_read();

        // Subtract the zero offset and apply the calibration factor
        float weight = (float)(raw_value - zero_offset) / CALIBRATION_FACTOR;

        int rounded_weight = (int)round(weight);

        printf("Weight: %d grams\n", rounded_weight);
        sleep_ms(500);
    }
 }
 
