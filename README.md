# CSC2106-IoT: Medication Nonadherence Project
## Project Description
Medication nonadherence is a growing concern, particularly among the elderly population who may forget to take their prescribed medication on time or lack the ability to interact with modern reminder systems such as smartphones. This project aims to address this challenge by leveraging the Internet of Things (IoT) to provide a seamless, non-intrusive solution that helps ensure medication compliance without relying on traditional mobile devices.

Our solution uses multiple ESP32 devices communicating via ESP-NOW. The main M5StickC ESP32 unit acts as a server, responsible for processing incoming heartbeat signals and activating reminders. The client ESP32 device periodically sends out heartbeat messages and utilizes Bluetooth Low Energy (BLE) to detect the proximity of a wearable device or beacon carried by the elderly user. Upon confirming the user's presence, the server ESP32 triggers reminders via a buzzer and LED indicators to notify the user when it is time to take their medication.

To make the system even more intelligent and responsive, a pressure sensor is integrated beneath the medication container (such as a pillbox or bottle). When the user lifts the container, the pressure change is detected, providing physical confirmation that the medication has been taken and prevents unnecessary pill consumption. This adds a crucial layer of accountability and automation without requiring the user to press any buttons or interact with a screen.

The use of RSSI (Received Signal Strength Indicator) values ensures that reminders are only activated when the user is within a practical proximity to the medication unit. Combined with the lightweight ESP-NOW protocol, the system remains fast, energy-efficient, and reliable — suitable for battery-powered and resource-constrained environments.

## Objectives
The primary objective of this project is to address the growing issue of medication nonadherence, particularly among the elderly. As the global population ages, there is an increasing number of elderly individuals who may suffer from memory-related issues or cognitive decline, making it easy for them to forget to take their prescribed medication on time. In many cases, these individuals live alone or do not have someone readily available to assist them with daily medication routines.

Our solution aims to:
- Provide an IoT-based reminder system that does not rely on smartphones or manual interaction, making it suitable for elderly users.
- Detect the user’s presence using BLE and RSSI values, and trigger timely reminders only when necessary.
- Help maintain consistent medication intake to improve health outcomes and reduce hospitalization risks.
- Offer caregivers peace of mind by automating part of the medication adherence process

## Installation Guide 

1. **Hardware Setup:**
   - Connect the ESP32 devices (M5StickC for the server, and other ESP32 for the client).
   - Attach the pressure sensor to the medication container.
   - Set up the BLE beacon or wearable device for proximity detection.

2. **Software Setup:**
   - Clone this repository to your local machine.
     ```bash
     git clone https://github.com/jl-sit-cs/CSC2106-IoT.git
     ```
   - Upload the code to your ESP32 devices using the Arduino IDE.
3. ** Pressure Weight Sensor Setup: **
    - To be added 



## How it works
1. Client ESP32

- The client ESP32 handles the core logic, including receiving medication timing and performing BLE scanning to detect the presence of the user via a wearable device or beacon.
- It uses RSSI (Received Signal Strength Indicator) values to determine if the user is within a practical range of the medication unit.
- Upon confirming the user's presence, the client triggers reminders via buzzer and LED to notify the user when it is time to take their medication.
- The client periodically sends heartbeat signals to the server to maintain the connection.

2. Server ESP32 (M5StickC)

- The server ESP32 (M5StickC) acts as a bridge to receive any updates on medication timing and other related information. It primarily receives MQTT updates from the client regarding medication schedules or reminders.

3. Pressure Sensor

- A pressure sensor is integrated beneath the medication container (such as a pillbox or bottle). This sensor detects when the container is lifted, which provides physical confirmation that the medication has been taken.
= This prevents unnecessary pill consumption and ensures that the user is correctly adhering to the medication schedule without the need for manual input or screen interaction.

4. RSSI and Proximity Detection

- The system uses RSSI values to only trigger reminders when the user is within a reasonable distance from the medication unit, reducing false alarms and improving usability.

5. Energy Efficiency

- Using ESP-NOW as the communication protocol, the system remains lightweight, fast, and energy-efficient, making it ideal for battery-powered devices in resource-constrained environments.

## Usage

Once the system is set up, the ESP32 devices will:
- Continuously monitor the proximity of the user via BLE.
- Trigger reminders when the user is in range of the medication unit.
- Confirm medication intake using the integrated pressure sensor.

Users will receive notifications through the buzzer and LED indicators when it's time to take their medication.

## License 
This project is licensed under the MIT License, see the full MIT License text at https://opensource.org/licenses/MIT.

## Contact Us 
If you have any questions, suggestions, or feedback, feel free to reach out to us!
- Email: 2301075@sit.singaporetech.edu.sg
We'd love to hear from you!


