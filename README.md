# Smart Locker Security and Environmental Monitoring System

## Overview

https://github.com/user-attachments/assets/cba112b6-b64e-4834-82ce-c1d546b76cc3


This project is a smart locker security and environmental monitoring system based on the NUCLEO-L432KC development board and Mbed OS. The system uses a physical button-based password interface for access control. Once unlocked, it automatically activates environmental monitoring functions, including abnormal temperature sensing and vibration-based fall/tamper detection.

## Core Features
*   **Button-Based Authentication:** Utilizes interrupt-driven physical buttons for a 3-digit password verification (Default: `1-2-1`), eliminating continuous CPU polling to conserve power.
*   **Temperature Monitoring:** Uses the TMP102 sensor (via I2C) to read real-time temperature. It triggers LED alerts if the temperature exceeds 29°C or drops below 28°C.
*   **Tamper & Fall Detection:** Leverages the ADXL345 accelerometer (via SPI) to monitor abnormal acceleration (threshold: ±0.5g) and tilt angles (>45°).
*   **Low-Power Management:** When the system is locked or idle, the temperature sensor is placed into a custom Shutdown Mode via register configuration, significantly reducing peripheral power consumption.

## Hardware Components
*   **Microcontroller:** NUCLEO-L432KC
*   **Sensors:** ADXL345 Accelerometer, TMP102 Temperature Sensor
*   **Peripherals:** 2x Push Buttons, 3x Status LEDs

## Software Architecture
*   Developed in **C++** using the **Mbed OS** framework.
*   Implements **Event Queue** and multi-threading to handle interrupt logic efficiently, ensuring stable and non-blocking system performance.

## Future Work
*   Upgrade the ADXL345 data acquisition from continuous polling to an interrupt-driven approach for further power efficiency.
*   Implement runtime dynamic password configuration.
*   Integrate a buzzer module to provide audible alarms.
