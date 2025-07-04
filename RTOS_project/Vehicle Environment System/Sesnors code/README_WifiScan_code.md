# ESP32 WiFi Scanner

This project scans for all available WiFi networks using the built-in WiFi functionality of the **ESP32 development board**. It prints the **network names (SSIDs)** and their **signal strength (RSSI)** to the **Serial Monitor**. This is a simple utility project that helps identify nearby WiFi networks for connectivity diagnostics or development.

## GitHub Repository

- [Main Repository](https://github.com/AbisheckD/Projects_at_PSG/tree/main)
- [Project Folder](https://github.com/AbisheckD/Projects_at_PSG/tree/main/WiFi_Scanner)

## Hardware Required

- ESP32 development board
- USB Cable (for programming and serial monitoring)

## Software Required

- Arduino IDE
- ESP32 Board Support Package (Install via Board Manager)
- WiFi Library (included by default in ESP32 core)

## Features

- Scans and lists all available WiFi networks
- Displays SSID (name) and RSSI (signal strength in dBm)
- Uses `Serial Monitor` for output
- Simple, efficient implementation using native ESP32 WiFi APIs

## Arduino Wiring

Component ESP32 with Wifi and Provide power supply to ESP32  

> ✅ **Note:**  
> This requires no external hardware or sensors. Simply upload and run on any ESP32 board.

## Code

```c
#include <WiFi.h>

void setup() {
  Serial.begin(115200);
  Serial.println("Scanning WiFi...");

  int networks = WiFi.scanNetworks();
  if (networks == 0) {
    Serial.println("No networks found.");
  } else {
    Serial.println("Networks found:");
    for (int i = 0; i < networks; ++i) {
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(WiFi.SSID(i));
      Serial.print(" (");
      Serial.print(WiFi.RSSI(i));
      Serial.println(" dBm)");
    }
  }
}

void loop() {}
```
##OUTPUT
```yash
Scanning WiFi...
Networks found:
1: HomeWiFi (−45 dBm)
2: OfficeNet (−65 dBm)
3: MobileHotspot (−70 dBm)
```

## Getting Started

- Open the Arduino IDE
- Install the ESP32 board support via Board Manager
- Select your ESP32 board and appropriate COM port
- Copy and paste the provided code into a new sketch
- Upload to your ESP32 board
- Open the Serial Monitor at 115200 baud to view the output

## Reference

- ESP32 WiFi Library (https://github.com/espressif/arduino-esp32)
- Official ESP32 Documentation: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/

## Notes

- The scan is performed once during setup; reset the ESP32 to rescan
- RSSI closer to 0 dBm indicates stronger signal (e.g., −30 dBm is better than −80 dBm)
- No credentials are required to scan; only reading available SSIDs

## License

- This project is licensed under the MIT License.

## Clone the Repo

```bash
git clone https://github.com/AbisheckD/Projects_at_PSG.git
```
