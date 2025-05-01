# Vehicle Environment System - DHT11

This project reads temperature and humidity data using the DHT11 sensor and displays the results over the serial monitor. It is part of the broader `RTOS_project` repository under the `Projects_at_PSG` collection.

## GitHub Repository

- [Main Repository](https://github.com/AbisheckD/Projects_at_PSG/tree/main)
- [Project Folder](https://github.com/AbisheckD/Projects_at_PSG/tree/main/RTOS_project/Vehicle%20Environment%20System)

## Hardware Required

- Arduino-compatible board (ESP32, Uno, Mega, etc.)
- DHT11 Temperature and Humidity Sensor
- Jumper wires
- USB Cable

## Software Required

- Arduino IDE
- Adafruit DHT Sensor Library
- Adafruit Unified Sensor Library

## Features

- Reads humidity and temperature using DHT11 sensor
- Serial output every 2 seconds
- Error handling when sensor read fails

## Arduino Wiring

| DHT11 Pin | Arduino Pin |
|-----------|-------------|
| VCC       | 5V          |
| GND       | GND         |
| DATA      | D4          |

> Ensure the data pin from the DHT11 is connected to digital pin D4 as defined in the code.

## Code

```cpp
#include "DHT.h"

#define DHTPIN 4
#define DHTTYPE DHT11

DHT dht(DHTPIN, DHTTYPE);

void setup() {
  Serial.begin(115200);
  dht.begin();
}

void loop() {
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();
  
  if (isnan(humidity) || isnan(temperature)) {
    Serial.println("Failed to read from DHT sensor!");
  } else {
    Serial.print("Humidity: ");
    Serial.print(humidity);
    Serial.print("%  Temperature: ");
    Serial.print(temperature);
    Serial.println("°C");
  }

  delay(2000);
}
```
## Output

```
Humidity: 54.00%  Temperature: 25.00°C
Humidity: 53.00%  Temperature: 24.80°C
Humidity: 55.00%  Temperature: 25.20°C
```
## Getting Started

- Open Arduino IDE.
- Go to Sketch > Include Library > Manage Libraries…
- Search for DHT and install the Adafruit DHT sensor library.
- Also install the Adafruit Unified Sensor library if prompted.
- Connect the DHT11 sensor as per the wiring table.
- Upload the code to your board.
- Open the Serial Monitor at 115200 baud rate to view output.

## Reference

- [Adafruit DHT Sensor Library](https://github.com/adafruit/DHT-sensor-library)
- [Adafruit Unified Sensor Library](https://github.com/adafruit/Adafruit_Sensor)
- [Arduino DHT11 Guide](https://learn.adafruit.com/dht)

## Calibration Tip

You may calibrate the sensor readings by cross-checking with a thermometer and hygrometer for enusre accuracy in surrounding environments.

## License

This project is licensed under the MIT License.

## Clone the Repo

```bash
git clone https://github.com/AbisheckD/Projects_at_PSG.git

```
