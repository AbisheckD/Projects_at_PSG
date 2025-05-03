# Vehicle Environment System – DHT11 + Water Level Sensor + MPU6050

This project reads **temperature** and **humidity** using the **DHT11 sensor**, measures **water level** using an analog sensor, and captures **acceleration** and **gyroscope data** using the **MPU6050** sensor. All sensor data is printed to the serial monitor every 2 seconds. This is part of the `RTOS_project` in the `Projects_at_PSG` repository.

## GitHub Repository

- [Main Repository](https://github.com/AbisheckD/Projects_at_PSG/tree/main)
- [Project Folder](https://github.com/AbisheckD/Projects_at_PSG/tree/main/RTOS_project/Vehicle%20Environment%20System)

## Hardware Required

- ESP32 development board
- DHT11 Temperature and Humidity Sensor
- Analog Water Level Sensor
- MPU6050 6-axis Accelerometer + Gyroscope
- Jumper wires
- USB Cable

## Software Required

- Arduino IDE
- Adafruit DHT Sensor Library
- Adafruit Unified Sensor Library
- MPU6050 Library by Electronic Cats or equivalent
- Wire Library (included by default)

## Features

- Reads humidity and temperature from DHT11
- Reads analog water level
- Reads accelerometer and gyroscope data from MPU6050
- Serial output every 2 seconds
- Connection check for MPU6050

## Arduino Wiring

| Sensor         | ESP32 Pin     |
|----------------|---------------|
| DHT11 VCC      | 5V            |
| DHT11 GND      | GND           |
| DHT11 DATA     | D4            |
| Water Sensor VCC | 3.3V/5V     |
| Water Sensor GND | GND         |
| Water Sensor Signal | GPIO34   |
| MPU6050 VCC    | 3.3V or 5V    |
| MPU6050 GND    | GND           |
| MPU6050 SDA    | GPIO21        |
| MPU6050 SCL    | GPIO22        |

## Code

```cpp
#include "DHT.h"
#include <Wire.h>
#include <MPU6050.h>

#define DHTPIN 4
#define DHTTYPE DHT11
#define WATER_LEVEL_PIN 34

DHT dht(DHTPIN, DHTTYPE);
MPU6050 mpu;

void setup() {
  Serial.begin(115200);

  // DHT11 Setup
  dht.begin();

  // Water Level Sensor Setup
  pinMode(WATER_LEVEL_PIN, INPUT);

  // MPU6050 Setup
  Wire.begin();
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
  } else {
    Serial.println("MPU6050 connected successfully.");
  }
}

void loop() {
  // Read DHT11
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

  // Read Water Level Sensor
  int waterLevel = analogRead(WATER_LEVEL_PIN);
  Serial.print("Water Level (Analog Value): ");
  Serial.println(waterLevel);

  // Read MPU6050
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  Serial.print("Accel X: "); Serial.print(ax);
  Serial.print(" | Accel Y: "); Serial.print(ay);
  Serial.print(" | Accel Z: "); Serial.print(az);
  Serial.println();
  Serial.print("Gyro X: "); Serial.print(gx);
  Serial.print(" | Gyro Y: "); Serial.print(gy);
  Serial.print(" | Gyro Z: "); Serial.println(gz);

  Serial.println("------------------------------------");

  delay(2000);
}
```

## Output

```yaml
Humidity: 53.00%  Temperature: 25.20°C
Water Level (Analog Value): 804
Accel X: -2036 | Accel Y: 108 | Accel Z: 15900
Gyro X: 41 | Gyro Y: -22 | Gyro Z: 18
------------------------------------
```

## Getting Started

- Open Arduino IDE.
- Install required libraries:
- Adafruit DHT Sensor Library
- Adafruit Unified Sensor Library
- MPU6050 by Electronic Cats or similar
- Connect the sensors as per the wiring table above.
- Upload the code to the ESP32 board.
- Open the Serial Monitor at 115200 baud rate to view output.

## Reference

- Adafruit DHT Sensor Library
- Adafruit Unified Sensor Library
- MPU6050 Library
- ESP32 I2C Reference

## Notes

- Make sure I2C SDA and SCL pins are correctly connected.
- You can use I2C Scanner to confirm MPU6050 is detected.
- Water level readings vary based on the sensor and liquid type.

## License

This project is licensed under the MIT License.

## Clone the Repo

```bash
git clone https://github.com/AbisheckD/Projects_at_PSG.git

```
