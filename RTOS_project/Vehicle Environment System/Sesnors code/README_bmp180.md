# Vehicle Environment System – DHT11 + Water Level Sensor + MPU6050 + BMP180

This project reads **temperature** and **humidity** using the **DHT11 sensor**, measures **water level** using an analog sensor, captures **acceleration** and **gyroscope data** using the **MPU6050**, and reads **atmospheric pressure and temperature** using the **BMP180** sensor. All sensor data is printed to the serial monitor every 2 seconds. This is part of the `RTOS_project` in the `Projects_at_PSG` repository.

## GitHub Repository

- [Main Repository](https://github.com/AbisheckD/Projects_at_PSG/tree/main)
- [Project Folder](https://github.com/AbisheckD/Projects_at_PSG/tree/main/RTOS_project/Vehicle%20Environment%20System)

## Hardware Required

- ESP32 development board
- DHT11 Temperature and Humidity Sensor
- Analog Water Level Sensor
- MPU6050 6-axis Accelerometer + Gyroscope
- BMP180 Barometric Pressure Sensor
- 4.7kΩ or 10kΩ pull-up resistors (for SDA and SCL lines)
- Jumper wires
- USB Cable

## Software Required

- Arduino IDE
- Adafruit DHT Sensor Library
- Adafruit Unified Sensor Library
- MPU6050 Library by Electronic Cats or equivalent
- Adafruit BMP085/BMP180 Unified Library
- Wire Library (included by default)

## Features

- Reads humidity and temperature from DHT11
- Reads analog water level
- Reads accelerometer and gyroscope data from MPU6050
- Reads atmospheric pressure and temperature from BMP180
- Serial output every 2 seconds
- Connection check for I2C devices

## Arduino Wiring

| Sensor            | ESP32 Pin     |
|-------------------|---------------|
| DHT11 VCC         | 5V            |
| DHT11 GND         | GND           |
| DHT11 DATA        | D4            |
| Water Sensor VCC  | 3.3V          |
| Water Sensor GND  | GND           |
| Water Sensor Signal | GPIO34      |
| MPU6050 VCC       | 3.3V          |
| MPU6050 GND       | GND           |
| MPU6050 SDA       | GPIO21        |
| MPU6050 SCL       | GPIO22        |
| BMP180 VCC        | 3.3V          |
| BMP180 GND        | GND           |
| BMP180 SDA        | GPIO21        |
| BMP180 SCL        | GPIO22        |

> ✅ **I2C Pull-up Notes**  
> Connect **4.7kΩ or 10kΩ resistors** between **SDA (GPIO21)** and **3.3V**, and between **SCL (GPIO22)** and **3.3V**. This improves I2C signal stability, especially when using multiple sensors on the same bus.

## Code

```cpp
#include "DHT.h"
#include <Wire.h>
#include <MPU6050.h>
#include <Adafruit_BMP085.h>

#define DHTPIN 4
#define DHTTYPE DHT11
#define WATER_LEVEL_PIN 34

DHT dht(DHTPIN, DHTTYPE);
MPU6050 mpu;
Adafruit_BMP085 bmp;

void setup() {
  Serial.begin(115200);

  dht.begin();
  pinMode(WATER_LEVEL_PIN, INPUT);

  Wire.begin();

  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
  } else {
    Serial.println("MPU6050 connected successfully.");
  }

  if (!bmp.begin()) {
    Serial.println("BMP180 connection failed!");
  } else {
    Serial.println("BMP180 connected successfully.");
  }
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

  int waterLevel = analogRead(WATER_LEVEL_PIN);
  Serial.print("Water Level (Analog Value): ");
  Serial.println(waterLevel);

  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  Serial.print("Accel X: "); Serial.print(ax);
  Serial.print(" | Accel Y: "); Serial.print(ay);
  Serial.print(" | Accel Z: "); Serial.println(az);

  Serial.print("Gyro X: "); Serial.print(gx);
  Serial.print(" | Gyro Y: "); Serial.print(gy);
  Serial.print(" | Gyro Z: "); Serial.println(gz);

  if (bmp.begin()) {
    Serial.print("BMP180 Temp: ");
    Serial.print(bmp.readTemperature());
    Serial.println(" °C");

    Serial.print("BMP180 Pressure: ");
    Serial.print(bmp.readPressure());
    Serial.println(" Pa");
  }

  Serial.println("------------------------------------");
  delay(2000);
}
```

## Output

```yaml
Humidity: 52.00%  Temperature: 25.00°C
Water Level (Analog Value): 801
Accel X: -2010 | Accel Y: 120 | Accel Z: 15890
Gyro X: 42 | Gyro Y: -21 | Gyro Z: 19
BMP180 Temp: 24.50 °C
BMP180 Pressure: 100867 Pa
------------------------------------
```

## Getting Started

- Open Arduino IDE.
- Install required libraries:
- Adafruit DHT Sensor Library
- Adafruit Unified Sensor Library
- MPU6050 by Electronic Cats or equivalent
- Adafruit BMP085/BMP180 Unified Library
- Connect the sensors as per the wiring table.
- Ensure pull-up resistors are added to SDA and SCL lines.
- Upload the code to the ESP32 board.
- Open the Serial Monitor at 115200 baud rate.

## Reference

- Adafruit DHT Sensor Library
- Adafruit Unified Sensor Library
- MPU6050 Library by Electronic Cats
- Adafruit BMP180 Library
- ESP32 I2C Hardware Reference

## Notes

- Use an I2C scanner sketch to verify all I2C devices are detected.
- Pull-up resistors on SDA and SCL are essential when multiple I2C devices are connected.
- BMP180 and MPU6050 share the I2C bus successfully with pull-ups.

## License

This project is licensed under the MIT License.

## Clone the Repo

```bash
git clone https://github.com/AbisheckD/Projects_at_PSG.git
```
