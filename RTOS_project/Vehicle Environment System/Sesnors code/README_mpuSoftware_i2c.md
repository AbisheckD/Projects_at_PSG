# MPU6050 Sensor Interface using Software I2C (SoftwareWire)

This project demonstrates how to interface the **MPU6050 Accelerometer and Gyroscope** sensor using **Software I2C** via the `SoftwareWire` library. It enables communication over custom-defined GPIO pins, useful when hardware I2C lines are occupied or unavailable.

## Features

- Communicates with MPU6050 using **bit-banged I2C**
- Reads **acceleration (X, Y, Z)** and **gyroscope (X, Y, Z)** values
- Prints data to the **Serial Monitor** every second
- Allows usage of **custom SDA and SCL pins** (GPIO26 and GPIO27 by default)

## Hardware Required

- ESP32 / Arduino UNO / Mega / Nano
- MPU6050 sensor module (GY-521 or similar)
- Jumper wires
- Breadboard (optional)

## Wiring

| MPU6050 Pin | ESP32 Pin (Example) |
|-------------|---------------------|
| VCC         | 3.3V                |
| GND         | GND                 |
| SDA         | GPIO26              |
| SCL         | GPIO27              |

> ⚠️ **Note**: Always use 3.3V with ESP32. If you're using an Arduino UNO (5V), ensure the MPU6050 is 5V tolerant or use a logic level shifter.

## Software Required

- Arduino IDE
- **SoftwareWire** library ([Install from GitHub](https://github.com/Testato/SoftwareWire))
- **I2Cdevlib** (for MPU6050 driver) – https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
- Wire Library (built-in)
- MPU6050 breakout module

## Code

```c
#include <SoftwareWire.h>
#include <MPU6050.h>
#include <I2Cdev.h>
#include <Wire.h>

#define MPU_SDA 26
#define MPU_SCL 27

SoftwareWire mpuWire(MPU_SDA, MPU_SCL);
MPU6050 mpu(0x68);  // Default I2C address for MPU6050

void setup() {
  Serial.begin(115200);
  mpuWire.begin();

  I2Cdev::begin(&mpuWire);  // Tell I2Cdev to use SoftwareWire
  mpu.initialize();

  if (mpu.testConnection()) {
    Serial.println("MPU6050 connection successful");
  } else {
    Serial.println("MPU6050 connection failed");
  }
}

void loop() {
  int16_t ax, ay, az, gx, gy, gz;

  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  Serial.print("Accel X: "); Serial.print(ax);
  Serial.print(" | Y: "); Serial.print(ay);
  Serial.print(" | Z: "); Serial.println(az);

  Serial.print("Gyro X: "); Serial.print(gx);
  Serial.print(" | Y: "); Serial.print(gy);
  Serial.print(" | Z: "); Serial.println(gz);

  delay(1000);
}
```

## OUTPUT

``` yash
MPU6050 connection successful
Accel X: -341 | Y: 56 | Z: 15960
Gyro X: 12 | Y: -5 | Z: 3
```
## Use Case

- Ideal for boards with limited I2C peripherals
- Use custom GPIOs for I2C communication
- Helpful for debugging I2C issues or integrating multiple I2C devices

## Notes
- Ensure the SoftwareWire library you use supports your architecture (ESP32 or AVR).
- This setup is generally slower than hardware I2C but offers more flexibility.
- using multiple SoftwareWire instances, assign different pins carefully.

## License

- This project is licensed under the MIT License.

## References

- SoftwareWire Library by Testato
- I2Cdevlib for MPU6050
- MPU6050 Datasheet

