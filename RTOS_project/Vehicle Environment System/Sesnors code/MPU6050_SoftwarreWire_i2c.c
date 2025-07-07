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
