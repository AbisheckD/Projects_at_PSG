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
