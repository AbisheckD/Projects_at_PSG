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

  // BMP180 Setup
  if (!bmp.begin()) {
    Serial.println("BMP180 initialization failed!");
  } else {
    Serial.println("BMP180 connected successfully.");
  }
}

void loop() {
  // Read DHT11
  float humidity = dht.readHumidity();
  float temperature_dht = dht.readTemperature();

  if (isnan(humidity) || isnan(temperature_dht)) {
    Serial.println("Failed to read from DHT sensor!");
  } else {
    Serial.print("Humidity: ");
    Serial.print(humidity);
    Serial.print("%  Temperature (DHT11): ");
    Serial.print(temperature_dht);
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

  // Read BMP180
  Serial.print("Temperature (BMP180): ");
  Serial.print(bmp.readTemperature());
  Serial.println("°C");

  Serial.print("Pressure: ");
  Serial.print(bmp.readPressure());
  Serial.println(" Pa");

  Serial.print("Altitude: ");
  Serial.print(bmp.readAltitude(1013.25));
  Serial.println(" meters");

  Serial.println("------------------------------------");

  delay(2000);
}
