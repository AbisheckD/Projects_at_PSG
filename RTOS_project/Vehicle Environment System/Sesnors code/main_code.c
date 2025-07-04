#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <MPU6050.h>
#include <Adafruit_BMP085_U.h>

#define DHTPIN 4
#define DHTTYPE DHT11
#define WATER_LEVEL_PIN 34

DHT dht(DHTPIN, DHTTYPE);
MPU6050 mpu;
Adafruit_BMP085_Unified bmp;

TaskHandle_t DHTTaskHandle;
TaskHandle_t WaterLevelTaskHandle;
TaskHandle_t MPU6050TaskHandle;
TaskHandle_t BMP180TaskHandle;

void DHT11Task(void *parameter) {
  while (true) {
    float humidity = dht.readHumidity();
    float temperature = dht.readTemperature();
    if (isnan(humidity) || isnan(temperature)) {
      Serial.println("Failed to read from DHT sensor");
    } else {
      Serial.println("DHT11 Sensor: ");
      Serial.print("Humidity: ");
      Serial.print(humidity);
      Serial.print("%  Temperature: ");
      Serial.print(temperature);
      Serial.println("°C");
      Serial.println();
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void WaterLevelTask(void *parameter) {
  while (true) {
    int waterLevel = analogRead(WATER_LEVEL_PIN);
    Serial.print("WATER LEVEL Sensor (Analog Value): ");
    Serial.println(waterLevel);
    Serial.println();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void MPU6050Task(void *parameter) {
  while (true) {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    mpu.getAcceleration(&ax, &ay, &az);
    mpu.getRotation(&gx, &gy, &gz);
    Serial.println("MPU6050 Sensor: ");
    Serial.print("Accel X: "); Serial.print(ax);
    Serial.print(" | Accel Y: "); Serial.print(ay);
    Serial.print(" | Accel Z: "); Serial.println(az);
    Serial.print("Gyro X: "); Serial.print(gx);
    Serial.print(" | Gyro Y: "); Serial.print(gy);
    Serial.print(" | Gyro Z: "); Serial.println(gz);
    Serial.println();

    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void BMP180Task(void *parameter) {
  while (true) {
    float pressure, temperature;
    
    // Get pressure and temperature values
    bmp.getPressure(&pressure);
    bmp.getTemperature(&temperature);
    
    // Calculate altitude using pressure
    float altitude = bmp.pressureToAltitude(1013.25, pressure);  // hPa
    Serial.println("BMP180 Sensor: ");
    Serial.print("Temperature (bmp180): ");
    Serial.println(temperature);
    Serial.print("Pressure: ");
    Serial.println(pressure);
    Serial.print("Altitude: ");
    Serial.println(altitude);
    Serial.println();
    
    vTaskDelay(1000 / portTICK_PERIOD_MS); // Task delay
  }
}

void setup() {
  Serial.begin(115200);

  dht.begin();
  Wire.begin();
  mpu.initialize();
  
  if (!bmp.begin()) {
    Serial.println("BMP180 initialization failed!");
    while (1);
  }

  xTaskCreatePinnedToCore(DHT11Task, "DHT11Task", 1024*4, NULL, 1, &DHTTaskHandle, 0);
  xTaskCreatePinnedToCore(WaterLevelTask, "WaterLevelTask", 1024*4, NULL, 1, &WaterLevelTaskHandle, 1);
  xTaskCreatePinnedToCore(MPU6050Task, "MPU6050Task", 1024*4, NULL, 1, &MPU6050TaskHandle, 0);
  xTaskCreatePinnedToCore(BMP180Task, "BMP180Task", 1024*4, NULL, 1, &BMP180TaskHandle, 0);
}

void loop() {
  // Tasks are handled by FreeRTOS
}
