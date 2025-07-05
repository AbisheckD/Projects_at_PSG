#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <MPU6050.h>
#include <Adafruit_BMP085_U.h>
#include <RTClib.h>
#include <WiFi.h>
#include <ThingSpeak.h>

#define DHTPIN 4
#define DHTTYPE DHT11
#define WATER_LEVEL_PIN 34

const char *ssid = "hello_buddy";      
const char *password = "Abisheck003"; 
unsigned long myChannelNumber = 2937230;  
const char *myWriteAPIKey = "YWTYTM209FQT5V5I";  

DHT dht(DHTPIN, DHTTYPE);
MPU6050 mpu;
Adafruit_BMP085_Unified bmp;
RTC_DS3231 rtc;

TaskHandle_t DHTTaskHandle;
TaskHandle_t WaterLevelTaskHandle;
TaskHandle_t MPU6050TaskHandle;
TaskHandle_t BMP180TaskHandle;
TaskHandle_t RTCReadTaskHandle;

float bmpTemperature, pressure, altitude;
String currentDateTime;
int waterLevel;
float dhtTemperature, dhtHumidity;
int16_t ax, ay, az, gx, gy, gz;

WiFiClient client;  

void DHT11Task(void *parameter) {
  while (true) {
    dhtHumidity = dht.readHumidity();
    dhtTemperature = dht.readTemperature();
    if (isnan(dhtHumidity) || isnan(dhtTemperature)) {
      Serial.println("Failed to read from DHT sensor");
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void WaterLevelTask(void *parameter) {
  while (true) {
    waterLevel = analogRead(WATER_LEVEL_PIN);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void MPU6050Task(void *parameter) {
  while (true) {
    mpu.getAcceleration(&ax, &ay, &az);
    mpu.getRotation(&gx, &gy, &gz);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void BMP180Task(void *parameter) {
  while (true) {
    bmp.getPressure(&pressure);       // pass address
    bmp.getTemperature(&bmpTemperature); // pass address
    altitude = bmp.pressureToAltitude(1013.25, pressure);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void RTCReadTask(void *parameter) {
  while (true) {
    DateTime now = rtc.now();
    currentDateTime = String(now.day()) + "/" + String(now.month()) + "/" + String(now.year()) + " " +
                      String(now.hour()) + ":" + String(now.minute()) + ":" + String(now.second());
    vTaskDelay(1000 / portTICK_PERIOD_MS);
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

  rtc.begin();

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  ThingSpeak.begin(client);  // <== Fixed

  xTaskCreatePinnedToCore(DHT11Task, "DHT11Task", 1024*4, NULL, 1, &DHTTaskHandle, 0);
  xTaskCreatePinnedToCore(WaterLevelTask, "WaterLevelTask", 1024*4, NULL, 1, &WaterLevelTaskHandle, 1);
  xTaskCreatePinnedToCore(MPU6050Task, "MPU6050Task", 1024*4, NULL, 1, &MPU6050TaskHandle, 0);
  xTaskCreatePinnedToCore(BMP180Task, "BMP180Task", 1024*4, NULL, 1, &BMP180TaskHandle, 0);
  xTaskCreatePinnedToCore(RTCReadTask, "RTCReadTask", 1024*4, NULL, 1, &RTCReadTaskHandle, 0);
}

void loop() {
  ThingSpeak.setField(1, currentDateTime);
  ThingSpeak.setField(2, dhtTemperature);
  ThingSpeak.setField(3, dhtHumidity);
  ThingSpeak.setField(4, waterLevel);
  ThingSpeak.setField(5, pressure);
  ThingSpeak.setField(6, altitude);
  ThingSpeak.setField(7, String(ax) + "," + String(ay) + "," + String(az));
  ThingSpeak.setField(8, String(gx) + "," + String(gy) + "," + String(gz));

  ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);

  Serial.println("Data sent to ThingSpeak");
  delay(20000);
}
