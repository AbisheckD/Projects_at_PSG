#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <MPU6050.h>
#include <Adafruit_BMP085_U.h>
#include <WiFi.h>
#include <ThingSpeak.h>
#include <SPI.h>
#include <SD.h>
#include <time.h>

#define DHTPIN 4
#define DHTTYPE DHT11
#define WATER_LEVEL_PIN 36
#define SD_CS 5

const char *ssid = "hellobuddy";
const char *password = "abisheck003";
unsigned long myChannelNumber = 2937230;
const char *myWriteAPIKey = "YWTYTM209FQT5V5I";

const char *ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 19800; // GMT+5:30
const int daylightOffset_sec = 0;

DHT dht(DHTPIN, DHTTYPE);
MPU6050 mpu;
Adafruit_BMP085_Unified bmp;

WiFiClient client;

TaskHandle_t DHTTaskHandle, WaterLevelTaskHandle, MPU6050TaskHandle;
TaskHandle_t BMP180TaskHandle, SDCardLoggerTaskHandle, ThingSpeakTaskHandle;

float bmpTemperature, pressure, altitude;
float dhtTemperature, dhtHumidity;
int16_t ax, ay, az, gx, gy, gz;
int waterLevel;

SemaphoreHandle_t printMutex;
SemaphoreHandle_t resourceMutex;

String getCurrentTime() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) return "N/A";
  char timeStr[30];
  strftime(timeStr, sizeof(timeStr), "%Y-%m-%d %H:%M:%S", &timeinfo);
  return String(timeStr);
}

void DHT11Task(void *parameter) {
  while (true) {
    dhtHumidity = dht.readHumidity();
    dhtTemperature = dht.readTemperature();
    if (!isnan(dhtHumidity) && !isnan(dhtTemperature)) {
      xSemaphoreTake(printMutex, portMAX_DELAY);
      Serial.println("DHT11 Sensor:");
      Serial.print("Humidity: "); Serial.print(dhtHumidity); Serial.print("%");
      Serial.print(" | Temp: "); Serial.print(dhtTemperature); Serial.println("°C");
      xSemaphoreGive(printMutex);
    }
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}

void WaterLevelTask(void *parameter) {
  while (true) {
    waterLevel = analogRead(WATER_LEVEL_PIN);
    xSemaphoreTake(printMutex, portMAX_DELAY);
    Serial.print("Water Level: "); Serial.print(waterLevel); Serial.println("%");
    xSemaphoreGive(printMutex);
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}

void MPU6050Task(void *parameter) {
  while (true) {
    mpu.getAcceleration(&ax, &ay, &az);
    mpu.getRotation(&gx, &gy, &gz);
    xSemaphoreTake(printMutex, portMAX_DELAY);
    Serial.println("MPU6050 Sensor:");
    Serial.print("Ax: "); Serial.print(ax / 9.81); Serial.print(" m/s²");
    Serial.print(" | Ay: "); Serial.print(ay / 9.81); Serial.print(" m/s²");
    Serial.print(" | Az: "); Serial.print(az / 9.81); Serial.print(" m/s²");
    Serial.print(" | Gx: "); Serial.print(gx); Serial.print("°/s");
    Serial.print(" | Gy: "); Serial.print(gy); Serial.print("°/s");
    Serial.print(" | Gz: "); Serial.print(gz); Serial.println("°/s");
    xSemaphoreGive(printMutex);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

void BMP180Task(void *parameter) {
  const float knownAltitude = 427.0;
  float seaLevelPressure;

  while (true) {
    bmp.getPressure(&pressure);
    bmp.getTemperature(&bmpTemperature);
    float pressure_hPa = pressure / 100.0;
    seaLevelPressure = bmp.seaLevelForAltitude(knownAltitude, pressure_hPa);
    altitude = bmp.pressureToAltitude(seaLevelPressure, pressure_hPa);

    xSemaphoreTake(printMutex, portMAX_DELAY);
    Serial.println("BMP180 Sensor:");
    Serial.print("BMP_Temperature: "); Serial.print(bmpTemperature); Serial.print("°C");
    Serial.print(" | Pressure: "); Serial.print(pressure_hPa); Serial.print(" hPa");
    Serial.print(" | Altitude: "); Serial.print(altitude); Serial.println(" m");
    xSemaphoreGive(printMutex);

    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

void ThingSpeakTask(void *parameter) {
  while (true) {
    xSemaphoreTake(resourceMutex, portMAX_DELAY); // protect ThingSpeak client
    ThingSpeak.setField(1, dhtTemperature);
    ThingSpeak.setField(2, dhtHumidity);
    ThingSpeak.setField(3, waterLevel);
    ThingSpeak.setField(4, pressure);
    ThingSpeak.setField(5, altitude);
    ThingSpeak.setField(6, (float)(ax / 9.81));
    ThingSpeak.setField(7, (float)(ay / 9.81));
    ThingSpeak.setField(8, (float)(az / 9.81));

    int code = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
    xSemaphoreGive(resourceMutex);

    xSemaphoreTake(printMutex, portMAX_DELAY);
    if (code == 200) {
      Serial.println("✅ ThingSpeak update successful");
    } else {
      Serial.printf("❌ ThingSpeak update failed (code: %d)\n", code);
    }
    xSemaphoreGive(printMutex);

    vTaskDelay(30000 / portTICK_PERIOD_MS);
  }
}

void SDCardLoggerTask(void *parameter) {
  while (true) {
    String now = getCurrentTime();

    xSemaphoreTake(resourceMutex, portMAX_DELAY); // protect SD access
    File file = SD.open("/log.csv", FILE_APPEND);
    if (file) {
      file.printf("%s,%.2f,%.2f,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
                  now.c_str(), dhtTemperature, dhtHumidity, waterLevel,
                  pressure, altitude, bmpTemperature,
                  ax / 9.81, ay / 9.81, az / 9.81, gx, gy, gz);
      file.close();
      xSemaphoreTake(printMutex, portMAX_DELAY);
      Serial.println("✅ Data logged to SD card");
      xSemaphoreGive(printMutex);
    } else {
      xSemaphoreTake(printMutex, portMAX_DELAY);
      Serial.println("❌ Failed to open log.csv");
      xSemaphoreGive(printMutex);
    }
    xSemaphoreGive(resourceMutex);
    vTaskDelay(15000 / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(115200);

  dht.begin();
  Wire.begin();
  mpu.initialize();

  printMutex = xSemaphoreCreateMutex();
  resourceMutex = xSemaphoreCreateMutex();

  if (!bmp.begin()) {
    Serial.println("BMP180 not detected. Check wiring!");
    while (1) delay(1000);
  }

  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("Waiting for connection...");

  int retryCount = 0;
  while (WiFi.status() != WL_CONNECTED && retryCount < 20) {
    delay(500);
    Serial.print(".");
    retryCount++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✅ Connected to WiFi!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\n❌ Failed to connect to WiFi");
  }

  ThingSpeak.begin(client);
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  Serial.println("Initializing SD card...");
  if (!SD.begin(SD_CS)) {
    Serial.println("Card Mount Failed");
    return;
  }

  uint8_t cardType = SD.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    return;
  }

  Serial.println("✅ SD card initialized.");
  File file = SD.open("/log.csv", FILE_WRITE);
  if (file) {
    file.println("Timestamp,DHT_Temperature(°C),DHT_Humidity(%),Water_Level(%),Pressure(hPa),Altitude(m),BMP_Temperature(°C),Ax(m/s²),Ay(m/s²),Az(m/s²),Gx(°/s),Gy(°/s),Gz(°/s)");
    file.close();
    Serial.println("Test file written.");
  } else {
    Serial.println("Failed to write header to file.");
  }

  xTaskCreatePinnedToCore(DHT11Task, "DHT11", 4096, NULL, 1, &DHTTaskHandle, 0);
  xTaskCreatePinnedToCore(WaterLevelTask, "WaterLevel", 4096, NULL, 1, &WaterLevelTaskHandle, 0);
  xTaskCreatePinnedToCore(MPU6050Task, "MPU6050", 4096, NULL, 1, &MPU6050TaskHandle, 0);
  xTaskCreatePinnedToCore(BMP180Task, "BMP180", 4096, NULL, 1, &BMP180TaskHandle, 0);
  xTaskCreatePinnedToCore(SDCardLoggerTask, "SDLogger", 4096, NULL, 1, &SDCardLoggerTaskHandle, 1);
  xTaskCreatePinnedToCore(ThingSpeakTask, "ThingSpeak", 4096, NULL, 2, &ThingSpeakTaskHandle, 1);
}

void loop() {
}
