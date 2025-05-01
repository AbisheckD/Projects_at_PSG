#include "DHT.h"

#define DHTPIN 4
#define DHTTYPE DHT11
#define WATER_LEVEL_PIN 34

DHT dht(DHTPIN, DHTTYPE);

void setup() {
  Serial.begin(115200);
  dht.begin();
  pinMode(WATER_LEVEL_PIN, INPUT);
}

void loop() {
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();
  int waterLevel = analogRead(WATER_LEVEL_PIN);

  if (isnan(humidity) || isnan(temperature)) {
    Serial.println("Failed to read from DHT sensor!");
  } else {
    Serial.print("Humidity: ");
    Serial.print(humidity);
    Serial.print("%  Temperature: ");
    Serial.print(temperature);
    Serial.println("°C");
  }

  Serial.print("Water Level (Analog Value): ");
  Serial.println(waterLevel);

  delay(2000);
}
