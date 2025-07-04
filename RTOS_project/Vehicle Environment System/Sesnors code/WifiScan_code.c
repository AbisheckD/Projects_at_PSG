#include <WiFi.h>

void setup() {
  Serial.begin(115200);
  Serial.println("Scanning WiFi...");

  int networks = WiFi.scanNetworks();
  if (networks == 0) {
    Serial.println("No networks found.");
  } else {
    Serial.println("Networks found:");
    for (int i = 0; i < networks; ++i) {
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(WiFi.SSID(i));
      Serial.print(" (");
      Serial.print(WiFi.RSSI(i));
      Serial.println(" dBm)");
    }
  }
}
void loop() {}
