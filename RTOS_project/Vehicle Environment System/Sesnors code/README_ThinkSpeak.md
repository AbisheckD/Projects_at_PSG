# ESP32 ThingSpeak Data Logger

This project collects environmental and motion data from multiple sensors using an **ESP32**, then uploads the data to **ThingSpeak** for real-time monitoring and cloud-based analysis. It uses **FreeRTOS tasks** for concurrent sensor readings and publishes the data every 20 seconds to an 8-field ThingSpeak channel.

## ThingSpeak Overview

[ThingSpeak](https://thingspeak.com/) is an IoT analytics platform that allows you to collect, visualize, and analyze live sensor data. It is particularly useful for projects involving remote monitoring and data logging.

## Features

- Sends sensor data to ThingSpeak:
  - **Date and Time** (from DS3231)
  - **Temperature** and **Humidity** (DHT11)
  - **Water Level** (Analog sensor)
  - **Pressure** and **Altitude** (BMP180)
  - **Acceleration (X, Y, Z)** (MPU6050)
  - **Gyroscope (X, Y, Z)** (MPU6050)
- Sends data every 20 seconds
- Uses `WiFiClient` and `ThingSpeak` libraries
- Visualizes data in ThingSpeak dashboard graphs

## Channel Configuration

1. Go to [ThingSpeak](https://thingspeak.com/) and sign in.
2. Click **"New Channel"** and enable these 8 fields:
   - Field 1: Date and Time
   - Field 2: DHT11 Temperature (°C)
   - Field 3: DHT11 Humidity (%)
   - Field 4: Water Level (Analog Value)
   - Field 5: Pressure (Pa)
   - Field 6: Altitude (m)
   - Field 7: Acceleration (X,Y,Z)
   - Field 8: Gyroscope (X,Y,Z)

3. Click **Save Channel**.

## API Credentials

1. In your ThingSpeak channel, go to the **API Keys** tab.
2. Copy the **Write API Key**.
3. Copy the **Channel ID** from the channel's main page.

## Update the Arduino Code

Update the following lines in your Arduino code:

```c
const char *ssid = "your_wifi_ssid";
const char *password = "your_wifi_password";
unsigned long myChannelNumber = YOUR_CHANNEL_ID;
const char *myWriteAPIKey = "YOUR_WRITE_API_KEY";
```

```c
const char *ssid = "hello_buddy";
const char *password = "Abisheck003";
unsigned long myChannelNumber = 2937230;
const char *myWriteAPIKey = "YWTYTM209FQT5V5I";
```

```c
ThingSpeak.setField(1, currentDateTime);
ThingSpeak.setField(2, dhtTemperature);
ThingSpeak.setField(3, dhtHumidity);
ThingSpeak.setField(4, waterLevel);
ThingSpeak.setField(5, pressure);
ThingSpeak.setField(6, altitude);
ThingSpeak.setField(7, String(ax) + "," + String(ay) + "," + String(az));
ThingSpeak.setField(8, String(gx) + "," + String(gy) + "," + String(gz));

ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
```
> ✅ **Note:**  
> The data is sent every 20 seconds to comply with ThingSpeak’s free tier minimum update rate of 15 seconds.

## Visualizing Data

- Go to your ThingSpeak channel.
- Click Private View or Public View.
- Graphs for each field will update in real-time.
- Customize charts using MATLAB Visualizations (optional).

## Libraries Used

- WiFi.h – ESP32 WiFi support
- ThingSpeak.h – ThingSpeak client for Arduino
- Wire.h – I2C communication
- DHT.h, MPU6050.h, Adafruit_BMP085_U.h, RTClib.h – Sensor libraries

## License

This project is licensed under the MIT License.

## References

- ThingSpeak Arduino Library
- ThingSpeak Docs
- ESP32 FreeRTOS
