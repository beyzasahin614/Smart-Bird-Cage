# Smart Bird Cage 🐦📡

An IoT-based smart bird cage system using **ESP32**, **DHT11**, **gas sensor**, and **servo motors**. Sensor data is sent securely to **Adafruit IO (MQTTS)**, and servos respond to environmental conditions like high temperature or gas levels.

---

## 🔧 Features

- 🌡️ **Temperature & Humidity Monitoring** via DHT11
- 🧪 **Gas Detection** with analog sensor (e.g., MQ2)
- 🔄 **Servo Motor Control** (ventilation or door flaps)
- ☁️ **Cloud Connectivity** using **Adafruit IO (MQTT over TLS)**
- 🔒 **Secure Data Transmission** with SSL/TLS (port 8883)
- ⏲️ **Automatic Servo Actions**:
  - Moves servo1 to 0° if gas exceeds threshold
  - Moves servo2 to 180° if temperature exceeds 35°C
  - Periodically moves servo2 every 10 seconds (simulating 1 hour)

---

## 📦 Components

- ESP32
- DHT11 sensor
- Gas sensor (connected to A34)
- 2x Servo Motors (GPIO 13 and 14)
- Adafruit IO account (for feeds and API key)
- Wi-Fi connection

---

## 🛠️ Libraries Required

- `WiFi.h`
- `WiFiClientSecure.h`
- `Adafruit_MQTT.h`
- `Adafruit_MQTT_Client.h`
- `ESP32Servo.h`
- `DHT.h`

---

## 🧠 MQTT Feeds

**Subscriptions:**
- `servo1` (controls servo1 angle)
- `servo2` (controls servo2 angle)

**Publications:**
- `temperature`, `humidity`, `GAZ` (sensor values)
- `servo1`, `servo2` (current angle states)

---

## 🔐 Security

MQTT uses port **8883** (TLS) with **Adafruit IO Root CA** certificate embedded in the firmware for secure communication.

---

## 🔄 Logic Summary

- Every **3 seconds**:
  - Reads humidity, temperature, gas levels
  - Publishes data to Adafruit IO
  - Checks gas and temperature thresholds

- **Gas > threshold**: Sets `servo1` to `0°`

- **Temperature > 35°C**: Sets `servo2` to `180°` for 1 second, then returns to `0°`

- **Every 10 seconds** (simulating 1 hour): Toggles `servo2` 180° → 0°

---

## 📡 Example Dashboard

You can visualize real-time data and control servo angles via [Adafruit IO dashboard](https://io.adafruit.com/).

---



> This project demonstrates a practical smart system combining environmental monitoring, automation, and secure cloud integration using ESP32 and Adafruit IO.
