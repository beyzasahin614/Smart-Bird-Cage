#include <ESP32Servo.h>
#include <WiFi.h>
#include "WiFiClientSecure.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include "DHT.h" // DHT sensör kütüphanesi

// Define servos and the pins they are connected to
Servo servo1;
Servo servo2;
const int servo1Pin = 13;
const int servo2Pin = 14;
const int defaultServoAngle = 90;
int servo1Angle = defaultServoAngle;
int servo2Angle = defaultServoAngle;

// Define the minimum and maximum pulse widths for the servos
const int minPulseWidth = 500; // 0.5 ms
const int maxPulseWidth = 2500; // 2.5 ms

// DHT sensör tanımı
#define DHTPIN 4     // DHT veri pini
#define DHTTYPE DHT11   // DHT11 için DHT11 kullanın
DHT dht(DHTPIN, DHTTYPE);

// Gaz sensör tanımı
const int gasSensorPin = 34; // Analog pin
const int gasThreshold = 300; // Gaz seviyesi eşiği

/********* WiFi Access Point ***********/

#define WLAN_SSID //"wifi name"
#define WLAN_PASS //"wifi password"

/********* Adafruit.io Setup ***********/

#define AIO_SERVER "io.adafruit.com"
#define AIO_SERVERPORT 8883 // Using port 8883 for MQTTS
#define AIO_USERNAME "Jeqo"
#define AIO_KEY "aio_cThN664Owro2k3PditNCzqmfUcSe"

// WiFiClientSecure for SSL/TLS support
WiFiClientSecure client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

// io.adafruit.com root CA
const char* adafruitio_root_ca = \
      "-----BEGIN CERTIFICATE-----\n"
      "MIIEjTCCA3WgAwIBAgIQDQd4KhM/xvmlcpbhMf/ReTANBgkqhkiG9w0BAQsFADBh\n"
      "MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3\n"
      "d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBH\n"
      "MjAeFw0xNzExMDIxMjIzMzdaFw0yNzExMDIxMjIzMzdaMGAxCzAJBgNVBAYTAlVT\n"
      "MRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxGTAXBgNVBAsTEHd3dy5kaWdpY2VydC5j\n"
      "b20xHzAdBgNVBAMTFkdlb1RydXN0IFRMUyBSU0EgQ0EgRzEwggEiMA0GCSqGSIb3\n"
      "DQEBAQUAA4IBDwAwggEKAoIBAQC+F+jsvikKy/65LWEx/TMkCDIuWegh1Ngwvm4Q\n"
      "yISgP7oU5d79eoySG3vOhC3w/3jEMuipoH1fBtp7m0tTpsYbAhch4XA7rfuD6whU\n"
      "gajeErLVxoiWMPkC/DnUvbgi74BJmdBiuGHQSd7LwsuXpTEGG9fYXcbTVN5SATYq\n"
      "DfbexbYxTMwVJWoVb6lrBEgM3gBBqiiAiy800xu1Nq07JdCIQkBsNpFtZbIZhsDS\n"
      "fzlGWP4wEmBQ3O67c+ZXkFr2DcrXBEtHam80Gp2SNhou2U5U7UesDL/xgLK6/0d7\n"
      "6TnEVMSUVJkZ8VeZr+IUIlvoLrtjLbqugb0T3OYXW+CQU0kBAgMBAAGjggFAMIIB\n"
      "PDAdBgNVHQ4EFgQUlE/UXYvkpOKmgP792PkA76O+AlcwHwYDVR0jBBgwFoAUTiJU\n"
      "IBiV5uNu5g/6+rkS7QYXjzkwDgYDVR0PAQH/BAQDAgGGMB0GA1UdJQQWMBQGCCsG\n"
      "AQUFBwMBBggrBgEFBQcDAjASBgNVHRMBAf8ECDAGAQH/AgEAMDQGCCsGAQUFBwEB\n"
      "BCgwJjAkBggrBgEFBQcwAYYYaHR0cDovL29jc3AuZGlnaWNlcnQuY29tMEIGA1Ud\n"
      "HwQ7MDkwN6A1oDOGMWh0dHA6Ly9jcmwzLmRpZ2ljZXJ0LmNvbS9EaWdpQ2VydEds\n"
      "b2JhbFJvb3RHMi5jcmwwPQYDVR0gBDYwNDAyBgRVHSAAMCowKAYIKwYBBQUHAgEW\n"
      "HGh0dHBzOi8vd3d3LmRpZ2ljZXJ0LmNvbS9DUFMwDQYJKoZIhvcNAQELBQADggEB\n"
      "AIIcBDqC6cWpyGUSXAjjAcYwsK4iiGF7KweG97i1RJz1kwZhRoo6orU1JtBYnjzB\n"
      "c4+/sXmnHJk3mlPyL1xuIAt9sMeC7+vreRIF5wFBC0MCN5sbHwhNN1JzKbifNeP5\n"
      "ozpZdQFmkCo+neBiKR6HqIA+LMTMCMMuv2khGGuPHmtDze4GmEGZtYLyF8EQpa5Y\n"
      "jPuV6k2Cr/N3XxFpT3hRpt/3usU/Zb9wfKPtWpoznZ4/44c1p9rzFcZYrWkj3A+7\n"
      "TNBJE0GmP2fhXhP1D/XVfIW/h0yCJGEiV9Glm/uGOa3DXHlmbAcxSyCRraG+ZBkA\n"
      "7h4SeM6Y8l/7MBRpPCz6l8Y=\n"
      "-----END CERTIFICATE-----\n";

/********** Feeds *************/

// Setup feeds for two servos
Adafruit_MQTT_Subscribe servo1Feed = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/servo1");
Adafruit_MQTT_Subscribe servo2Feed = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/servo2");
Adafruit_MQTT_Publish temperatureFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/temperature");
Adafruit_MQTT_Publish humidityFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/humidity");
Adafruit_MQTT_Publish gasFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/GAZ");
Adafruit_MQTT_Publish servo1Publish = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/servo1");
Adafruit_MQTT_Publish servo2Publish = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/servo2");

/********* Sketch Code ************/

unsigned long lastCheckTime = 0;
unsigned long lastServoMoveTime = 0;
unsigned long lastPeriodicServoMoveTime = 0;
const unsigned long checkInterval = 3000; // 3 saniye
const unsigned long servoMoveDuration = 1000; // 1 saniye
const unsigned long periodicServoMoveInterval = 10000; // 10 saniye/ normalde 1 saat olacak 3.600.000 milisaniye
bool servoMoving = false;
bool periodicServoMoving = false;

void setup() {
  Serial.begin(115200);
  delay(10);

  Serial.println(F("Adafruit IO MQTTS (SSL/TLS) Example"));

  // Attach the servos to the specified pins and set their pulse width range
  servo1.attach(servo1Pin, minPulseWidth, maxPulseWidth);
  servo2.attach(servo2Pin, minPulseWidth, maxPulseWidth);

  // Set the PWM frequency for the servos
  servo1.setPeriodHertz(50); // Standard 50Hz servo
  servo2.setPeriodHertz(50); // Standard 50Hz servo

  // Connect to WiFi access point
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WLAN_SSID);

  delay(1000);

  WiFi.begin(WLAN_SSID, WLAN_PASS);
  delay(2000);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  // Set Adafruit IO's root CA
  client.setCACert(adafruitio_root_ca);

  // Setup MQTT subscription for feeds
  mqtt.subscribe(&servo1Feed);
  mqtt.subscribe(&servo2Feed);

  // DHT sensör başlatma
  dht.begin();
}

void loop() {
  // Ensure the connection to the MQTT server is alive (this will make the first
  // connection and automatically reconnect when disconnected).
  MQTT_connect();

  // Process incoming messages
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(10000))) {
    if (subscription == &servo1Feed) {
      int newAngle = atoi((char *)servo1Feed.lastread);
      Serial.print(F("Received servo1 angle: "));
      Serial.println(newAngle);
      servo1.write(newAngle);
      servo1Publish.publish(newAngle); // Publish servo1 angle
    }
    if (subscription == &servo2Feed) {
      int newAngle = atoi((char *)servo2Feed.lastread);
      Serial.print(F("Received servo2 angle: "));
      Serial.println(newAngle);
      servo2.write(newAngle);
      servo2Publish.publish(newAngle); // Publish servo2 angle
    }
  }

  unsigned long currentMillis = millis();

  // Check sensors every 3 seconds
  if (currentMillis - lastCheckTime >= checkInterval) {
    lastCheckTime = currentMillis;

    // DHT sensörden veri okuma
    float humidity = dht.readHumidity();
    float temperature = dht.readTemperature();

    if (isnan(humidity) || isnan(temperature)) {
      Serial.println(F("Failed to read from DHT sensor!"));
      return;
    }

    Serial.print(F("Humidity: "));
    Serial.print(humidity);
    Serial.print(F("%  Temperature: "));
    Serial.print(temperature);
    Serial.println(F("°C"));

    // Gaz sensöründen veri okuma
    int gasLevel = analogRead(gasSensorPin);
    Serial.print(F("Gas level: "));
    Serial.println(gasLevel);

    // Gaz seviyesi eşiğini kontrol et
    if (gasLevel > gasThreshold) {
      Serial.println(F("Gas level exceeded threshold! Setting servo1 to 0 degrees."));
      servo1.write(0);
      servo1Publish.publish(0); // Publish servo1 angle change
    }

    // Sıcaklık 35°C'yi geçerse servo2'yi hareket ettir
    if (temperature > 35) {
      Serial.println(F("Temperature exceeded 35°C! Moving servo2 to 180 degrees."));
      servo2.write(180);
      servoMoving = true;
      lastServoMoveTime = currentMillis;
    }

    // Publish readings to Adafruit IO
    if (!temperatureFeed.publish(temperature)) {
      Serial.println(F("Failed to publish temperature"));
    }

    if (!humidityFeed.publish(humidity)) {
      Serial.println(F("Failed to publish humidity"));
    }

    if (!gasFeed.publish(gasLevel)) {
      Serial.println(F("Failed to publish gas level"));
    }
  }

  // Servo2'yi 1 saniye boyunca 180 dereceye döndür ve sonra geri döndür (sıcaklık nedeniyle)
  if (servoMoving && currentMillis - lastServoMoveTime >= servoMoveDuration) {
    servo2.write(0);
    servoMoving = false;
    Serial.println(F("Servo2 moved back to 0 degrees after temperature exceeded 35°C."));
  }

  // Servo2'yi her 5 saniyede bir hareket ettir
  if (!periodicServoMoving && currentMillis - lastPeriodicServoMoveTime >= periodicServoMoveInterval) {
    periodicServoMoving = true;
    lastPeriodicServoMoveTime = currentMillis;
    servo2.write(180);
    Serial.println(F("Servo2 moved to 180 degrees (periodic)."));
  }

  if (periodicServoMoving && currentMillis - lastPeriodicServoMoveTime >= servoMoveDuration) {
    servo2.write(0);
    periodicServoMoving = false;
    Serial.println(F("Servo2 moved back to 0 degrees (periodic)."));
  }

  delay(200); // 200ms bekle
}

// Function to ensure the connection to the MQTT server is alive
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for success
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();
    delay(5000);  // wait 5 seconds
  }
  Serial.println("MQTT Connected!");
}
