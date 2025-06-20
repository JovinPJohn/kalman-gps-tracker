#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <TinyGPS++.h>
#include <SimpleKalmanFilter.h>

const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";

const char* FIREBASE_HOST = "YOUR_FIREBASE_HOST";
const char* FIREBASE_SECRET = "YOUR_FIREBASE_SECRET";

TinyGPSPlus gps;
HardwareSerial gpsSerial(1);

SimpleKalmanFilter kalmanLat(2, 2, 0.01);
SimpleKalmanFilter kalmanLng(2, 2, 0.01);

unsigned long previousMillis = 0;
const long interval = 1000;

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(9600, SERIAL_8N1, 16, 17);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
}

void loop() {
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      if (gps.location.isValid()) {
        unsigned long currentMillis = millis();
        if (currentMillis - previousMillis >= interval) {
          previousMillis = currentMillis;

          float rawLat = gps.location.lat();
          float rawLng = gps.location.lng();

          float filteredLat = kalmanLat.updateEstimate(rawLat);
          float filteredLng = kalmanLng.updateEstimate(rawLng);

          sendToFirebase(rawLat, rawLng, filteredLat, filteredLng);

          Serial.printf("Raw: %.6f, %.6f | Filtered: %.6f, %.6f\n", rawLat, rawLng, filteredLat, filteredLng);
        }
      }
    }
  }
}

void sendToFirebase(float rawLat, float rawLng, float filteredLat, float filteredLng) {
  if (WiFi.status() != WL_CONNECTED) return;

  HTTPClient http;
  StaticJsonDocument<384> doc;

  doc["raw"]["lat"] = rawLat;
  doc["raw"]["lng"] = rawLng;
  doc["filtered"]["lat"] = filteredLat;
  doc["filtered"]["lng"] = filteredLng;
  doc["satellites"] = gps.satellites.value();
  doc["hdop"] = gps.hdop.hdop();
  doc["timestamp"] = millis();

  String url = "https://" + String(FIREBASE_HOST) + "/devices/esp32_01.json?auth=" + String(FIREBASE_SECRET);

  http.begin(url);
  http.addHeader("Content-Type", "application/json");

  String jsonData;
  serializeJson(doc, jsonData);

  int httpCode = http.POST(jsonData);

  if (httpCode != HTTP_CODE_OK) {
    Serial.println("Firebase error: " + String(httpCode));
  }

  http.end();
}