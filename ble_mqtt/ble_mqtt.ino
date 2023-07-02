#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <ArduinoJson.h>

const char* ssid = "rede";
const char* password = "senha";

const char* broker = "mqtt.tago.io";
const int port = 8883;
const char* mqttUser = "MQTTTuser";
const char* mqttPassword = "9ab8a1a9-a735-4fd4-ad80-12c109188cb7";

int scanTime = 10;
BLEScan* pBLEScan;

WiFiClientSecure espClient;
PubSubClient client(espClient);

float calculateDistance(int rssi);

const int ledPin = 13;

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    int rssi = advertisedDevice.getRSSI();
    float distance = calculateDistance(rssi);

    if (distance <= 10.0) {
      const char* topic = "ble/info";

      // Criação do objeto JSON
      DynamicJsonDocument jsonBuffer(256);

      // Preenchimento dos dados no objeto JSON
      JsonObject data = jsonBuffer.createNestedObject("data");
      data["Name"] = advertisedDevice.getName().c_str();
      data["Address"] = advertisedDevice.getAddress().toString().c_str();
      data["RSSI"] = rssi;
      data["Distance"] = distance;

      // Conversão do JSON em string
      String payload;
      serializeJson(data, payload);

      client.publish(topic, payload.c_str());

      // Acende o LED
      digitalWrite(ledPin, LOW);
    }
  }
};

void connectToMQTT() {
  while (!client.connected()) {
    Serial.println(F("Connecting to MQTT..."));
    if (client.connect("ESP32Client", mqttUser, mqttPassword)) {
      Serial.println(F("Connected to MQTT"));
    } else {
      Serial.print(F("Failed with state "));
      Serial.print(client.state());
      delay(2000);
    }
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println(F("Connecting to WiFi..."));
  }
  Serial.println(F("Connected to WiFi"));

  espClient.setInsecure();
  client.setServer(broker, port);

  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);
}

void loop() {
  if (!client.connected()) {
    connectToMQTT();
  }
  client.loop();

  Serial.println(F("Starting BLE scan..."));
  BLEScanResults foundDevices = pBLEScan->start(scanTime, false);

  // Verifica a distância e conta os dispositivos dentro do limite
  int deviceCount = 0;
  for (int i = 0; i < foundDevices.getCount(); i++) {
    BLEAdvertisedDevice device = foundDevices.getDevice(i);
    int rssi = device.getRSSI();
    float distance = calculateDistance(rssi);

    if (distance <= 10.0) {
      deviceCount++;
    }
  }

  const char* topic = "ble/devices";

  // Criação do objeto JSON
  DynamicJsonDocument jsonBuffer(256);

  // Preenchimento dos dados no objeto JSON
  JsonObject data = jsonBuffer.createNestedObject("data");
  data["DeviceCount"] = deviceCount;

  // Conversão do JSON em string
  String payload;
  serializeJson(data, payload);

  client.publish(topic, payload.c_str());

  // Apaga o LED se nenhum dispositivo foi encontrado
  if (deviceCount == 0) {
    digitalWrite(ledPin, HIGH);
  }

  pBLEScan->clearResults();
  delay(10000);
}

float calculateDistance(int rssi) {
  float txPower = -59.0;  // Potência de transmissão de referência
  float pathLossExponent = 2.0;  // Expoente de perda de percurso 

  // Converter o RSSI para dBm
  float rssi_dBm = float(rssi);

  // Calcular a distância usando a fórmula do modelo de perda de caminho
  float distance = pow(10.0, (txPower - rssi_dBm) / (10.0 * pathLossExponent));

  return distance;
}

