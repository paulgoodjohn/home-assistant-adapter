#include <Arduino.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include "Config.h"
#include "HomeAssistantBridge.h"

#ifdef MQTT_TLS
static WiFiClientSecure wifiClient;
#else
static WiFiClient wifiClient;
#endif
static PubSubClient mqttClient(wifiClient);
static HomeAssistantBridge bridge;

static void configureWifi()
{
  Serial.println("\nConnecting to " + String(ssid));

  WiFi.begin(ssid, password);

  while(WiFi.status() != WL_CONNECTED) {
    delay(100);
    Serial.print(".");
  }

#ifdef MQTT_TLS
#ifdef MQTT_TLS_VERIFY
  X509List* cert = new X509List(CERT);
  wifiClient.setTrustAnchors(cert);
#else
  wifiClient.setInsecure();
#endif
#endif

  Serial.println("WiFi connected");
}

static void configureMqtt()
{
  mqttClient.setServer(mqtt_server, mqtt_server_port);
}

static void connectToMqtt()
{
  while(WiFi.status() != WL_CONNECTED) {
    digitalWrite(LED_WIFI, LOW);
    delay(100);
    Serial.print(".");
  }
  digitalWrite(LED_WIFI, HIGH);

  if(!mqttClient.connected()) {
    digitalWrite(LED_MQTT, LOW);

    while(!mqttClient.connected()) {
      Serial.print("Attempting MQTT connection...");

      if(mqttClient.connect("", mqttUser, mqttPassword)) {
        Serial.println("connected");
        digitalWrite(LED_MQTT, HIGH);
      }
      else {
        Serial.println("failed, rc=" + String(mqttClient.state()) + " will try again in 5 seconds");
        delay(5000);
      }
    }

    bridge.notifyMqttDisconnected();
  }
}

void setup()
{
  Serial.begin(115200);

  pinMode(LED_HEARTBEAT, OUTPUT);
  pinMode(LED_WIFI, OUTPUT);
  pinMode(LED_MQTT, OUTPUT);

  configureWifi();
  configureMqtt();

  Serial1.begin(HomeAssistantBridge::baud, SERIAL_8N1, D7, D6);
  bridge.begin(mqttClient, Serial1, deviceId);
}

void loop()
{
  connectToMqtt();
  bridge.loop();
  digitalWrite(LED_HEARTBEAT, millis() % 1000 < 500);
}
