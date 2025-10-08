#include <Arduino.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include "Config.h"
#include "HomeAssistantGea2Bridge.h"

#ifdef MQTT_TLS
static WiFiClientSecure wifiClient;
#else
static WiFiClient wifiClient;
#endif
static PubSubClient mqttClient(wifiClient);
static HomeAssistantGea2Bridge bridge;

static void connectToWifi()
{
  if(WiFi.status() == WL_CONNECTED) {
    return;
  }

  Serial.println("Connecting to WiFi...");

  unsigned retries = 0;
  while(WiFi.status() != WL_CONNECTED) {
    if(retries++ > 100) {
      Serial.println("WiFi connection failed, restarting...");
      ESP.restart();
    }

    digitalWrite(LED_WIFI, LOW);
    delay(100);
    Serial.print(".");
  }
}

static void configureWifi()
{
  Serial.println("WiFi SSID: " + String(ssid));

  WiFi.begin(ssid, password);

  connectToWifi();

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
  connectToWifi();
  digitalWrite(LED_WIFI, HIGH);

  if(!mqttClient.connected()) {
    digitalWrite(LED_MQTT, LOW);

    unsigned retries = 0;
    while(!mqttClient.connected()) {
      if(retries++ > 10) {
        Serial.println("MQTT connection failed, restarting...");
        ESP.restart();
      }

      Serial.print("Attempting MQTT connection...");

      if(mqttClient.connect("", mqttUser, mqttPassword)) {
        Serial.println("connected");
        digitalWrite(LED_MQTT, HIGH);
      }
      else {
        Serial.println("failed, rc=" + String(mqttClient.state()) + " will try again in 1 second");
        delay(1000);
      }
    }

    bridge.notifyMqttDisconnected();
  }
}

void setup()
{
  Serial.begin(115200);
  Serial.println();
  Serial.println("GEA2 adapter startup");

  pinMode(LED_HEARTBEAT, OUTPUT);
  pinMode(LED_WIFI, OUTPUT);
  pinMode(LED_MQTT, OUTPUT);

  configureWifi();
  configureMqtt();

  Serial1.begin(HomeAssistantGea2Bridge::baud, SERIAL_8N1, D10, D9);
  bridge.begin(mqttClient, Serial1, deviceId);
}

void loop()
{
  // connectToMqtt();
  bridge.loop();
  digitalWrite(LED_HEARTBEAT, millis() % 1000 < 500);
}
