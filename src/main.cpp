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
  else {
  }
}

void onceEveryFiveSeconds()
{
  Serial.println("Every 5 seconds");
}

void onceEveryFifteenSeconds()
{
  Serial.println("Every 15 seconds");
}

void flashHeartbeatLed()
{
  static bool state;
  state = !state;
  digitalWrite(LED_HEARTBEAT, !state);
}

// Simple scheduler stolen from Rick Suel's IoT class
typedef struct
{
  unsigned long previousMillis;
  unsigned long elapsedMillis;
  unsigned long timeoutMillis;
  void (*callback)();
} Timer_t;

static Timer_t schedulerTable[] = {
  { 0, 0, 500, &flashHeartbeatLed },
  { 0, 0, 1000, &connectToMqtt },
  { 0, 0, 5000, &onceEveryFiveSeconds },
  { 0, 0, 15000, &onceEveryFifteenSeconds },
};

void runScheduler()
{
  // Run each timer in the scheduler table, and call
  for(int i = 0; i < sizeof(schedulerTable) / sizeof(Timer_t); i++) {
    // Note: millis() will overflow after ~50 days.
    unsigned long currentMillis = millis();
    Timer_t* t = &schedulerTable[i];
    t->elapsedMillis += currentMillis - t->previousMillis;
    t->previousMillis = currentMillis;
    if(t->elapsedMillis >= t->timeoutMillis) {
      t->elapsedMillis = 0;
      t->callback();
    }
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
  bridge.loop();
  runScheduler();
}
