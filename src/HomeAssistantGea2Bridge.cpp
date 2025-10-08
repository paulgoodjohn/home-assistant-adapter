/*!
 * @file
 * @brief
 */

#include <Arduino.h>
#include "HomeAssistantGea2Bridge.h"

extern "C" {
#include "tiny_time_source.h"
}

static const tiny_gea2_erd_client_configuration_t client_configuration = {
  .request_timeout = 250,
  .request_retries = 10
};

void HomeAssistantGea2Bridge::begin(PubSubClient& pubSubClient, Stream& uart, const char* deviceId, uint8_t clientAddress)
{
  Serial.println("GEA2 bridge startup");
  this->pubSubClient = &pubSubClient;

  Serial.println("Timer group startup");
  tiny_timer_group_init(&timer_group, tiny_time_source_init());

  Serial.println("UART startup");
  tiny_uart_adapter_init(&uart_adapter, &timer_group, uart);

  Serial.println("MQTT client adapter init");
  mqtt_client_adapter_init(&client_adapter, &pubSubClient, deviceId);

  Serial.println("Fake msec interrupt init");
  tiny_event_init(&fakeMsecInterrupt);
  tiny_timer_start_periodic(
    &timer_group, &fakeMsecTimer, 1, &fakeMsecInterrupt, +[](void* context) {
      auto msecInterrupt = reinterpret_cast<tiny_event_t*>(context);
      tiny_event_publish(msecInterrupt, nullptr);
    });

  Serial.println("GEA2 interface startup");
  tiny_gea2_interface_init(
    &gea2_interface,
    &uart_adapter.interface,
    tiny_time_source_init(),
    &fakeMsecInterrupt.interface,
    clientAddress,
    send_queue_buffer,
    sizeof(send_queue_buffer),
    receive_buffer,
    sizeof(receive_buffer),
    false,
    3);

  targetAddress = 0xFF; // Start with broadcast

  Serial.println("GEA2 erd client startup");
  tiny_gea2_erd_client_init(
    &erd_client,
    &timer_group,
    &gea2_interface.interface,
    client_queue_buffer,
    sizeof(client_queue_buffer),
    &client_configuration);

  Serial.println("MQTT bridge init");
  gea2_mqtt_bridge_init(
    &gea2_mqtt_bridge,
    &timer_group,
    &erd_client.interface,
    &client_adapter.interface);
  Serial.println("GEA2 bridge started");
}

void HomeAssistantGea2Bridge::deviceAddress(uint8_t deviceAddress)
{
  targetAddress = deviceAddress;
}

void HomeAssistantGea2Bridge::loop()
{
  pubSubClient->loop();
  tiny_timer_group_run(&timer_group);
  tiny_gea2_interface_run(&gea2_interface);
}

void HomeAssistantGea2Bridge::notifyMqttDisconnected()
{
  mqtt_client_adapter_notify_mqtt_disconnected(&client_adapter);
}
