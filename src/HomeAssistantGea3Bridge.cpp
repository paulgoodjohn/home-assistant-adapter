/*!
 * @file
 * @brief
 */

#include <Arduino.h>
#include "HomeAssistantGea3Bridge.h"

extern "C" {
#include "tiny_time_source.h"
}

static const tiny_gea3_erd_client_configuration_t client_configuration = {
  .request_timeout = 250,
  .request_retries = 10
};

void HomeAssistantGea3Bridge::begin(PubSubClient& pubSubClient, Stream& uart, const char* deviceId, uint8_t clientAddress)
{
  Serial.println("GEA3 bridge startup");
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

  Serial.println("GEA3 interface startup");
  tiny_gea3_interface_init(
    &gea3_interface,
    &uart_adapter.interface,
    clientAddress,
    send_queue_buffer,
    sizeof(send_queue_buffer),
    receive_buffer,
    sizeof(receive_buffer),
    false);

  Serial.println("GEA3 erd client startup");
  tiny_gea3_erd_client_init(
    &erd_client,
    &timer_group,
    &gea3_interface.interface,
    client_queue_buffer,
    sizeof(client_queue_buffer),
    &client_configuration);

  Serial.println("MQTT bridge init");
  gea3_mqtt_bridge_init(
    &gea3_mqtt_bridge,
    &timer_group,
    &erd_client.interface,
    &client_adapter.interface);
  Serial.println("GEA3 bridge started");
}

void HomeAssistantGea3Bridge::loop()
{
  pubSubClient->loop();
  tiny_timer_group_run(&timer_group);
  tiny_gea3_interface_run(&gea3_interface);
}

void HomeAssistantGea3Bridge::notifyMqttDisconnected()
{
  mqtt_client_adapter_notify_mqtt_disconnected(&client_adapter);
}
