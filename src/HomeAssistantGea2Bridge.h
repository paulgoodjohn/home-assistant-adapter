/*!
 * @file
 * @brief
 */

#ifndef HomeAssistantGea2Bridge_h
#define HomeAssistantGea2Bridge_h

#include <PubSubClient.h>
#include <cstdint>
#include "mqtt_client_adapter.hpp"
#include "tiny_uart_adapter.hpp"

extern "C" {
#include "Gea2MqttBridge.h"
#include "tiny_gea2_erd_client.h"
#include "tiny_gea2_interface.h"
#include "tiny_timer.h"
}

class HomeAssistantGea2Bridge {
 public:
  static constexpr unsigned long baud = 19200;

  void begin(PubSubClient& client, Stream& uart, const char* deviceId, uint8_t clientAddress = 0xE4);
  void deviceAddress(uint8_t deviceAddress);
  void loop();
  void notifyMqttDisconnected();

 private:
  PubSubClient* pubSubClient;

  tiny_timer_group_t timer_group;

  tiny_event_t fakeMsecInterrupt;
  tiny_timer_t fakeMsecTimer;

  tiny_uart_adapter_t uart_adapter;
  mqtt_client_adapter_t client_adapter;

  tiny_gea2_interface_t gea2_interface;
  uint8_t receive_buffer[255];
  uint8_t send_queue_buffer[1000];

  tiny_gea2_erd_client_t erd_client;
  uint8_t client_queue_buffer[1024];
  tiny_gea2_erd_client_request_id_t requestId;

  tiny_event_subscription_t activity;
  uint8_t targetAddress;
  bool firstApplianceType;

  Gea2MqttBridge_t gea2_mqtt_bridge;
};

#endif
