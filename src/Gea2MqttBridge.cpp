/*!
 * @file
 * @brief
 */

#include <Arduino.h>

extern "C" {
#include "Gea2MqttBridge.h"
#include "i_tiny_gea2_erd_client.h"
#include "tiny_gea_constants.h"
#include "tiny_utils.h"
}

#include <set>

using namespace std;

typedef Gea2MqttBridge_t self_t;

static void erd_client_activity(void* _self, const void* _args)
{
  reinterpret(self, _self, self_t*);
  reinterpret(args, _args, const tiny_gea2_erd_client_on_activity_args_t*);
  // const tiny_gea2_erd_client_on_activity_args_t* args = (const tiny_gea2_erd_client_on_activity_args_t*)_args;

  (void)self;

  Serial.println("Erd activity, type=" + String(args->type) + " address " + String(args->address));
  if(args->type == tiny_gea2_erd_client_activity_type_read_completed) {
    // targetAddress = args->address;

    Serial.println("Erd " + String(args->read_completed.erd));

    const uint8_t* ptr = (const uint8_t*)args->read_completed.data;
    uint8_t applianceType = *ptr;
    Serial.println("Read " + String(applianceType));
  }
}

void gea2_mqtt_bridge_init(
  self_t* self,
  tiny_timer_group_t* timer_group,
  i_tiny_gea2_erd_client_t* erd_client,
  i_mqtt_client_t* mqtt_client)
{
  Serial.println("bridge init start");
  self->timer_group = timer_group;
  self->erd_client = erd_client;
  self->mqtt_client = mqtt_client;
  self->erd_set = reinterpret_cast<void*>(new set<tiny_erd_t>());

  tiny_event_subscription_init(&self->erd_client_activity_subscription, self, erd_client_activity);
  tiny_event_subscribe(tiny_gea2_erd_client_on_activity(self->erd_client), &self->erd_client_activity_subscription);

  if(tiny_gea2_erd_client_read(self->erd_client, &self->request_id, tiny_gea_broadcast_address, 0x0008)) {
    Serial.println("Queued read erd 0x0008");
  }
  else {
    Serial.println("Failed to read erd 0x0008");
  }
  Serial.println("bridge init done");
}

void gea2_mqtt_bridge_destroy(self_t* self)
{
  delete reinterpret_cast<set<tiny_erd_t>*>(self->erd_set);
}
