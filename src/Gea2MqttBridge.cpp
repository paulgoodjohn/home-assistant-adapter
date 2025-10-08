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

#include "ErdLists.h"

using namespace std;

enum {
  retry_delay = 1000,
};

enum {
  signal_start = tiny_hsm_signal_user_start,
  signal_timer_expired,
  signal_read_failed,
  signal_read_completed,
  signal_mqtt_disconnected,
  signal_write_requested
};

typedef Gea2MqttBridge_t self_t;

static void arm_timer(Gea2MqttBridge_t* self, tiny_timer_ticks_t ticks)
{
  tiny_timer_start(
    self->timer_group, &self->timer, ticks, self, +[](void* context) {
      tiny_hsm_send_signal(&reinterpret_cast<Gea2MqttBridge_t*>(context)->hsm, signal_timer_expired, nullptr);
    });
}

static void arm_periodic_timer(Gea2MqttBridge_t* self, tiny_timer_ticks_t ticks)
{
  tiny_timer_start_periodic(
    self->timer_group, &self->timer, ticks, self, +[](void* context) {
      tiny_hsm_send_signal(&reinterpret_cast<Gea2MqttBridge_t*>(context)->hsm, signal_timer_expired, nullptr);
    });
}

static void disarm_timer(Gea2MqttBridge_t* self)
{
  tiny_timer_stop(self->timer_group, &self->timer);
}

static set<tiny_erd_t>& erd_set(Gea2MqttBridge_t* self)
{
  return *reinterpret_cast<set<tiny_erd_t>*>(self->erd_set);
}

static tiny_hsm_result_t state_top(tiny_hsm_t* hsm, tiny_hsm_signal_t signal, const void* data);
static tiny_hsm_result_t state_starting_up(tiny_hsm_t* hsm, tiny_hsm_signal_t signal, const void* data);
static tiny_hsm_result_t state_add_common_erds(tiny_hsm_t* hsm, tiny_hsm_signal_t signal, const void* data);
static tiny_hsm_result_t state_add_energy_erds(tiny_hsm_t* hsm, tiny_hsm_signal_t signal, const void* data);
static tiny_hsm_result_t state_add_appliance_erds(tiny_hsm_t* hsm, tiny_hsm_signal_t signal, const void* data);
static tiny_hsm_result_t state_polling(tiny_hsm_t* hsm, tiny_hsm_signal_t signal, const void* data);

static tiny_hsm_result_t state_top(tiny_hsm_t* hsm, tiny_hsm_signal_t signal, const void* data)
{
  Gea2MqttBridge_t* self = container_of(Gea2MqttBridge_t, hsm, hsm);
  (void)data;

  switch(signal) {
    case signal_write_requested: {
      auto args = reinterpret_cast<const mqtt_client_on_write_request_args_t*>(data);
      tiny_gea2_erd_client_write(self->erd_client, &self->request_id, self->erd_host_address, args->erd, args->value, args->size);
    } break;

    default:
      return tiny_hsm_result_signal_deferred;
  }

  return tiny_hsm_result_signal_consumed;
}

static tiny_hsm_result_t state_starting_up(tiny_hsm_t* hsm, tiny_hsm_signal_t signal, const void* data)
{
  Gea2MqttBridge_t* self = container_of(Gea2MqttBridge_t, hsm, hsm);
  auto args = reinterpret_cast<const tiny_gea2_erd_client_on_activity_args_t*>(data);

  switch(signal) {
    case tiny_hsm_signal_entry: {
      self->erd_host_address = tiny_gea_broadcast_address;
    }
      __attribute__((fallthrough));

    case signal_timer_expired: {
      if(!tiny_gea2_erd_client_read(self->erd_client, &self->request_id, self->erd_host_address, 0x0008)) {
        arm_timer(self, retry_delay);
      }
      break;
    }
    case signal_read_completed: {
      disarm_timer(self);
      if(args->read_completed.erd == 0x0008) {
        self->erd_host_address = args->address;
      }

      const uint8_t* applianceTypeResponse = (const uint8_t*)args->read_completed.data;
      self->appliance_type = *applianceTypeResponse;
      tiny_hsm_transition(hsm, state_add_common_erds);
      break;
    }
    case tiny_hsm_signal_exit: {
      disarm_timer(self);
      break;
    }
    default:
      return tiny_hsm_result_signal_deferred;
  }

  return tiny_hsm_result_signal_consumed;
}

static tiny_hsm_result_t state_add_common_erds(tiny_hsm_t* hsm, tiny_hsm_signal_t signal, const void* data)
{
  char buffer[40];
  Gea2MqttBridge_t* self = container_of(Gea2MqttBridge_t, hsm, hsm);
  auto args = reinterpret_cast<const tiny_gea2_erd_client_on_activity_args_t*>(data);

  switch(signal) {
    case tiny_hsm_signal_entry:
      self->applianceErdList = commonErds;
      self->applianceErdListCount = commonErdCount;
      Serial.println("Starting looking for " + String(self->applianceErdListCount) + " common erds");
      self->erd_index = 0;
      self->pollingListCount = 0;
      sprintf(buffer, "Start read erd %04X\n", self->applianceErdList[self->erd_index]);
      Serial.print(buffer);
      tiny_gea2_erd_client_read(self->erd_client, &self->request_id, self->erd_host_address, self->applianceErdList[self->erd_index]);
      arm_timer(self, retry_delay);
      break;

    case signal_timer_expired:
      Serial.print(".");
      self->erd_index++;
      if(self->erd_index >= self->applianceErdListCount) {
        Serial.println("Found " + String(self->pollingListCount) + " erds");
        tiny_hsm_transition(hsm, state_add_energy_erds);
      }
      else {
        sprintf(buffer, "Start read erd %04X\n", self->applianceErdList[self->erd_index]);
        Serial.print(buffer);

        tiny_gea2_erd_client_read(self->erd_client, &self->request_id, self->erd_host_address, self->applianceErdList[self->erd_index]);
        arm_timer(self, retry_delay);
      }
      break;

    case signal_read_completed:
      disarm_timer(self);
      if(args->read_completed.erd == self->applianceErdList[self->erd_index]) {
        self->erd_polling_list[self->pollingListCount++] = args->read_completed.erd;

        sprintf(buffer, "Read erd #%d  %04X ok, length was %d\n", self->pollingListCount, args->read_completed.erd, args->read_completed.data_size);
        Serial.print(buffer);
      }
      self->erd_index++;
      if(self->erd_index >= self->applianceErdListCount) {
        Serial.println("Found " + String(self->pollingListCount) + " erds from " + String(commonErdCount));
        tiny_hsm_transition(hsm, state_add_energy_erds);
      }
      else {
        sprintf(buffer, "Start read erd %04X\n", self->applianceErdList[self->erd_index]);
        Serial.print(buffer);

        tiny_gea2_erd_client_read(self->erd_client, &self->request_id, self->erd_host_address, self->applianceErdList[self->erd_index]);
        arm_timer(self, retry_delay);
      }
      break;

    default:
      return tiny_hsm_result_signal_deferred;
  }

  return tiny_hsm_result_signal_consumed;
}

static tiny_hsm_result_t state_add_energy_erds(tiny_hsm_t* hsm, tiny_hsm_signal_t signal, const void* data)
{
  char buffer[40];
  Gea2MqttBridge_t* self = container_of(Gea2MqttBridge_t, hsm, hsm);
  auto args = reinterpret_cast<const tiny_gea2_erd_client_on_activity_args_t*>(data);
  switch(signal) {
    case tiny_hsm_signal_entry:
      self->applianceErdList = energyErds;
      self->applianceErdListCount = energyErdCount;
      Serial.println("Starting looking for " + String(self->applianceErdListCount) + " energy erds");
      self->erd_index = 0;

      sprintf(buffer, "Start read erd %04X\n", self->applianceErdList[self->erd_index]);
      Serial.print(buffer);

      tiny_gea2_erd_client_read(self->erd_client, &self->request_id, self->erd_host_address, self->applianceErdList[self->erd_index]);
      arm_timer(self, retry_delay);
      break;

    case signal_timer_expired:
      Serial.print(".");
      self->erd_index++;
      if(self->erd_index >= self->applianceErdListCount) {
        Serial.println("Found " + String(self->pollingListCount) + " erds");
        tiny_hsm_transition(hsm, state_add_appliance_erds);
      }
      else {
        sprintf(buffer, "Start read erd %04X\n", self->applianceErdList[self->erd_index]);
        Serial.print(buffer);

        tiny_gea2_erd_client_read(self->erd_client, &self->request_id, self->erd_host_address, self->applianceErdList[self->erd_index]);
        arm_timer(self, retry_delay);
      }
      break;

    case signal_read_completed:
      disarm_timer(self);
      if(args->read_completed.erd == self->applianceErdList[self->erd_index]) {
        self->erd_polling_list[self->pollingListCount++] = args->read_completed.erd;

        sprintf(buffer, "Read erd #%d  %04X ok, length was %d\n", self->pollingListCount, args->read_completed.erd, args->read_completed.data_size);
        Serial.print(buffer);
      }
      self->erd_index++;
      if(self->erd_index >= self->applianceErdListCount) {
        Serial.println("Found " + String(self->pollingListCount) + " erds");
        tiny_hsm_transition(hsm, state_add_appliance_erds);
      }
      else {
        sprintf(buffer, "Start read erd %04X\n", self->applianceErdList[self->erd_index]);
        Serial.print(buffer);

        tiny_gea2_erd_client_read(self->erd_client, &self->request_id, self->erd_host_address, self->applianceErdList[self->erd_index]);
        arm_timer(self, retry_delay);
      }
      break;

    default:
      return tiny_hsm_result_signal_deferred;
  }

  return tiny_hsm_result_signal_consumed;
}

static tiny_hsm_result_t state_add_appliance_erds(tiny_hsm_t* hsm, tiny_hsm_signal_t signal, const void* data)
{
  char buffer[40];
  Gea2MqttBridge_t* self = container_of(Gea2MqttBridge_t, hsm, hsm);
  auto args = reinterpret_cast<const tiny_gea2_erd_client_on_activity_args_t*>(data);
  switch(signal) {
    case tiny_hsm_signal_entry:
      self->applianceErdList = applianceTypeToErdGroupTranslation[self->appliance_type].erdList;
      self->applianceErdListCount = applianceTypeToErdGroupTranslation[self->appliance_type].erdCount;
      Serial.println("Starting looking for " + String(self->applianceErdListCount) + " appliance erds");
      self->erd_index = 0;

      sprintf(buffer, "Start read erd %04X\n", self->applianceErdList[self->erd_index]);
      Serial.print(buffer);

      tiny_gea2_erd_client_read(self->erd_client, &self->request_id, self->erd_host_address, self->applianceErdList[self->erd_index]);
      arm_timer(self, retry_delay);
      break;

    case signal_timer_expired:
      Serial.print(".");
      self->erd_index++;
      if(self->erd_index >= self->applianceErdListCount) {
        Serial.println("Found " + String(self->pollingListCount) + " appliance erds");
        tiny_hsm_transition(hsm, state_polling);
      }
      else {
        sprintf(buffer, "Start read erd %04X\n", self->applianceErdList[self->erd_index]);
        Serial.print(buffer);

        tiny_gea2_erd_client_read(self->erd_client, &self->request_id, self->erd_host_address, self->applianceErdList[self->erd_index]);
        arm_timer(self, retry_delay);
      }
      break;

    case signal_read_completed:
      disarm_timer(self);
      if(args->read_completed.erd == self->applianceErdList[self->erd_index]) {
        self->erd_polling_list[self->pollingListCount++] = args->read_completed.erd;

        sprintf(buffer, "Read erd #%d  %04X ok, length was %d\n", self->pollingListCount, args->read_completed.erd, args->read_completed.data_size);
        Serial.print(buffer);
      }
      self->erd_index++;
      if(self->erd_index >= self->applianceErdListCount) {
        Serial.println("Found " + String(self->pollingListCount) + " appliance erds");
        tiny_hsm_transition(hsm, state_polling);
      }
      else {
        sprintf(buffer, "Start read erd %04X\n", self->applianceErdList[self->erd_index]);
        Serial.print(buffer);

        tiny_gea2_erd_client_read(self->erd_client, &self->request_id, self->erd_host_address, self->applianceErdList[self->erd_index]);
        arm_timer(self, retry_delay);
      }
      break;

    default:
      return tiny_hsm_result_signal_deferred;
  }

  return tiny_hsm_result_signal_consumed;
}

static tiny_hsm_result_t state_polling(tiny_hsm_t* hsm, tiny_hsm_signal_t signal, const void* data)
{
  Gea2MqttBridge_t* self = container_of(Gea2MqttBridge_t, hsm, hsm);
  auto args = reinterpret_cast<const tiny_gea2_erd_client_on_activity_args_t*>(data);
  (void)self;
  (void)args;
  Serial.println("state_polling");
  switch(signal) {
      // case tiny_hsm_signal_entry:
      //  arm_periodic_timer(self, subscription_retention_period);
      // break;

      // case signal_timer_expired:
      //  tiny_gea3_erd_client_retain_subscription(self->erd_client, erd_host_address);
      // break;

      // case signal_mqtt_disconnected:
      // tiny_hsm_transition(hsm, state_starting_up);
      // break;

      // case tiny_hsm_signal_exit:
      // disarm_timer(self);
      // break;

    default:
      return tiny_hsm_result_signal_deferred;
  }

  return tiny_hsm_result_signal_consumed;
}

static const tiny_hsm_state_descriptor_t hsm_state_descriptors[] = {
  { .state = state_top, .parent = nullptr },
  { .state = state_starting_up, .parent = state_top },
  { .state = state_add_common_erds, .parent = state_top },
  { .state = state_add_energy_erds, .parent = state_top },
  { .state = state_add_appliance_erds, .parent = state_top },
  { .state = state_polling, .parent = state_top }
};
static const tiny_hsm_configuration_t hsm_configuration = {
  .states = hsm_state_descriptors,
  .state_count = element_count(hsm_state_descriptors)
};

void gea2_mqtt_bridge_init(
  self_t* self,
  tiny_timer_group_t* timer_group,
  i_tiny_gea2_erd_client_t* erd_client,
  i_mqtt_client_t* mqtt_client)
{
  Serial.println("Bridge init start");
  self->timer_group = timer_group;
  self->erd_client = erd_client;
  self->mqtt_client = mqtt_client;
  self->erd_set = reinterpret_cast<void*>(new set<tiny_erd_t>());

  tiny_event_subscription_init(
    &self->erd_client_activity_subscription, self, +[](void* context, const void* _args) {
      auto self = reinterpret_cast<Gea2MqttBridge_t*>(context);
      auto args = reinterpret_cast<const tiny_gea2_erd_client_on_activity_args_t*>(_args);

      switch(args->type) {
        case tiny_gea2_erd_client_activity_type_read_completed:
          tiny_hsm_send_signal(&self->hsm, signal_read_completed, args);
          break;

        case tiny_gea2_erd_client_activity_type_read_failed:
          tiny_hsm_send_signal(&self->hsm, signal_read_failed, args);
          break;

        case tiny_gea2_erd_client_activity_type_write_completed:
          mqtt_client_update_erd_write_result(self->mqtt_client, args->write_completed.erd, true, 0);
          break;

        case tiny_gea2_erd_client_activity_type_write_failed:
          mqtt_client_update_erd_write_result(self->mqtt_client, args->write_failed.erd, false, args->write_failed.reason);
          break;
      }
    });
  tiny_event_subscribe(tiny_gea2_erd_client_on_activity(erd_client), &self->erd_client_activity_subscription);

  tiny_event_subscription_init(
    &self->mqtt_write_request_subscription, self, +[](void* context, const void* _args) {
      auto self = reinterpret_cast<Gea2MqttBridge_t*>(context);
      auto args = reinterpret_cast<const mqtt_client_on_write_request_args_t*>(_args);
      tiny_hsm_send_signal(&self->hsm, signal_write_requested, args);
    });
  tiny_event_subscribe(mqtt_client_on_write_request(mqtt_client), &self->mqtt_write_request_subscription);

  tiny_event_subscription_init(
    &self->mqtt_disconnect_subscription, self, +[](void* context, const void*) {
      auto self = reinterpret_cast<Gea2MqttBridge_t*>(context);
      reinterpret_cast<set<tiny_erd_t>*>(self->erd_set)->clear();
      tiny_hsm_send_signal(&self->hsm, signal_mqtt_disconnected, nullptr);
    });
  tiny_event_subscribe(mqtt_client_on_mqtt_disconnect(mqtt_client), &self->mqtt_disconnect_subscription);

  Serial.println("Start HSM");
  tiny_hsm_init(&self->hsm, &hsm_configuration, state_starting_up);

  Serial.println("Bridge init done");
}

void gea2_mqtt_bridge_destroy(self_t* self)
{
  delete reinterpret_cast<set<tiny_erd_t>*>(self->erd_set);
}
