/*!
 * @file
 * @brief
 */

#include <Arduino.h>

extern "C" {
#include "Gea3MqttBridge.h"
#include "i_tiny_gea3_erd_client.h"
#include "tiny_gea_constants.h"
#include "tiny_utils.h"
}

#include <Preferences.h>
#include <set>

#include "ErdLists.h"

using namespace std;
typedef Gea3MqttBridge_t self_t;

enum {
  retry_delay = 100,
  polling_delay = 10000,
  appliance_lost_timeout = 30000
};

enum {
  signal_start = tiny_hsm_signal_user_start,
  signal_timer_expired,
  signal_polling_timer_expired,
  signal_read_failed,
  signal_read_completed,
  signal_mqtt_disconnected,
  signal_appliance_lost,
  signal_write_requested
};

static Preferences nvStorage;
#define RW_MODE false
#define RO_MODE true

static bool valid_nv_data_loaded(self_t* self)
{
  self->pollingListCount = 0;
  if(nvStorage.begin("storage", RO_MODE)) {
    Serial.println("NV storage found and opened");
    self->pollingListCount = nvStorage.getUInt("erdCount", 0);
    Serial.println("Stored number of polled ERDs is " + String(self->pollingListCount));
    if(self->pollingListCount > 0) {
      size_t bytesRead = nvStorage.getBytes("erdList", self->erd_polling_list, sizeof(self->erd_polling_list));
      Serial.println("Loaded " + String(bytesRead) + " bytes from store");
    }
    nvStorage.end();
  }

  return (self->pollingListCount > 0);
}

static void save_polling_list_to_nv_data(self_t* self)
{
  if(nvStorage.begin("storage", RW_MODE)) {
    Serial.println("NV storage found and opened for write");
    if(nvStorage.clear()) {
      Serial.println("NV storage cleared");
    }
    else {
      Serial.println("NV storage not cleared");
    }
    size_t freeEntries = nvStorage.freeEntries();
    Serial.println("Initial free entries = " + String(freeEntries));
    size_t bytesWritten = nvStorage.putBytes("erdList", self->erd_polling_list, sizeof(self->erd_polling_list));
    Serial.println("Wrote " + String(bytesWritten) + " bytes to store for list");
    bytesWritten = nvStorage.putUInt("erdCount", self->pollingListCount);
    Serial.println("Wrote " + String(bytesWritten) + " bytes to store for count");
    Serial.println("Stored number of polled ERDs is " + String(self->pollingListCount));
    freeEntries = nvStorage.freeEntries();
    Serial.println("Final free entries = " + String(freeEntries));
    nvStorage.end();
  }
}

static void arm_timer(self_t* self, tiny_timer_ticks_t ticks)
{
  tiny_timer_start(
    self->timer_group, &self->timer, ticks, self, +[](void* context) {
      tiny_hsm_send_signal(&reinterpret_cast<self_t*>(context)->hsm, signal_timer_expired, nullptr);
    });
}

static void arm_polling_timer(self_t* self, tiny_timer_ticks_t ticks)
{
  Serial.println("Started polling timer");
  tiny_timer_start(
    self->timer_group, &self->pollingTimer, ticks, self, +[](void* context) {
      tiny_hsm_send_signal(&reinterpret_cast<self_t*>(context)->hsm, signal_polling_timer_expired, nullptr);
    });
}

static void reset_lost_appliance_timer(self_t* self)
{
  tiny_timer_stop(self->timer_group, &self->timer);
  tiny_timer_start(
    self->timer_group, &self->applianceLostTimer, appliance_lost_timeout, self, +[](void* context) {
      tiny_hsm_send_signal(&reinterpret_cast<self_t*>(context)->hsm, signal_appliance_lost, nullptr);
    });
}

static void disarm_timer(self_t* self)
{
  tiny_timer_stop(self->timer_group, &self->timer);
}

static set<tiny_erd_t>& erd_set(self_t* self)
{
  return *reinterpret_cast<set<tiny_erd_t>*>(self->erd_set);
}

static tiny_hsm_result_t state_top(tiny_hsm_t* hsm, tiny_hsm_signal_t signal, const void* data);
static tiny_hsm_result_t state_identify_appliance(tiny_hsm_t* hsm, tiny_hsm_signal_t signal, const void* data);
static tiny_hsm_result_t state_add_common_erds(tiny_hsm_t* hsm, tiny_hsm_signal_t signal, const void* data);
static tiny_hsm_result_t state_add_energy_erds(tiny_hsm_t* hsm, tiny_hsm_signal_t signal, const void* data);
static tiny_hsm_result_t state_add_appliance_erds(tiny_hsm_t* hsm, tiny_hsm_signal_t signal, const void* data);
static tiny_hsm_result_t state_polling(tiny_hsm_t* hsm, tiny_hsm_signal_t signal, const void* data);

static tiny_hsm_result_t state_top(tiny_hsm_t* hsm, tiny_hsm_signal_t signal, const void* data)
{
  self_t* self = container_of(self_t, hsm, hsm);
  (void)data;

  switch(signal) {
    case signal_write_requested: {
      auto args = reinterpret_cast<const mqtt_client_on_write_request_args_t*>(data);
      tiny_gea3_erd_client_write(self->erd_client, &self->request_id, self->erd_host_address, args->erd, args->value, args->size);
    } break;

    case signal_appliance_lost: {
      tiny_hsm_transition(hsm, state_identify_appliance);
    } break;

    default:
      return tiny_hsm_result_signal_deferred;
  }

  return tiny_hsm_result_signal_consumed;
}

static tiny_hsm_result_t state_identify_appliance(tiny_hsm_t* hsm, tiny_hsm_signal_t signal, const void* data)
{
  self_t* self = container_of(self_t, hsm, hsm);
  auto args = reinterpret_cast<const tiny_gea3_erd_client_on_activity_args_t*>(data);

  switch(signal) {
    case tiny_hsm_signal_entry: {
      self->erd_host_address = tiny_gea_broadcast_address;
    }
      __attribute__((fallthrough));

    case signal_timer_expired: {
      Serial.println("Asking for appliance type ERD 0x0008");
      tiny_gea3_erd_client_read(self->erd_client, &self->request_id, self->erd_host_address, 0x0008);
      arm_timer(self, retry_delay);
      break;
    }
    case signal_read_completed: {
      disarm_timer(self);
      reset_lost_appliance_timer(self);
      if(args->read_completed.erd == 0x0008) {
        self->erd_host_address = args->address;
      }

      const uint8_t* applianceTypeResponse = (const uint8_t*)args->read_completed.data;
      self->appliance_type = *applianceTypeResponse;
      if(self->appliance_type >= maximumApplianceType) {
        self->appliance_type = 0;
      }
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

static bool SendNextReadRequest(self_t* self)
{
  reset_lost_appliance_timer(self);
  self->erd_index++;
  bool more_erds_to_try = (self->erd_index < self->applianceErdListCount);
  if(more_erds_to_try) {
    self->request_id++;
    tiny_gea3_erd_client_read(self->erd_client, &self->request_id, self->erd_host_address, self->applianceErdList[self->erd_index]);
    arm_timer(self, retry_delay);
  }
  return more_erds_to_try;
}

static void AddErdToPollingList(self_t* self, tiny_erd_t erd)
{
  if(erd_set(self).find(erd) == erd_set(self).end()) {
    mqtt_client_register_erd(self->mqtt_client, erd);
    erd_set(self).insert(erd);
  }
  self->erd_polling_list[self->pollingListCount] = erd;
  self->pollingListCount++;

  char buffer[40];
  sprintf(buffer, "#%d Add ERD erd %04X to polling list\n", self->pollingListCount, erd);
  Serial.print(buffer);
}

static tiny_hsm_result_t state_add_common_erds(tiny_hsm_t* hsm, tiny_hsm_signal_t signal, const void* data)
{
  self_t* self = container_of(self_t, hsm, hsm);
  auto args = reinterpret_cast<const tiny_gea3_erd_client_on_activity_args_t*>(data);

  switch(signal) {
    case tiny_hsm_signal_entry:
      self->applianceErdList = commonErds;
      self->applianceErdListCount = commonErdCount;
      Serial.println("Starting looking for " + String(self->applianceErdListCount) + " common erds");
      self->erd_index = 0;
      self->pollingListCount = 0;
      tiny_gea3_erd_client_read(self->erd_client, &self->request_id, self->erd_host_address, self->applianceErdList[self->erd_index]);
      arm_timer(self, retry_delay);
      break;

    case signal_timer_expired:
      if(!SendNextReadRequest(self)) {
        tiny_hsm_transition(hsm, state_add_energy_erds);
      }
      break;

    case signal_read_completed:
      disarm_timer(self);
      AddErdToPollingList(self, args->read_completed.erd);
      mqtt_client_update_erd(
        self->mqtt_client,
        args->read_completed.erd,
        args->read_completed.data,
        args->read_completed.data_size);

      if(!SendNextReadRequest(self)) {
        tiny_hsm_transition(hsm, state_add_energy_erds);
      }
      break;

    default:
      return tiny_hsm_result_signal_deferred;
  }

  return tiny_hsm_result_signal_consumed;
}

static tiny_hsm_result_t state_add_energy_erds(tiny_hsm_t* hsm, tiny_hsm_signal_t signal, const void* data)
{
  self_t* self = container_of(self_t, hsm, hsm);
  auto args = reinterpret_cast<const tiny_gea3_erd_client_on_activity_args_t*>(data);
  switch(signal) {
    case tiny_hsm_signal_entry:
      self->applianceErdList = energyErds;
      self->applianceErdListCount = energyErdCount;
      Serial.println("Starting looking for " + String(self->applianceErdListCount) + " energy erds");
      self->erd_index = 0;

      tiny_gea3_erd_client_read(self->erd_client, &self->request_id, self->erd_host_address, self->applianceErdList[self->erd_index]);
      arm_timer(self, retry_delay);
      break;

    case signal_timer_expired:
      if(!SendNextReadRequest(self)) {
        tiny_hsm_transition(hsm, state_add_appliance_erds);
      }
      break;

    case signal_read_completed:
      disarm_timer(self);
      AddErdToPollingList(self, args->read_completed.erd);
      mqtt_client_update_erd(
        self->mqtt_client,
        args->read_completed.erd,
        args->read_completed.data,
        args->read_completed.data_size);

      if(!SendNextReadRequest(self)) {
        tiny_hsm_transition(hsm, state_add_appliance_erds);
      }
      break;

    default:
      return tiny_hsm_result_signal_deferred;
  }

  return tiny_hsm_result_signal_consumed;
}

static tiny_hsm_result_t state_add_appliance_erds(tiny_hsm_t* hsm, tiny_hsm_signal_t signal, const void* data)
{
  self_t* self = container_of(self_t, hsm, hsm);
  auto args = reinterpret_cast<const tiny_gea3_erd_client_on_activity_args_t*>(data);
  switch(signal) {
    case tiny_hsm_signal_entry:
      self->applianceErdList = applianceTypeToErdGroupTranslation[self->appliance_type].erdList;
      self->applianceErdListCount = applianceTypeToErdGroupTranslation[self->appliance_type].erdCount;
      Serial.println();
      Serial.println("Starting looking for " + String(self->applianceErdListCount) + " appliance erds");
      self->erd_index = 0;

      tiny_gea3_erd_client_read(self->erd_client, &self->request_id, self->erd_host_address, self->applianceErdList[self->erd_index]);
      arm_timer(self, retry_delay);
      break;

    case signal_timer_expired:
      if(!SendNextReadRequest(self)) {
        save_polling_list_to_nv_data(self);
        tiny_hsm_transition(hsm, state_polling);
      }
      break;

    case signal_read_completed:
      disarm_timer(self);
      AddErdToPollingList(self, args->read_completed.erd);
      mqtt_client_update_erd(
        self->mqtt_client,
        args->read_completed.erd,
        args->read_completed.data,
        args->read_completed.data_size);

      if(!SendNextReadRequest(self)) {
        save_polling_list_to_nv_data(self);
        tiny_hsm_transition(hsm, state_polling);
      }
      break;

    default:
      return tiny_hsm_result_signal_deferred;
  }

  return tiny_hsm_result_signal_consumed;
}

static void SendNextPollReadRequest(self_t* self)
{
  if(self->erd_index < self->pollingListCount) {
    Serial.println("Reading " + String(self->erd_polling_list[self->erd_index]) + " Erd");
    self->request_id++;
    tiny_gea3_erd_client_read(self->erd_client, &self->request_id, self->erd_host_address, self->erd_polling_list[self->erd_index]);
    self->erd_index++;
    arm_timer(self, retry_delay);
  }
}

static tiny_hsm_result_t state_polling(tiny_hsm_t* hsm, tiny_hsm_signal_t signal, const void* data)
{
  self_t* self = container_of(self_t, hsm, hsm);
  auto args = reinterpret_cast<const tiny_gea3_erd_client_on_activity_args_t*>(data);

  switch(signal) {
    case tiny_hsm_signal_entry:
      Serial.println("Polling " + String(self->pollingListCount) + " erds");
      arm_polling_timer(self, polling_delay);
      __attribute__((fallthrough));

    case signal_timer_expired:
      Serial.println("Reading " + String(self->erd_polling_list[self->erd_index - 1]) + " Erd failed");
      SendNextPollReadRequest(self);
      break;

    case signal_polling_timer_expired:
      if(self->erd_index >= self->pollingListCount) {
        self->erd_index = 0;
        SendNextPollReadRequest(self);
      }
      arm_polling_timer(self, polling_delay);
      break;

    case signal_read_completed:
      disarm_timer(self);
      reset_lost_appliance_timer(self);
      mqtt_client_update_erd(
        self->mqtt_client,
        args->read_completed.erd,
        args->read_completed.data,
        args->read_completed.data_size);
      Serial.println("Reading " + String(self->erd_polling_list[self->erd_index - 1]) + " Erd completed");

      SendNextPollReadRequest(self);
      break;

    case signal_mqtt_disconnected:
      if(valid_nv_data_loaded(self)) {
        Serial.println("Start HSM with previously discovered appliance");
        tiny_hsm_transition(&self->hsm, state_polling);
      }
      else {
        Serial.println("Start HSM and identify new appliance");
        tiny_hsm_transition(&self->hsm, state_identify_appliance);
      }
      break;

    case tiny_hsm_signal_exit:
      disarm_timer(self);
      break;

    default:
      return tiny_hsm_result_signal_deferred;
  }

  return tiny_hsm_result_signal_consumed;
}

static const tiny_hsm_state_descriptor_t hsm_state_descriptors[] = {
  { .state = state_top, .parent = nullptr },
  { .state = state_identify_appliance, .parent = state_top },
  { .state = state_add_common_erds, .parent = state_top },
  { .state = state_add_energy_erds, .parent = state_top },
  { .state = state_add_appliance_erds, .parent = state_top },
  { .state = state_polling, .parent = state_top }
};
static const tiny_hsm_configuration_t hsm_configuration = {
  .states = hsm_state_descriptors,
  .state_count = element_count(hsm_state_descriptors)
};

void gea3_mqtt_bridge_init(
  self_t* self,
  tiny_timer_group_t* timer_group,
  i_tiny_gea3_erd_client_t* erd_client,
  i_mqtt_client_t* mqtt_client)
{
  Serial.println("Bridge init start");
  self->timer_group = timer_group;
  self->erd_client = erd_client;
  self->mqtt_client = mqtt_client;
  self->erd_set = reinterpret_cast<void*>(new set<tiny_erd_t>());

  tiny_event_subscription_init(
    &self->erd_client_activity_subscription, self, +[](void* context, const void* _args) {
      auto self = reinterpret_cast<self_t*>(context);
      auto args = reinterpret_cast<const tiny_gea3_erd_client_on_activity_args_t*>(_args);

      switch(args->type) {
        case tiny_gea3_erd_client_activity_type_read_completed:
          tiny_hsm_send_signal(&self->hsm, signal_read_completed, args);
          break;

        case tiny_gea3_erd_client_activity_type_read_failed:
          tiny_hsm_send_signal(&self->hsm, signal_read_failed, args);
          break;

        case tiny_gea3_erd_client_activity_type_write_completed:
          mqtt_client_update_erd_write_result(self->mqtt_client, args->write_completed.erd, true, 0);
          break;

        case tiny_gea3_erd_client_activity_type_write_failed:
          mqtt_client_update_erd_write_result(self->mqtt_client, args->write_failed.erd, false, args->write_failed.reason);
          break;
      }
    });
  tiny_event_subscribe(tiny_gea3_erd_client_on_activity(erd_client), &self->erd_client_activity_subscription);

  tiny_event_subscription_init(
    &self->mqtt_write_request_subscription, self, +[](void* context, const void* _args) {
      auto self = reinterpret_cast<self_t*>(context);
      auto args = reinterpret_cast<const mqtt_client_on_write_request_args_t*>(_args);
      tiny_hsm_send_signal(&self->hsm, signal_write_requested, args);
    });
  tiny_event_subscribe(mqtt_client_on_write_request(mqtt_client), &self->mqtt_write_request_subscription);

  tiny_event_subscription_init(
    &self->mqtt_disconnect_subscription, self, +[](void* context, const void*) {
      auto self = reinterpret_cast<self_t*>(context);
      reinterpret_cast<set<tiny_erd_t>*>(self->erd_set)->clear();
      tiny_hsm_send_signal(&self->hsm, signal_mqtt_disconnected, nullptr);
    });
  tiny_event_subscribe(mqtt_client_on_mqtt_disconnect(mqtt_client), &self->mqtt_disconnect_subscription);

  if(valid_nv_data_loaded(self)) {
    Serial.println("Start HSM with previously discovered appliance");
    tiny_hsm_init(&self->hsm, &hsm_configuration, state_polling);
  }
  else {
    Serial.println("Start HSM and identify new appliance");
    tiny_hsm_init(&self->hsm, &hsm_configuration, state_identify_appliance);
  }

  Serial.println("Bridge init done");
}

void gea3_mqtt_bridge_destroy(self_t* self)
{
  delete reinterpret_cast<set<tiny_erd_t>*>(self->erd_set);
}
