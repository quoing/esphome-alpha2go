#pragma once

#include "protocol.h"
#include "esphome/components/alpha2_transport/alpha2_transport.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/select/select.h"
#include "esphome/components/number/number.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/core/automation.h"
#include "esphome/core/component.h"

#include <deque>
#include <cmath>
#include <functional>
#include <string>
#include <vector>

#ifdef USE_ESP32

#include <esp_gattc_api.h>

namespace esphome {
namespace alpha2 {

namespace espbt = esphome::esp32_ble_tracker;

class Alpha2Switch;
class Alpha2ModeSelect;
class Alpha2RegulationSelect;
class Alpha2SetpointNumber;

class Alpha2 : public alpha2_transport::Alpha2Transport, public PollingComponent {
 public:
  void loop() override;
  void update() override;
  void dump_config() override;

  void set_flow_sensor(sensor::Sensor *sensor) { this->flow_sensor_ = sensor; }
  void set_head_sensor(sensor::Sensor *sensor) { this->head_sensor_ = sensor; }
  void set_power_sensor(sensor::Sensor *sensor) { this->power_sensor_ = sensor; }
  void set_current_sensor(sensor::Sensor *sensor) { this->current_sensor_ = sensor; }
  void set_speed_sensor(sensor::Sensor *sensor) { this->speed_sensor_ = sensor; }
  void set_voltage_sensor(sensor::Sensor *sensor) { this->voltage_sensor_ = sensor; }
  void set_pump_switch(Alpha2Switch *pump_switch) { this->pump_switch_ = pump_switch; }
  void set_mode_select(Alpha2ModeSelect *entity) { this->mode_select_ = entity; }
  void set_regulation_select(Alpha2RegulationSelect *entity) { this->regulation_select_ = entity; }
  void set_pressure_number(Alpha2SetpointNumber *entity) { this->pressure_number_ = entity; }
  void set_flow_number(Alpha2SetpointNumber *entity) { this->flow_number_ = entity; }
  void set_curve_number(Alpha2SetpointNumber *entity) { this->curve_number_ = entity; }
  void set_setting_switch(protocol::SettingKind kind, Alpha2Switch *entity) { this->setting_switches_[static_cast<size_t>(kind)] = entity; }
  void set_setting_number(protocol::SettingKind kind, Alpha2SetpointNumber *entity) { this->setting_numbers_[static_cast<size_t>(kind)] = entity; }
  void set_command_status(text_sensor::TextSensor *entity) { this->command_status_ = entity; }
  void set_max_rpm(float value) { this->max_rpm_ = value; }

  bool read_object(uint8_t object_id, uint16_t sub_id);
  bool send_raw_hex(const std::string &hex);
  bool set_enabled(bool enabled);
  bool set_mode(uint8_t raw_mode);
  bool set_setpoint(protocol::SetpointKind kind, float value);
  bool set_setting(protocol::SettingKind kind, bool enabled, float value = NAN);

 protected:
  void on_geni_frame_(const std::vector<uint8_t> &frame) override { this->handle_frame_(frame); }
  void on_transport_disconnect_() override;
  void on_transport_ready_() override;
  void handle_frame_(const std::vector<uint8_t> &frame);
  void publish_float_(const std::vector<uint8_t> &frame, size_t offset, sensor::Sensor *sensor, float factor);
  void reset_sensors_();
  bool queue_transaction_(std::vector<uint8_t> frame, std::function<void(const std::vector<uint8_t> *)> callback);
  void finish_transaction_(const std::vector<uint8_t> *response);
  void log_unmatched_(const std::vector<uint8_t> &frame, protocol::ResponseType type) const;
  void publish_setting_(protocol::SettingKind kind, const protocol::SettingState &state);
  void read_setting_(protocol::SettingKind kind);
  bool start_write_(std::vector<uint8_t> frame, uint8_t expected_mode, bool verify_run,
                    bool requested_run, float expected_setpoint, float tolerance, const char *label);
  void verify_control_(uint8_t expected_mode, bool verify_run, bool requested_run,
                        float expected_setpoint, float tolerance, const char *label, uint8_t attempt = 1);
  void verify_setting_(protocol::SettingKind kind, bool enabled, float target, uint8_t attempt = 1);

  sensor::Sensor *flow_sensor_{nullptr};
  sensor::Sensor *head_sensor_{nullptr};
  sensor::Sensor *power_sensor_{nullptr};
  sensor::Sensor *current_sensor_{nullptr};
  sensor::Sensor *speed_sensor_{nullptr};
  sensor::Sensor *voltage_sensor_{nullptr};
  Alpha2Switch *pump_switch_{nullptr};
  Alpha2ModeSelect *mode_select_{nullptr};
  Alpha2RegulationSelect *regulation_select_{nullptr};
  Alpha2SetpointNumber *pressure_number_{nullptr};
  Alpha2SetpointNumber *flow_number_{nullptr};
  Alpha2SetpointNumber *curve_number_{nullptr};
  Alpha2Switch *setting_switches_[5]{};
  Alpha2SetpointNumber *setting_numbers_[5]{};
  protocol::SettingState setting_states_[5]{};
  bool setting_valid_[5]{};
  text_sensor::TextSensor *command_status_{nullptr};
  float max_rpm_{5837.0f};
  bool control_busy_{false};
  const char *command_phase_{"IDLE"};

  uint32_t last_tx_ms_{0};
  uint32_t request_sent_ms_{0};
  uint32_t next_tx_not_before_ms_{0};
  bool awaiting_response_{false};
  std::function<void(const std::vector<uint8_t> *)> transaction_callback_;
  std::vector<uint8_t> pending_frame_;
  std::deque<std::pair<std::vector<uint8_t>, std::function<void(const std::vector<uint8_t> *)>>> transactions_;
  protocol::OperatingState operating_state_{};
  bool operating_state_valid_{false};
  size_t tx_offset_{0};
  std::deque<std::vector<uint8_t>> tx_queue_;
};

class Alpha2Switch : public switch_::Switch, public Parented<Alpha2> {
 public:
  void set_setting_kind(protocol::SettingKind kind) { this->kind_ = kind; this->setting_ = true; }
 protected:
  void write_state(bool state) override;
  protocol::SettingKind kind_{protocol::SettingKind::NIGHT_SETBACK};
  bool setting_{false};
};

class Alpha2ModeSelect : public select::Select, public Parented<Alpha2> {
 protected:
  void control(const std::string &value) override;
};

class Alpha2RegulationSelect : public select::Select, public Parented<Alpha2> {
 protected:
  void control(const std::string &value) override;
};

class Alpha2SetpointNumber : public number::Number, public Parented<Alpha2> {
 public:
  void set_kind(protocol::SetpointKind kind) { this->kind_ = kind; }
  void set_setting_kind(protocol::SettingKind kind) { this->setting_kind_ = kind; this->setting_ = true; }
 protected:
  void control(float value) override;
  protocol::SetpointKind kind_{protocol::SetpointKind::PRESSURE};
  protocol::SettingKind setting_kind_{protocol::SettingKind::MIN_FLOW};
  bool setting_{false};
};

template<typename... Ts> class SendRawAction : public Action<Ts...>, public Parented<Alpha2> {
 public:
  TEMPLATABLE_VALUE(std::string, data)

  void play(Ts... x) override { this->parent_->send_raw_hex(this->data_.value(x...)); }
};

template<typename... Ts> class ReadObjectAction : public Action<Ts...>, public Parented<Alpha2> {
 public:
  TEMPLATABLE_VALUE(uint8_t, object_id)
  TEMPLATABLE_VALUE(uint16_t, sub_id)

  void play(Ts... x) override { this->parent_->read_object(this->object_id_.value(x...), this->sub_id_.value(x...)); }
};

}  // namespace alpha2
}  // namespace esphome

#endif
