#include "alpha2.h"

#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
#include <utility>

#ifdef USE_ESP32

namespace esphome {
namespace alpha2 {

static const char *const TAG = "alpha2";

static const uint8_t FLOW_HEAD_RESPONSE_TYPE[] = {0x2B, 0x00, 0x02, 0x35, 0x02, 0x00, 0x00, 0x24};
static const uint8_t POWER_RESPONSE_TYPE[] = {0x30, 0x00, 0x01, 0x00, 0x03, 0x00, 0x00, 0x29};
static const size_t RESPONSE_TYPE_OFFSET = 5;
static const size_t RESPONSE_TYPE_LENGTH = 8;
static const size_t PAYLOAD_OFFSET = 13;
static const uint32_t TX_SPACING_MS = 25;
static const uint32_t RESPONSE_TIMEOUT_MS = 1500;
static const uint32_t LATE_RESPONSE_QUIET_MS = 1100;
static const uint8_t VERIFY_MAX_ATTEMPTS = 3;
static const size_t MAX_GENI_FRAME_SIZE = 259;
static const size_t MAX_QUEUED_FRAMES = 8;
void Alpha2::dump_config() {
  ESP_LOGCONFIG(TAG, "Grundfos ALPHA2 GO");
  LOG_SENSOR("  ", "Flow", this->flow_sensor_);
  LOG_SENSOR("  ", "Head", this->head_sensor_);
  LOG_SENSOR("  ", "Power", this->power_sensor_);
  LOG_SENSOR("  ", "Current", this->current_sensor_);
  LOG_SENSOR("  ", "Speed", this->speed_sensor_);
  LOG_SENSOR("  ", "Voltage", this->voltage_sensor_);
}

bool Alpha2::queue_transaction_(std::vector<uint8_t> frame,
                                 std::function<void(const std::vector<uint8_t> *)> callback) {
  if (!this->transport_ready() || frame.empty() || frame.size() > MAX_GENI_FRAME_SIZE)
    return false;
  if (this->transactions_.size() >= MAX_QUEUED_FRAMES) {
    if (!this->control_busy_) return false;
    // Queued background reads are lower priority than the active control sequence.
    this->transactions_.pop_back();
  }
  if (this->control_busy_) this->transactions_.emplace_front(std::move(frame), std::move(callback));
  else this->transactions_.emplace_back(std::move(frame), std::move(callback));
  return true;
}

void Alpha2::finish_transaction_(const std::vector<uint8_t> *response) {
  this->awaiting_response_ = false;
  auto callback = std::move(this->transaction_callback_);
  this->transaction_callback_ = nullptr;
  this->pending_frame_.clear();
  if (response == nullptr) this->next_tx_not_before_ms_ = millis() + LATE_RESPONSE_QUIET_MS;
  if (callback) callback(response);
}

bool Alpha2::start_write_(std::vector<uint8_t> frame, uint8_t expected_mode, bool verify_run,
                          bool requested_run, float expected_setpoint, float tolerance, const char *label) {
  if (this->control_busy_ || !this->operating_state_valid_) {
    ESP_LOGW(TAG, "%s command refused: operating state unavailable or another command in progress", label);
    return false;
  }
  this->control_busy_ = true;
  this->command_phase_ = "WAIT_WRITE_ACK";
  if (!this->queue_transaction_(std::move(frame), [this, expected_mode, verify_run, requested_run,
                                                 expected_setpoint, tolerance, label](const std::vector<uint8_t> *response) {
        uint8_t status;
        if (response == nullptr || !protocol::parse_write_ack(response->data(), response->size(), status)) {
          ESP_LOGW(TAG, "%s write failed: timeout or unexpected ACK", label);
          if (this->command_status_ != nullptr) this->command_status_->publish_state("Write timeout or invalid ACK");
          this->control_busy_ = false;
          this->command_phase_ = "IDLE";
        } else if (status != 0) {
          ESP_LOGW(TAG, "%s write rejected, status=0x%02X", label, status);
          if (this->command_status_ != nullptr) {
            char message[32];
            snprintf(message, sizeof(message), "Write rejected: 0x%02X", status);
            this->command_status_->publish_state(message);
          }
          this->control_busy_ = false;
          this->command_phase_ = "IDLE";
        } else {
          this->verify_control_(expected_mode, verify_run, requested_run, expected_setpoint, tolerance, label);
        }
      })) {
    this->control_busy_ = false;
    this->command_phase_ = "IDLE";
    return false;
  }
  return true;
}

void Alpha2::verify_control_(uint8_t expected_mode, bool verify_run, bool requested_run,
                             float expected_setpoint, float tolerance, const char *label, uint8_t attempt) {
  this->command_phase_ = "WAIT_VERIFY_READ";
  std::vector<uint8_t> read;
  protocol::build_class10_read(0x56, 0x0006, read);
  if (!this->queue_transaction_(std::move(read), [this, expected_mode, verify_run, requested_run,
                                                 expected_setpoint, tolerance, label, attempt](const std::vector<uint8_t> *response) {
        protocol::OperatingState state;
        if (response == nullptr || !protocol::parse_operating_state(response->data(), response->size(), state) ||
            state.mode != expected_mode || (verify_run && state.running != requested_run) ||
            (std::isfinite(expected_setpoint) &&
              (!std::isfinite(state.setpoint) || std::fabs(state.setpoint - expected_setpoint) > tolerance))) {
          if (attempt < VERIFY_MAX_ATTEMPTS) {
            ESP_LOGW(TAG, "%s verification read failed (%u/%u), retrying read", label, attempt, VERIFY_MAX_ATTEMPTS);
            this->verify_control_(expected_mode, verify_run, requested_run, expected_setpoint, tolerance, label, attempt + 1);
            return;
          }
          ESP_LOGW(TAG, "%s verification failed", label);
          if (this->command_status_ != nullptr) this->command_status_->publish_state("Verification failed");
        } else {
          ESP_LOGI(TAG, "%s confirmed by pump readback", label);
          if (this->command_status_ != nullptr) this->command_status_->publish_state("Verified");
        }
        this->control_busy_ = false;
        this->command_phase_ = "IDLE";
      })) {
    ESP_LOGW(TAG, "%s verification read could not be queued", label);
    if (this->command_status_ != nullptr) this->command_status_->publish_state("Readback unavailable");
    this->control_busy_ = false;
    this->command_phase_ = "IDLE";
  }
}

void Alpha2::loop() {
  if (!this->transport_ready())
    return;

  this->expire_partial_frame_();

  if (this->awaiting_response_ && this->tx_queue_.empty()) {
    if (millis() - this->request_sent_ms_ < RESPONSE_TIMEOUT_MS)
      return;
    ESP_LOGW(TAG, "[%s] GENI response timeout", this->parent_->address_str());
    this->finish_transaction_(nullptr);
    this->next_tx_not_before_ms_ = millis() + LATE_RESPONSE_QUIET_MS;
    this->rx_frame_.clear();
    this->rx_buffer_size_ = 0;
    this->expected_rx_length_ = 0;
  }
  if (this->next_tx_not_before_ms_ != 0 &&
      static_cast<int32_t>(millis() - this->next_tx_not_before_ms_) >= 0)
    this->next_tx_not_before_ms_ = 0;
  if (!this->awaiting_response_ && this->tx_queue_.empty() && !this->transactions_.empty() &&
      this->next_tx_not_before_ms_ == 0) {
    auto transaction = std::move(this->transactions_.front());
    this->transactions_.pop_front();
    this->pending_frame_ = transaction.first;
    this->transaction_callback_ = std::move(transaction.second);
    this->tx_queue_.push_back(std::move(transaction.first));
  }
  if (this->tx_queue_.empty() || millis() - this->last_tx_ms_ < TX_SPACING_MS ||
      this->next_tx_not_before_ms_ != 0 || this->awaiting_response_)
    return;

  auto &frame = this->tx_queue_.front();
  if (this->tx_offset_ == 0) {
    ESP_LOGVV(TAG, "[%s] TX: %s", this->parent_->address_str(), format_hex_pretty(frame).c_str());
  }
  const auto status = this->write_frame_fragment_(frame, this->tx_offset_);
  this->last_tx_ms_ = millis();
  if (status != ESP_OK) {
    ESP_LOGW(TAG, "[%s] GENI write failed at offset %zu, status=%d", this->parent_->address_str(), this->tx_offset_,
             status);
    this->tx_offset_ = 0;
    this->tx_queue_.pop_front();
    this->finish_transaction_(nullptr);
    this->next_tx_not_before_ms_ = millis() + LATE_RESPONSE_QUIET_MS;
  } else {
    if (this->tx_offset_ == frame.size()) {
      this->request_sent_ms_ = this->last_tx_ms_;
      this->awaiting_response_ = true;
      this->tx_offset_ = 0;
      this->tx_queue_.pop_front();
    }
  }
}

bool Alpha2::read_object(uint8_t object_id, uint16_t sub_id) {
  std::vector<uint8_t> frame;
  if (!protocol::build_class10_read(object_id, sub_id, frame))
    return false;
  ESP_LOGD(TAG, "[%s] Queueing Class 10 read object=%u sub_id=%u", this->parent_->address_str(), object_id, sub_id);
  return this->queue_transaction_(std::move(frame), nullptr);
}

bool Alpha2::set_enabled(bool enabled) {
  if (this->control_busy_) return false;
  std::vector<uint8_t> read;
  protocol::build_class10_read(0x56, 0x0006, read);
  this->control_busy_ = true;
  this->command_phase_ = "WAIT_PRE_READ";
  if (!this->queue_transaction_(std::move(read), [this, enabled](const std::vector<uint8_t> *response) {
        protocol::OperatingState state;
        if (response == nullptr || !protocol::parse_operating_state(response->data(), response->size(), state)) {
          ESP_LOGW(TAG, "Pump run state pre-read failed");
          if (this->command_status_ != nullptr) this->command_status_->publish_state("Pre-read failed");
          this->control_busy_ = false;
          this->command_phase_ = "IDLE";
          return;
        }
        this->operating_state_ = state;
        this->operating_state_valid_ = true;
        std::vector<uint8_t> frame;
        protocol::build_run_write(state, enabled, frame);
        this->control_busy_ = false;
        if (!this->start_write_(std::move(frame), state.mode, true, enabled, NAN, 0, "Pump run state"))
          ESP_LOGW(TAG, "Pump run-state write could not be queued");
  })) {
    this->control_busy_ = false;
    this->command_phase_ = "IDLE";
    return false;
  }
  return true;
}

bool Alpha2::set_mode(uint8_t raw_mode) {
  // 0x0E selects AUTOADAPT; the app also writes undecoded 56/0015 tuning data.
  // Never synthesize or overwrite those tuning fields.
  std::vector<uint8_t> frame;
  return protocol::build_mode_write(raw_mode, frame) &&
         this->start_write_(std::move(frame), raw_mode, false, false, NAN, 0, "Control mode");
}

bool Alpha2::set_setpoint(protocol::SetpointKind kind, float value) {
  const uint8_t required_mode = kind == protocol::SetpointKind::PRESSURE ? 0x00 :
                                kind == protocol::SetpointKind::FLOW ? 0x08 : 0x02;
  if (!this->operating_state_valid_ || this->operating_state_.mode != required_mode) {
    ESP_LOGW(TAG, "Setpoint rejected: select the corresponding manual mode first");
    return false;
  }
  std::vector<uint8_t> frame;
  if (!protocol::build_setpoint_write(kind, value, this->max_rpm_, frame)) return false;
  float raw = kind == protocol::SetpointKind::PRESSURE ? value * 9803.921875F :
              kind == protocol::SetpointKind::FLOW ? value / 3600.0F : value * this->max_rpm_ / 100.0F;
  float tolerance = kind == protocol::SetpointKind::PRESSURE ? 0.02F * 9803.921875F :
                    kind == protocol::SetpointKind::FLOW ? 0.01F / 3600.0F : 0.002F * this->max_rpm_;
  return this->start_write_(std::move(frame), required_mode, false, false, raw, tolerance, "Setpoint");
}

static void setting_address(protocol::SettingKind kind, uint8_t &object, uint16_t &sub) {
  object = 0;
  sub = 0;
  switch (kind) {
    case protocol::SettingKind::NIGHT_SETBACK: object = 0x5A; sub = 0x0008; break;
    case protocol::SettingKind::AIR_DETECTION: object = 0x58; sub = 0x0263; break;
    case protocol::SettingKind::PANEL_LOCK: object = 0x5E; sub = 0x0002; break;
    case protocol::SettingKind::MIN_FLOW: object = 0x56; sub = 0x0259; break;
    case protocol::SettingKind::MAX_FLOW: object = 0x56; sub = 0x0258; break;
  }
}

void Alpha2::publish_setting_(protocol::SettingKind kind, const protocol::SettingState &state) {
  const auto index = static_cast<size_t>(kind);
  this->setting_states_[index] = state;
  this->setting_valid_[index] = true;
  if (this->setting_switches_[index] != nullptr) this->setting_switches_[index]->publish_state(state.enabled);
  if (this->setting_numbers_[index] != nullptr && std::isfinite(state.value))
    this->setting_numbers_[index]->publish_state(state.value);
}

void Alpha2::read_setting_(protocol::SettingKind kind) {
  uint8_t object;
  uint16_t sub;
  setting_address(kind, object, sub);
  std::vector<uint8_t> read;
  protocol::build_class10_read(object, sub, read);
  this->queue_transaction_(std::move(read), [this, kind](const std::vector<uint8_t> *response) {
    protocol::SettingState state;
    if (response != nullptr && protocol::parse_setting(response->data(), response->size(), kind, state))
      this->publish_setting_(kind, state);
  });
}

bool Alpha2::set_setting(protocol::SettingKind kind, bool enabled, float value) {
  if (!this->transport_ready() || this->control_busy_ ||
      (kind == protocol::SettingKind::MAX_FLOW && !enabled)) {
    ESP_LOGW(TAG, "Setting command refused (busy, disconnected, or unconfirmed maximum-flow disable)");
    return false;
  }
  uint8_t object;
  uint16_t sub;
  setting_address(kind, object, sub);
  std::vector<uint8_t> read;
  protocol::build_class10_read(object, sub, read);
  this->control_busy_ = true;
  this->command_phase_ = "WAIT_PRE_READ";
  if (!this->queue_transaction_(std::move(read), [this, kind, enabled, value](const std::vector<uint8_t> *response) {
    protocol::SettingState before;
    if (response == nullptr || !protocol::parse_setting(response->data(), response->size(), kind, before)) {
      if (this->command_status_) this->command_status_->publish_state("Setting pre-read failed");
      this->control_busy_ = false;
      this->command_phase_ = "IDLE";
      return;
    }
    this->publish_setting_(kind, before);
    const float target = std::isfinite(value) ? value : before.value;
    std::vector<uint8_t> write;
    if (!protocol::build_setting_write(kind, before, enabled, value, write) ||
        !this->queue_transaction_(std::move(write), [this, kind, enabled, target](const std::vector<uint8_t> *ack) {
          uint8_t status = 0xFF;
          if (ack == nullptr || !protocol::parse_write_ack(ack->data(), ack->size(), status) || status != 0) {
            ESP_LOGW(TAG, "Setting write rejected or timed out, status=0x%02X", status);
            if (this->command_status_) this->command_status_->publish_state("Setting write failed");
            this->control_busy_ = false;
            this->command_phase_ = "IDLE";
            return;
          }
          this->verify_setting_(kind, enabled, target);
        })) {
      if (this->command_status_) this->command_status_->publish_state("Setting write unavailable");
      this->control_busy_ = false;
      this->command_phase_ = "IDLE";
    } else {
      this->command_phase_ = "WAIT_WRITE_ACK";
    }
  })) {
    this->control_busy_ = false;
    this->command_phase_ = "IDLE";
    return false;
  }
  return true;
}

void Alpha2::verify_setting_(protocol::SettingKind kind, bool enabled, float target, uint8_t attempt) {
  this->command_phase_ = "WAIT_VERIFY_READ";
  uint8_t object;
  uint16_t sub;
  setting_address(kind, object, sub);
  std::vector<uint8_t> verify;
  protocol::build_class10_read(object, sub, verify);
  if (!this->queue_transaction_(std::move(verify), [this, kind, enabled, target, attempt](const std::vector<uint8_t> *reply) {
    protocol::SettingState actual;
    const bool valid = reply != nullptr && protocol::parse_setting(reply->data(), reply->size(), kind, actual);
    const bool matched = valid && actual.enabled == enabled &&
        ((kind != protocol::SettingKind::MIN_FLOW && kind != protocol::SettingKind::MAX_FLOW) ||
         !std::isfinite(target) || std::fabs(actual.value - target) < 0.01F);
    if (valid) this->publish_setting_(kind, actual);
    if (!matched && attempt < VERIFY_MAX_ATTEMPTS) {
      ESP_LOGW(TAG, "Setting verification failed (%u/%u), retrying read", attempt, VERIFY_MAX_ATTEMPTS);
      this->verify_setting_(kind, enabled, target, attempt + 1);
      return;
    }
    if (this->command_status_) this->command_status_->publish_state(matched ? "Verified" : "Verification failed");
    this->control_busy_ = false;
    this->command_phase_ = "IDLE";
  })) {
    if (this->command_status_) this->command_status_->publish_state("Readback unavailable");
    this->control_busy_ = false;
    this->command_phase_ = "IDLE";
  }
}

bool Alpha2::send_raw_hex(const std::string &hex) {
  std::vector<uint8_t> frame;
  int high_nibble = -1;
  for (char character : hex) {
    if (std::isspace(static_cast<unsigned char>(character)) || character == ':' || character == '-')
      continue;
    uint8_t nibble;
    if (character >= '0' && character <= '9')
      nibble = character - '0';
    else if (character >= 'a' && character <= 'f')
      nibble = character - 'a' + 10;
    else if (character >= 'A' && character <= 'F')
      nibble = character - 'A' + 10;
    else {
      ESP_LOGW(TAG, "Invalid character in raw HEX command");
      return false;
    }

    if (high_nibble < 0) {
      high_nibble = nibble;
    } else {
      frame.push_back(static_cast<uint8_t>((high_nibble << 4) | nibble));
      high_nibble = -1;
    }
  }
  if (high_nibble >= 0 || frame.empty()) {
    ESP_LOGW(TAG, "Raw HEX command must contain a non-empty, even number of digits");
    return false;
  }
  return this->queue_transaction_(std::move(frame), nullptr);
}

void Alpha2::publish_float_(const std::vector<uint8_t> &frame, size_t offset, sensor::Sensor *sensor, float factor) {
  if (sensor == nullptr || offset + sizeof(float) > frame.size() - 2)
    return;
  const float value = protocol::decode_float_be(frame.data() + offset);
  if (!std::isfinite(value)) {
    ESP_LOGW(TAG, "[%s] Ignoring non-finite telemetry value", this->parent_->address_str());
    return;
  }
  sensor->publish_state(value * factor);
}

void Alpha2::handle_frame_(const std::vector<uint8_t> &frame) {
  ESP_LOGVV(TAG, "[%s] RX frame: %s", this->parent_->address_str(), format_hex_pretty(frame).c_str());
  const auto type = protocol::classify_response(this->pending_frame_, frame);
  if (type == protocol::ResponseType::INVALID) {
    this->log_unmatched_(frame, type);
    return;
  }
  const auto pending = protocol::describe_transaction(this->pending_frame_);
  const bool matches = this->awaiting_response_ && this->tx_queue_.empty() &&
      ((pending.transaction == protocol::TransactionType::WRITE &&
        (type == protocol::ResponseType::WRITE_ACK || type == protocol::ResponseType::REJECTED_ACK)) ||
       (pending.transaction == protocol::TransactionType::READ && type == protocol::ResponseType::READ_DATA) ||
       (pending.transaction == protocol::TransactionType::RAW && type != protocol::ResponseType::INVALID));
  if (!matches) this->log_unmatched_(frame, type);
  if (type == protocol::ResponseType::WRITE_ACK || type == protocol::ResponseType::REJECTED_ACK) {
    ESP_LOGI(TAG, "[%s] GENI write ACK status=0x%02X", this->parent_->address_str(), frame[6]);
    if (matches) this->finish_transaction_(&frame);
    return;
  }
  if (matches) this->finish_transaction_(&frame);
  if (frame.size() >= 8 && frame[4] == 0x03) {
    const bool accepted = frame[5] == 0x00;
    ESP_LOGI(TAG, "[%s] Class 3 command %s", this->parent_->address_str(), accepted ? "acknowledged" : "rejected");
    return;
  }
  protocol::OperatingState state;
  if (protocol::parse_operating_state(frame.data(), frame.size(), state)) {
    this->operating_state_ = state;
    this->operating_state_valid_ = true;
    const bool enabled = state.running;
    ESP_LOGI(TAG, "[%s] Pump run state: %s (operation_mode=%u)", this->parent_->address_str(),
              enabled ? "ON" : "OFF", state.mode);
    if (this->pump_switch_ != nullptr)
      this->pump_switch_->publish_state(enabled);
    const char *mode = nullptr;
    if (state.mode == 0x00) mode = "Constant pressure — Manual";
    else if (state.mode == 0x0E) mode = "Constant pressure — AUTOADAPT";
    else if (state.mode == 0x01) mode = "Proportional pressure";
    else if (state.mode == 0x02) mode = "Constant curve";
    else if (state.mode == 0x08) mode = "Constant flow";
    else ESP_LOGW(TAG, "Unknown pump control mode 0x%02X", state.mode);
    if (mode != nullptr && this->mode_select_ != nullptr) this->mode_select_->publish_state(mode);
    if (state.mode == 0x00 || state.mode == 0x0E) {
      if (this->regulation_select_ != nullptr)
        this->regulation_select_->publish_state(state.mode == 0x00 ? "Manual" : "AUTOADAPT");
    }
    // 7F FF FF FF is an observed unavailable-value sentinel, not a setpoint.
    if (std::isfinite(state.setpoint) &&
        !(state.setpoint_bytes[0] == 0x7F && state.setpoint_bytes[1] == 0xFF &&
          state.setpoint_bytes[2] == 0xFF && state.setpoint_bytes[3] == 0xFF)) {
      if (state.mode == 0x00 && this->pressure_number_ != nullptr)
        this->pressure_number_->publish_state(state.setpoint / 9803.921875F);
      else if (state.mode == 0x08 && this->flow_number_ != nullptr)
        this->flow_number_->publish_state(state.setpoint * 3600.0F);
      else if (state.mode == 0x02 && this->curve_number_ != nullptr)
        this->curve_number_->publish_state(state.setpoint / this->max_rpm_ * 100.0F);
    }
    return;
  }
  if (frame.size() < PAYLOAD_OFFSET + 2) return;

  const uint8_t *response_signature = frame.data() + RESPONSE_TYPE_OFFSET;
  if (std::memcmp(response_signature, FLOW_HEAD_RESPONSE_TYPE, RESPONSE_TYPE_LENGTH) == 0) {
    this->publish_float_(frame, PAYLOAD_OFFSET, this->flow_sensor_, 3600.0F);
    this->publish_float_(frame, PAYLOAD_OFFSET + 4, this->head_sensor_, 0.0001F);
  } else if (std::memcmp(response_signature, POWER_RESPONSE_TYPE, RESPONSE_TYPE_LENGTH) == 0) {
    this->publish_float_(frame, PAYLOAD_OFFSET, this->voltage_sensor_, 1.0F);
    this->publish_float_(frame, PAYLOAD_OFFSET + 8, this->current_sensor_, 1.0F);
    this->publish_float_(frame, PAYLOAD_OFFSET + 12, this->power_sensor_, 1.0F);
    this->publish_float_(frame, PAYLOAD_OFFSET + 20, this->speed_sensor_, 1.0F);
  }
}

void Alpha2::log_unmatched_(const std::vector<uint8_t> &frame, protocol::ResponseType type) const {
  const auto pending = protocol::describe_transaction(this->pending_frame_);
  const char *transaction = pending.transaction == protocol::TransactionType::WRITE ? "WRITE" :
                            pending.transaction == protocol::TransactionType::READ ? "READ" :
                            pending.transaction == protocol::TransactionType::RAW ? "RAW" : "NONE";
  const char *classification = type == protocol::ResponseType::INVALID ? "invalid" :
                               type == protocol::ResponseType::WRITE_ACK ? "write ACK" :
                               type == protocol::ResponseType::REJECTED_ACK ? "rejected ACK" :
                               type == protocol::ResponseType::READ_DATA ? "read data" : "unsolicited/unknown";
  ESP_LOGW(TAG, "Unmatched %s RX GENI len=%zu: %s; pending=%s object=0x%02X sub=0x%04X awaiting=%d phase=%s",
           classification, frame.size(), format_hex_pretty(frame).c_str(), transaction, pending.object,
           pending.sub_id, this->awaiting_response_, this->command_phase_);
}

void Alpha2::reset_sensors_() {
  sensor::Sensor *sensors[] = {this->flow_sensor_, this->head_sensor_, this->power_sensor_,
                               this->current_sensor_, this->speed_sensor_, this->voltage_sensor_};
  for (auto *sensor : sensors) {
    if (sensor != nullptr)
      sensor->publish_state(NAN);
  }
}

void Alpha2::on_transport_disconnect_() {
  this->tx_queue_.clear();
  this->transactions_.clear();
  this->transaction_callback_ = nullptr;  // Do not enqueue follow-up work from an old session.
  this->pending_frame_.clear();
  this->operating_state_valid_ = false;
  for (bool &valid : this->setting_valid_) valid = false;
  this->control_busy_ = false;
  this->command_phase_ = "IDLE";
  if (this->command_status_ != nullptr) this->command_status_->publish_state("Disconnected");
  this->tx_offset_ = 0;
  this->awaiting_response_ = false;
  this->next_tx_not_before_ms_ = 0;
  this->reset_sensors_();
}

void Alpha2::on_transport_ready_() {
  this->read_object(0x56, 0x0006);
  for (size_t i = 0; i < 5; i++)
    if (this->setting_switches_[i] || this->setting_numbers_[i])
      this->read_setting_(static_cast<protocol::SettingKind>(i));
}

void Alpha2Switch::write_state(bool state) {
  if (!(this->setting_ ? this->parent_->set_setting(this->kind_, state) : this->parent_->set_enabled(state)))
    ESP_LOGW(TAG, "Pump run-state command was not queued");
}

void Alpha2ModeSelect::control(const std::string &value) {
  uint8_t raw;
  if (value == "Constant pressure — Manual") raw = 0x00;
  else if (value == "Constant pressure — AUTOADAPT") raw = 0x0E;
  else if (value == "Proportional pressure") raw = 0x01;
  else if (value == "Constant curve") raw = 0x02;
  else if (value == "Constant flow") raw = 0x08;
  else return;
  if (!this->parent_->set_mode(raw)) ESP_LOGW(TAG, "Control mode command was not queued");
}

void Alpha2RegulationSelect::control(const std::string &value) {
  if (value == "Manual") {
    if (!this->parent_->set_mode(0x00)) ESP_LOGW(TAG, "Manual regulation command was not queued");
  } else if (value == "AUTOADAPT") {
    if (!this->parent_->set_mode(0x0E)) ESP_LOGW(TAG, "AUTOADAPT command was not queued");
  }
}

void Alpha2SetpointNumber::control(float value) {
  if (!(this->setting_ ? this->parent_->set_setting(this->setting_kind_, true, value) :
                         this->parent_->set_setpoint(this->kind_, value)))
    ESP_LOGW(TAG, "Setpoint command was not queued");
}

void Alpha2::update() {
  if (this->control_busy_) return;
  if (this->node_state != espbt::ClientState::ESTABLISHED) {
    ESP_LOGW(TAG, "[%s] Cannot poll: not connected", this->parent_->address_str());
    return;
  }
  this->read_object(0x56, 0x0006);
  for (size_t i = 0; i < 5; i++)
    if (this->setting_switches_[i] || this->setting_numbers_[i])
      this->read_setting_(static_cast<protocol::SettingKind>(i));
  if (this->flow_sensor_ != nullptr || this->head_sensor_ != nullptr)
    this->read_object(0x5D, 0x0122);
  if (this->power_sensor_ != nullptr || this->current_sensor_ != nullptr || this->speed_sensor_ != nullptr ||
      this->voltage_sensor_ != nullptr)
    this->read_object(0x57, 0x0045);
}

}  // namespace alpha2
}  // namespace esphome

#endif
