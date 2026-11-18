#include "alpha2_transport.h"

#include "esphome/core/log.h"

#include <algorithm>
#include <cstring>

#ifdef USE_ESP32
namespace esphome {
namespace alpha2_transport {

static const char *const TAG = "alpha2_transport";
static const espbt::ESPBTUUID GENI_SERVICE_UUID = espbt::ESPBTUUID::from_uint16(0xFE5D);
static const espbt::ESPBTUUID GENI_CHARACTERISTIC_UUID =
    espbt::ESPBTUUID::from_raw({static_cast<char>(0xA9), 0x7B, static_cast<char>(0xB8), static_cast<char>(0x85), 0x00,
                                0x1A, 0x28, static_cast<char>(0xAA), 0x2A, 0x43, 0x6E, 0x03,
                                static_cast<char>(0xD1), static_cast<char>(0xFF), static_cast<char>(0x9C),
                                static_cast<char>(0x85)});

esp_err_t Alpha2Transport::write_chunk_(const uint8_t *data, size_t length) {
  if (!this->transport_ready() || data == nullptr || length == 0 || length > ATT_CHUNK_SIZE) return ESP_ERR_INVALID_STATE;
  return esp_ble_gattc_write_char(this->parent_->get_gattc_if(), this->parent_->get_conn_id(), this->geni_handle_,
                                 length, const_cast<uint8_t *>(data), ESP_GATT_WRITE_TYPE_NO_RSP,
                                 ESP_GATT_AUTH_REQ_NONE);
}

esp_err_t Alpha2Transport::write_frame_fragment_(const std::vector<uint8_t> &frame, size_t &offset) {
  if (offset >= frame.size() || frame.size() > MAX_FRAME_SIZE) return ESP_ERR_INVALID_ARG;
  const size_t length = std::min(ATT_CHUNK_SIZE, frame.size() - offset);
  const auto status = this->write_chunk_(frame.data() + offset, length);
  if (status == ESP_OK) offset += length;
  return status;
}

void Alpha2Transport::expire_partial_frame_() {
  if (!this->rx_frame_.empty() && millis() - this->reassembly_started_ms_ >= REASSEMBLY_TIMEOUT_MS) {
    ESP_LOGW(TAG, "Discarding stale GENI fragment (%zu/%zu bytes)", this->rx_frame_.size(), this->expected_rx_length_);
    this->rx_frame_.clear();
    this->rx_buffer_size_ = 0;
    this->expected_rx_length_ = 0;
  }
}

void Alpha2Transport::handle_notification_(const uint8_t *data, size_t length) {
  if (!this->transport_ready()) return;
  for (size_t i = 0; i < length; i++) {
    uint8_t byte = data[i];
    if (this->rx_frame_.empty() && byte != 0x24) continue;
    if (this->rx_frame_.empty()) this->reassembly_started_ms_ = millis();
    this->rx_frame_.push_back(byte);
    this->rx_buffer_size_ = this->rx_frame_.size();
    if (this->rx_frame_.size() == 2) {
      this->expected_rx_length_ = static_cast<size_t>(this->rx_frame_[1]) + 4;
      if (this->rx_frame_[1] < 4 || this->expected_rx_length_ > MAX_FRAME_SIZE) {
        ESP_LOGW(TAG, "Invalid GENI response length %zu", this->expected_rx_length_);
        this->rx_frame_.clear();
        this->rx_buffer_size_ = 0;
        this->expected_rx_length_ = 0;
      }
    }
    if (this->expected_rx_length_ != 0 && this->rx_frame_.size() == this->expected_rx_length_) {
      this->on_geni_frame_(this->rx_frame_);
      this->rx_frame_.clear();
      this->rx_buffer_size_ = 0;
      this->expected_rx_length_ = 0;
    }
  }
}

void Alpha2Transport::log_readiness_() const {
  ESP_LOGI(TAG, "BLE connected: %s; authenticated: %s; services discovered: %s; "
                "GENI notifications: %s; transport READY: %s",
           this->link_connected() ? "yes" : "no", this->authenticated() ? "yes" : "no",
           this->services_ready() ? "yes" : "no", this->notifications_enabled() ? "yes" : "no",
           this->transport_ready() ? "yes" : "no");
}

void Alpha2Transport::update_ready_state_() {
  const bool ready = this->ble_connected_ && this->authenticated_ && this->services_ready_ &&
                     this->notify_registered_ && this->cccd_enabled_;
  if (ready == this->ready_.load()) return;
  this->ready_ = ready;
  if (ready) {
    this->node_state = espbt::ClientState::ESTABLISHED;
    ESP_LOGI(TAG, "ALPHA2 transport READY (authenticated and notifications enabled)");
    this->on_transport_ready_();
  } else {
    this->node_state = espbt::ClientState::CONNECTED;
    this->on_transport_disconnect_();  // Cancel any queued/in-flight GENI work.
  }
}

void Alpha2Transport::gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
  if (event == ESP_GAP_BLE_AUTH_CMPL_EVT &&
      std::memcmp(param->ble_security.auth_cmpl.bd_addr, this->parent_->get_remote_bda(), 6) == 0) {
    if (!this->link_connected()) return;
    this->authenticated_ = param->ble_security.auth_cmpl.success;
    if (this->authenticated_) ESP_LOGI(TAG, "BLE authentication/encryption established");
    else ESP_LOGW(TAG, "BLE authentication/encryption failed, reason=%u", param->ble_security.auth_cmpl.fail_reason);
    this->update_ready_state_();
    this->log_readiness_();
  }
}

void Alpha2Transport::gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                                          esp_ble_gattc_cb_param_t *param) {
  switch (event) {
    case ESP_GATTC_OPEN_EVT:
      if (std::memcmp(param->open.remote_bda, this->parent_->get_remote_bda(), 6) != 0) return;
      if (param->open.status == ESP_GATT_OK) ESP_LOGI(TAG, "BLE connected");
      break;
    case ESP_GATTC_CONNECT_EVT:
      if (std::memcmp(param->connect.remote_bda, this->parent_->get_remote_bda(), 6) != 0) return;
      this->ble_connected_ = true;
      this->ready_ = false;
      this->authenticated_ = false;
      this->services_ready_ = false;
      this->notify_registered_ = false;
      this->cccd_enabled_ = false;
      this->node_state = espbt::ClientState::CONNECTED;
      this->log_readiness_();
      if (esp_ble_set_encryption(param->connect.remote_bda, ESP_BLE_SEC_ENCRYPT) != ESP_OK)
        ESP_LOGW(TAG, "BLE encryption request failed");
      break;
    case ESP_GATTC_DISCONNECT_EVT:
      if (std::memcmp(param->disconnect.remote_bda, this->parent_->get_remote_bda(), 6) != 0) return;
      this->ready_ = false;
      this->ble_connected_ = false;
      this->authenticated_ = false;
      this->services_ready_ = false;
      this->notify_registered_ = false;
      this->cccd_enabled_ = false;
      this->node_state = espbt::ClientState::IDLE;
      this->geni_handle_ = 0;
      this->cccd_handle_ = 0;
      this->rx_frame_.clear();
      this->rx_buffer_size_ = 0;
      this->expected_rx_length_ = 0;
      this->on_transport_disconnect_();
      this->log_readiness_();
      break;
    case ESP_GATTC_SEARCH_CMPL_EVT: {
      if (param->search_cmpl.conn_id != this->parent_->get_conn_id()) break;
      this->ready_ = false;
      this->services_ready_ = false;
      this->notify_registered_ = false;
      this->cccd_enabled_ = false;
      // ESPHome marks the parent established before dispatching this event to nodes.
      // Hold this node back until the CCCD write has actually completed.
      this->node_state = espbt::ClientState::CONNECTED;
      auto *service = this->parent_->get_service(GENI_SERVICE_UUID);
      if (service == nullptr) {
        ESP_LOGE(TAG, "GENI service FE5D not found");
        break;
      }
      ESP_LOGI(TAG, "GENI service FE5D found");
      auto *characteristic = this->parent_->get_characteristic(GENI_SERVICE_UUID, GENI_CHARACTERISTIC_UUID);
      if (characteristic == nullptr) {
        ESP_LOGE(TAG, "GENI characteristic not found");
        break;
      }
      this->geni_handle_ = characteristic->handle;
      ESP_LOGI(TAG, "GENI characteristic found, handle=0x%04X", this->geni_handle());
      auto *descriptor = this->parent_->get_config_descriptor(this->geni_handle_);
      if (descriptor == nullptr) {
        ESP_LOGE(TAG, "GENI notification descriptor not found");
        this->geni_handle_ = 0;
        break;
      }
      this->cccd_handle_ = descriptor->handle;
      this->services_ready_ = true;
      this->log_readiness_();
      ESP_LOGI(TAG, "Registering GENI notifications (CCCD=0x%04X)", this->cccd_handle_);
      const auto status = this->parent_->register_for_notify(this->geni_handle_);
      if (status != ESP_OK) ESP_LOGE(TAG, "Notification registration failed, status=%d", status);
      break;
    }
    case ESP_GATTC_REG_FOR_NOTIFY_EVT:
      if (param->reg_for_notify.handle != this->geni_handle_) break;
      if (param->reg_for_notify.status == ESP_GATT_OK) {
        this->notify_registered_ = true;
        ESP_LOGI(TAG, "GENI notifications registered; awaiting CCCD write");
        this->update_ready_state_();
      } else {
        ESP_LOGE(TAG, "GENI notification registration failed, status=%d", param->reg_for_notify.status);
      }
      break;
    case ESP_GATTC_WRITE_DESCR_EVT:
      if (param->write.handle != this->cccd_handle_ || this->cccd_handle_ == 0) break;
      if (param->write.status == ESP_GATT_OK) {
        this->cccd_enabled_ = true;
        ESP_LOGI(TAG, "GENI notifications enabled");
        this->update_ready_state_();
        this->log_readiness_();
      } else {
        ESP_LOGE(TAG, "GENI CCCD write failed, status=%d", param->write.status);
      }
      break;
    case ESP_GATTC_NOTIFY_EVT:
      if (param->notify.handle == this->geni_handle_)
        this->handle_notification_(param->notify.value, param->notify.value_len);
      break;
    default:
      break;
  }
}

}  // namespace alpha2_transport
}  // namespace esphome
#endif
