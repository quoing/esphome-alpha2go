#pragma once

#include "esphome/components/ble_client/ble_client.h"
#include "esphome/components/esp32_ble_tracker/esp32_ble_tracker.h"

#include <cstddef>
#include <cstdint>
#include <atomic>
#include <vector>

#ifdef USE_ESP32
#include <esp_gattc_api.h>

namespace esphome {
namespace alpha2_transport {

namespace espbt = esphome::esp32_ble_tracker;

class Alpha2Transport : public ble_client::BLEClientNode {
 public:
  bool transport_ready() const { return this->ready_.load(); }
  bool link_connected() const { return this->ble_connected_.load(); }
  bool authenticated() const { return this->authenticated_.load(); }
  bool services_ready() const { return this->services_ready_.load(); }
  bool notifications_registered() const { return this->notify_registered_.load(); }
  bool notifications_enabled() const { return this->cccd_enabled_.load(); }
  uint16_t geni_handle() const { return this->geni_handle_.load(); }
  size_t rx_buffer_size() const { return this->rx_buffer_size_.load(); }
  void gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                           esp_ble_gattc_cb_param_t *param) override;
  void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) override;

 protected:
  static constexpr size_t MAX_FRAME_SIZE = 259;
  static constexpr size_t ATT_CHUNK_SIZE = 20;
  esp_err_t write_chunk_(const uint8_t *data, size_t length);
  esp_err_t write_frame_fragment_(const std::vector<uint8_t> &frame, size_t &offset);
  virtual void on_geni_frame_(const std::vector<uint8_t> &frame) = 0;
  virtual void on_transport_disconnect_() {}
  virtual void on_transport_ready_() {}

  std::atomic<uint16_t> geni_handle_{0};
  uint16_t cccd_handle_{0};
  std::atomic<bool> ble_connected_{false};
  std::atomic<bool> authenticated_{false};
  std::atomic<bool> services_ready_{false};
  std::atomic<bool> notify_registered_{false};
  std::atomic<bool> cccd_enabled_{false};
  std::atomic<bool> ready_{false};
  std::atomic<size_t> rx_buffer_size_{0};
  std::vector<uint8_t> rx_frame_;
  size_t expected_rx_length_{0};
  uint32_t reassembly_started_ms_{0};
  static constexpr uint32_t REASSEMBLY_TIMEOUT_MS = 1000;
  void expire_partial_frame_();

 private:
  void handle_notification_(const uint8_t *data, size_t length);
  void update_ready_state_();
  void log_readiness_() const;
};

}  // namespace alpha2_transport
}  // namespace esphome
#endif
