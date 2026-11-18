#pragma once

#include "esphome/components/alpha2_transport/alpha2_transport.h"
#include "esphome/components/web_server_base/web_server_base.h"
#include "esphome/core/component.h"

#include <mutex>
#include <string>
#include <vector>

#ifdef USE_ESP32
#include <esp_gattc_api.h>

namespace esphome {
namespace alpha2_proxy {

namespace espbt = esphome::esp32_ble_tracker;

class Alpha2Proxy : public alpha2_transport::Alpha2Transport, public Component, public AsyncWebHandler {
 public:
  void setup() override;
  float get_setup_priority() const override { return setup_priority::WIFI - 1.0f; }
  void loop() override;
  bool canHandle(AsyncWebServerRequest *request) const override;
  void handleRequest(AsyncWebServerRequest *request) override;
  void handleBody(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) override;

 protected:
  void on_geni_frame_(const std::vector<uint8_t> &frame) override;
  void on_transport_disconnect_() override;
  void on_transport_ready_() override;
  void fail_(const char *error);

  // HTTP handlers run on the HTTP server task; BLE callbacks run on the ESPHome loop.
  std::mutex mutex_;
  std::string body_;
  struct Pending {
    bool active{false};
    bool sent{false};
    bool complete{false};
    uint32_t started_ms{0};
    std::vector<uint8_t> tx;
    std::vector<uint8_t> rx;
    std::string error;
    httpd_req_t *http_request{nullptr};
  } pending_;

  size_t tx_offset_{0};
  uint32_t last_tx_ms_{0};
  uint32_t response_started_ms_{0};
  uint32_t quiet_until_ms_{0};
  bool awaiting_response_{false};
};

}  // namespace alpha2_proxy
}  // namespace esphome
#endif
