#include "alpha2_proxy.h"

#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

#include <algorithm>
#include <cctype>

#ifdef USE_ESP32
namespace esphome {
namespace alpha2_proxy {

static const char *const TAG = "alpha2_proxy";
static const uint32_t TX_SPACING_MS = 25;
static const uint32_t RESPONSE_TIMEOUT_MS = 2000;
static const uint32_t LATE_RESPONSE_QUIET_MS = 1100;

static std::string hex_string(const std::vector<uint8_t> &bytes) {
  static const char digits[] = "0123456789ABCDEF";
  std::string result;
  for (uint8_t byte : bytes) {
    if (!result.empty()) result += ' ';
    result += digits[byte >> 4];
    result += digits[byte & 15];
  }
  return result;
}

static void reply(AsyncWebServerRequest *request, int status, const std::string &json) {
  request->send(request->beginResponse(status, "application/json", json));
}

void Alpha2Proxy::setup() {
  auto *server = web_server_base::global_web_server_base;
  server->add_handler(this);
  server->init();
}

bool Alpha2Proxy::canHandle(AsyncWebServerRequest *request) const {
  char url[AsyncWebServerRequest::URL_BUF_SIZE];
  return request->method() == HTTP_POST && request->url_to(url) == "/geni";
}

void Alpha2Proxy::handleBody(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
  std::lock_guard<std::mutex> lock(this->mutex_);
  if (index == 0) this->body_.clear();
  if (total <= MAX_FRAME_SIZE * 3) this->body_.append(reinterpret_cast<char *>(data), len);
}

void Alpha2Proxy::handleRequest(AsyncWebServerRequest *request) {
  std::lock_guard<std::mutex> lock(this->mutex_);
  ESP_LOGI(TAG,
            "PROXY HTTP request: connected=%d authenticated=%d services_ready=%d transport_ready=%d "
            "notify_registered=%d notify_enabled=%d "
           "characteristic_handle=0x%04X transaction_pending=%d rx_buffer_size=%zu",
            this->link_connected(), this->authenticated(), this->services_ready(), this->transport_ready(),
            this->notifications_registered(),
           this->notifications_enabled(), this->geni_handle(), this->pending_.active || this->pending_.complete,
           this->rx_buffer_size());
  if (this->pending_.active || this->pending_.complete) {
    ESP_LOGW(TAG, "PROXY reject: transaction already pending");
    reply(request, 409, "{\"ok\":false,\"error\":\"transaction_pending\"}");
    return;
  }
  if (!this->link_connected()) {
    ESP_LOGW(TAG, "PROXY reject: BLE not connected");
    reply(request, 503, "{\"ok\":false,\"error\":\"ble_not_connected\"}");
    return;
  }
  if (!this->transport_ready()) {
    ESP_LOGW(TAG, "PROXY reject: transport not ready (GENI notifications not enabled)");
    reply(request, 503, "{\"ok\":false,\"error\":\"transport_not_ready\"}");
    return;
  }
  if (request->contentLength() > MAX_FRAME_SIZE * 3) {
    reply(request, 400, "{\"ok\":false,\"error\":\"invalid_hex\"}");
    return;
  }
  std::vector<uint8_t> tx;
  int high = -1;
  for (char c : this->body_) {
    if (std::isspace(static_cast<unsigned char>(c))) continue;
    int digit = c >= '0' && c <= '9' ? c - '0' : c >= 'A' && c <= 'F' ? c - 'A' + 10
                : c >= 'a' && c <= 'f' ? c - 'a' + 10 : -1;
    if (digit < 0 || tx.size() >= MAX_FRAME_SIZE) {
      reply(request, 400, "{\"ok\":false,\"error\":\"invalid_hex\"}");
      return;
    }
    if (high < 0) high = digit;
    else {
      tx.push_back(static_cast<uint8_t>((high << 4) | digit));
      high = -1;
    }
  }
  if (high >= 0 || tx.empty()) {
    reply(request, 400, "{\"ok\":false,\"error\":\"invalid_hex\"}");
    return;
  }
  httpd_req_t *async_request = nullptr;
  if (httpd_req_async_handler_begin(*request, &async_request) != ESP_OK) {
    reply(request, 500, "{\"ok\":false,\"error\":\"http_failed\"}");
    return;
  }
  this->pending_ = Pending{};
  this->pending_.active = true;
  this->pending_.tx = std::move(tx);
  this->pending_.http_request = async_request;
  ESP_LOGVV(TAG, "HTTP RX: %s", format_hex_pretty(this->pending_.tx).c_str());
}

// Called with mutex_ held on the main loop or from the BLE event handler.
void Alpha2Proxy::fail_(const char *error) {
  if (!this->pending_.active) return;
  this->pending_.error = error;
  this->pending_.active = false;
  this->pending_.complete = true;
  this->awaiting_response_ = false;
  this->tx_offset_ = 0;
  this->rx_frame_.clear();
  this->expected_rx_length_ = 0;
  this->quiet_until_ms_ = millis() + LATE_RESPONSE_QUIET_MS;
}

void Alpha2Proxy::loop() {
  httpd_req_t *request = nullptr;
  std::string response;
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    const uint32_t now = millis();
    if (this->quiet_until_ms_ != 0 && static_cast<int32_t>(now - this->quiet_until_ms_) >= 0)
      this->quiet_until_ms_ = 0;
    this->expire_partial_frame_();
    if (this->pending_.active && !this->transport_ready())
      this->fail_("pump_not_connected");
    if (this->pending_.active && !this->pending_.sent && this->quiet_until_ms_ == 0 &&
        this->transport_ready() && now - this->last_tx_ms_ >= TX_SPACING_MS) {
      auto &tx = this->pending_.tx;
      const auto status = this->write_frame_fragment_(tx, this->tx_offset_);
      this->last_tx_ms_ = now;
      if (status != ESP_OK) {
        ESP_LOGW(TAG, "BLE write failed at offset %zu, status=%d", this->tx_offset_, status);
        this->fail_("write_failed");
      } else {
        if (this->tx_offset_ == tx.size()) {
          this->tx_offset_ = 0;
          this->pending_.sent = true;
          this->awaiting_response_ = true;
          this->response_started_ms_ = now;
        }
      }
    }
    if (this->pending_.active && this->awaiting_response_ &&
        now - this->response_started_ms_ >= RESPONSE_TIMEOUT_MS)
      this->fail_("timeout");
    if (this->pending_.complete) {
      request = this->pending_.http_request;
      if (this->pending_.error.empty()) {
        response = "{\"ok\":true,\"tx\":\"" + hex_string(this->pending_.tx) +
                   "\",\"rx\":\"" + hex_string(this->pending_.rx) + "\"}";
      } else {
        response = "{\"ok\":false,\"error\":\"" + this->pending_.error + "\"}";
      }
      this->pending_.complete = false;
      this->pending_.http_request = nullptr;
    }
  }
  if (request != nullptr) {
    httpd_resp_set_type(request, "application/json");
    if (httpd_resp_send(request, response.c_str(), response.size()) != ESP_OK)
      ESP_LOGW(TAG, "HTTP response send failed");
    httpd_req_async_handler_complete(request);
  }
}

void Alpha2Proxy::on_geni_frame_(const std::vector<uint8_t> &frame) {
  std::lock_guard<std::mutex> lock(this->mutex_);
  if (!this->pending_.active || !this->awaiting_response_) return;
  this->pending_.rx = frame;
  ESP_LOGVV(TAG, "BLE RX: %s", format_hex_pretty(frame).c_str());
  this->pending_.active = false;
  this->pending_.complete = true;
  this->awaiting_response_ = false;
}

void Alpha2Proxy::on_transport_disconnect_() {
  std::lock_guard<std::mutex> lock(this->mutex_);
  this->fail_("pump_not_connected");
}

void Alpha2Proxy::on_transport_ready_() { ESP_LOGI(TAG, "HTTP /geni ready"); }

}  // namespace alpha2_proxy
}  // namespace esphome
#endif
