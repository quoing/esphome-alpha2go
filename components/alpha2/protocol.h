#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace esphome {
namespace alpha2 {
namespace protocol {

static const uint8_t REQUEST_START = 0x27;
static const uint8_t RESPONSE_START = 0x24;
static const uint8_t PUMP_ADDRESS = 0xE7;
static const uint8_t CONTROLLER_ADDRESS = 0xF8;

uint16_t crc16_geni(const uint8_t *data, size_t length);
bool build_frame(const uint8_t *apdu, size_t apdu_length, std::vector<uint8_t> &frame);
bool build_class10_read(uint8_t object_id, uint16_t sub_id, std::vector<uint8_t> &frame);
bool build_class3_run(bool enabled, std::vector<uint8_t> &frame);
bool validate_frame(const uint8_t *frame, size_t length);
float decode_float_be(const uint8_t *data);
void append_float_be(std::vector<uint8_t> &out, float value);
struct OperatingState {
  bool running;
  uint8_t mode;
  float setpoint;
  uint8_t setpoint_bytes[4];
};
struct SettingState {
  bool enabled;
  float value;
  uint8_t value_bytes[4];
  std::vector<uint8_t> payload;
};
enum class SettingKind : uint8_t { NIGHT_SETBACK, AIR_DETECTION, PANEL_LOCK, MIN_FLOW, MAX_FLOW };
bool parse_setting(const uint8_t *frame, size_t length, SettingKind kind, SettingState &state);
bool build_setting_write(SettingKind kind, const SettingState &state, bool enabled, float value,
                         std::vector<uint8_t> &frame);
bool parse_operating_state(const uint8_t *frame, size_t length, OperatingState &state);
bool parse_write_ack(const uint8_t *frame, size_t length, uint8_t &status);
enum class TransactionType : uint8_t { NONE, WRITE, READ, RAW };
enum class ResponseType : uint8_t { INVALID, WRITE_ACK, REJECTED_ACK, READ_DATA, UNSOLICITED };
struct TransactionMatch {
  TransactionType transaction;
  uint8_t object;
  uint16_t sub_id;
};
TransactionMatch describe_transaction(const std::vector<uint8_t> &request);
ResponseType classify_response(const std::vector<uint8_t> &request, const std::vector<uint8_t> &response);
bool build_run_write(const OperatingState &state, bool running, std::vector<uint8_t> &frame);
bool build_mode_write(uint8_t mode, std::vector<uint8_t> &frame);
enum class SetpointKind : uint8_t { PRESSURE, FLOW, CURVE };
bool build_setpoint_write(SetpointKind kind, float value, float max_rpm, std::vector<uint8_t> &frame);

}  // namespace protocol
}  // namespace alpha2
}  // namespace esphome
