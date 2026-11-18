#include "protocol.h"

#include <cstring>
#include <cmath>
#include <algorithm>

namespace esphome {
namespace alpha2 {
namespace protocol {

uint16_t crc16_geni(const uint8_t *data, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; i++) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (uint8_t bit = 0; bit < 8; bit++)
      crc = (crc & 0x8000) != 0 ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                               : static_cast<uint16_t>(crc << 1);
  }
  return crc ^ 0xFFFF;
}

bool build_frame(const uint8_t *apdu, size_t apdu_length, std::vector<uint8_t> &frame) {
  if ((apdu == nullptr && apdu_length != 0) || apdu_length > 253)
    return false;

  const size_t body_length = apdu_length + 2;
  frame.resize(body_length + 4);
  frame[0] = REQUEST_START;
  frame[1] = static_cast<uint8_t>(body_length);
  frame[2] = PUMP_ADDRESS;
  frame[3] = CONTROLLER_ADDRESS;
  if (apdu_length != 0)
    std::memcpy(frame.data() + 4, apdu, apdu_length);

  const uint16_t crc = crc16_geni(frame.data() + 1, body_length + 1);
  frame[frame.size() - 2] = static_cast<uint8_t>(crc >> 8);
  frame[frame.size() - 1] = static_cast<uint8_t>(crc);
  return true;
}

bool build_class10_read(uint8_t object_id, uint16_t sub_id, std::vector<uint8_t> &frame) {
  const uint8_t apdu[] = {0x0A, 0x03, object_id, static_cast<uint8_t>(sub_id >> 8), static_cast<uint8_t>(sub_id)};
  return build_frame(apdu, sizeof(apdu), frame);
}

bool build_class3_run(bool enabled, std::vector<uint8_t> &frame) {
  const uint8_t apdu[] = {0x03, 0x81, static_cast<uint8_t>(enabled ? 0x06 : 0x05)};
  return build_frame(apdu, sizeof(apdu), frame);
}

bool validate_frame(const uint8_t *frame, size_t length) {
  if (frame == nullptr || length < 7 || static_cast<size_t>(frame[1]) + 4 != length)
    return false;
  if (frame[0] != REQUEST_START && frame[0] != RESPONSE_START)
    return false;

  const uint16_t expected = (static_cast<uint16_t>(frame[length - 2]) << 8) | frame[length - 1];
  return crc16_geni(frame + 1, length - 3) == expected;
}

float decode_float_be(const uint8_t *data) {
  const uint32_t raw = (static_cast<uint32_t>(data[0]) << 24) | (static_cast<uint32_t>(data[1]) << 16) |
                       (static_cast<uint32_t>(data[2]) << 8) | static_cast<uint32_t>(data[3]);
  float value;
  std::memcpy(&value, &raw, sizeof(value));
  return value;
}

void append_float_be(std::vector<uint8_t> &out, float value) {
  uint32_t bits;
  std::memcpy(&bits, &value, sizeof(bits));
  out.push_back(static_cast<uint8_t>(bits >> 24));
  out.push_back(static_cast<uint8_t>(bits >> 16));
  out.push_back(static_cast<uint8_t>(bits >> 8));
  out.push_back(static_cast<uint8_t>(bits));
}

bool parse_operating_state(const uint8_t *frame, size_t length, OperatingState &state) {
  // The 56/0006 response has an eight-byte object header followed by the operating block.
  if (!validate_frame(frame, length) || length < 22 || frame[2] != CONTROLLER_ADDRESS ||
      frame[3] != PUMP_ADDRESS || frame[4] != 0x0A || frame[5] != 0x0E ||
      frame[6] != 0x00 || frame[7] != 0x01 || frame[8] != 0x2F || frame[9] != 0x01 ||
      frame[10] != 0x00 || frame[11] != 0x00 || frame[12] != 0x07 ||
      frame[13] != 0x00 || (frame[14] != 0x00 && frame[14] != 0x01))
    return false;
  state.running = frame[14] == 0x00;
  state.mode = frame[15];
  std::memcpy(state.setpoint_bytes, frame + 16, 4);
  state.setpoint = decode_float_be(state.setpoint_bytes);
  return true;
}

bool parse_write_ack(const uint8_t *frame, size_t length, uint8_t &status) {
  if (!validate_frame(frame, length) || length != 9 || frame[0] != RESPONSE_START ||
      frame[2] != CONTROLLER_ADDRESS ||
      frame[3] != PUMP_ADDRESS || frame[4] != 0x0A || frame[5] != 0x01)
    return false;
  status = frame[6];
  return true;
}

TransactionMatch describe_transaction(const std::vector<uint8_t> &request) {
  if (request.empty()) return {TransactionType::NONE, 0, 0};
  TransactionMatch result{TransactionType::RAW, 0, 0};
  if (request.size() < 9 || !validate_frame(request.data(), request.size()) || request[0] != REQUEST_START ||
      request[2] != PUMP_ADDRESS || request[3] != CONTROLLER_ADDRESS || request[4] != 0x0A)
    return result;
  result.object = request[6];
  result.sub_id = (static_cast<uint16_t>(request[7]) << 8) | request[8];
  if (request[5] == 0x03) result.transaction = TransactionType::READ;
  else if (request[5] == 0x90 || request[5] == 0x9B || request[5] == 0x8A || request[5] == 0x8C)
    result.transaction = TransactionType::WRITE;
  return result;
}

ResponseType classify_response(const std::vector<uint8_t> &request, const std::vector<uint8_t> &response) {
  if (!validate_frame(response.data(), response.size()) || response[0] != RESPONSE_START ||
      response[2] != CONTROLLER_ADDRESS || response[3] != PUMP_ADDRESS)
    return ResponseType::INVALID;
  uint8_t status;
  if (parse_write_ack(response.data(), response.size(), status))
    return status == 0 ? ResponseType::WRITE_ACK : ResponseType::REJECTED_ACK;
  const auto pending = describe_transaction(request);
  if (pending.transaction != TransactionType::READ || response.size() < 9 || response[4] != 0x0A)
    return ResponseType::UNSOLICITED;
  if (pending.object == 0x56 && pending.sub_id == 0x0006) {
    OperatingState state;
    return parse_operating_state(response.data(), response.size(), state) ? ResponseType::READ_DATA :
                                                                  ResponseType::UNSOLICITED;
  }
  SettingKind kind;
  if (pending.object == 0x5A && pending.sub_id == 0x0008) kind = SettingKind::NIGHT_SETBACK;
  else if (pending.object == 0x58 && pending.sub_id == 0x0263) kind = SettingKind::AIR_DETECTION;
  else if (pending.object == 0x5E && pending.sub_id == 0x0002) kind = SettingKind::PANEL_LOCK;
  else if (pending.object == 0x56 && pending.sub_id == 0x0259) kind = SettingKind::MIN_FLOW;
  else if (pending.object == 0x56 && pending.sub_id == 0x0258) kind = SettingKind::MAX_FLOW;
  else {
    static const uint8_t flow_type[] = {0x2B, 0x00, 0x02, 0x35, 0x02, 0x00, 0x00, 0x24};
    static const uint8_t power_type[] = {0x30, 0x00, 0x01, 0x00, 0x03, 0x00, 0x00, 0x29};
    if (response.size() >= 15 &&
        ((pending.object == 0x5D && pending.sub_id == 0x0122 &&
          std::memcmp(response.data() + 5, flow_type, sizeof(flow_type)) == 0) ||
         (pending.object == 0x57 && pending.sub_id == 0x0045 &&
          std::memcmp(response.data() + 5, power_type, sizeof(power_type)) == 0)))
      return ResponseType::READ_DATA;
    // Generic diagnostic reads lack a known object signature; accept only Class 10 data,
    // never an ACK or a Class 3 response. Known polling/controls remain strictly matched.
    if ((pending.object != 0x5D || pending.sub_id != 0x0122) &&
        (pending.object != 0x57 || pending.sub_id != 0x0045) && response.size() >= 10 &&
        response[5] != 0x01)
      return ResponseType::READ_DATA;
    return ResponseType::UNSOLICITED;
  }
  SettingState state;
  return parse_setting(response.data(), response.size(), kind, state) ? ResponseType::READ_DATA :
                                                               ResponseType::UNSOLICITED;
}

bool build_run_write(const OperatingState &state, bool running, std::vector<uint8_t> &frame) {
  const uint8_t apdu[] = {0x0A, 0x90, 0x56, 0x00, 0x06, 0x01, 0x2F, 0x01, 0x00, 0x00,
                          0x07, 0x00, static_cast<uint8_t>(running ? 0 : 1), state.mode,
                          state.setpoint_bytes[0], state.setpoint_bytes[1], state.setpoint_bytes[2], state.setpoint_bytes[3]};
  return build_frame(apdu, sizeof(apdu), frame);
}

bool build_mode_write(uint8_t mode, std::vector<uint8_t> &frame) {
  if (mode != 0x00 && mode != 0x01 && mode != 0x02 && mode != 0x08 && mode != 0x0E) return false;
  const uint8_t apdu[] = {0x0A, 0x90, 0x56, 0x00, 0x0A, 0x01, 0x2F, 0x01, 0x00, 0x00,
                          0x07, 0x00, 0x06, mode, 0x7F, 0xFF, 0xFF, 0xFF};
  return build_frame(apdu, sizeof(apdu), frame);
}

bool build_setpoint_write(SetpointKind kind, float value, float max_rpm, std::vector<uint8_t> &frame) {
  if (!std::isfinite(value) || value <= 0) return false;
  uint16_t sub_id;
  uint8_t unit;
  float internal;
  const uint8_t *trailer;
  static const uint8_t pressure_tail[] = {0x3F, 0x19, 0x99, 0x9A, 0x3F, 0x99, 0x99, 0x9A, 0, 0, 0, 0};
  static const uint8_t flow_tail[] = {0x3F, 0x19, 0x99, 0x9A, 0x3F, 0xCC, 0xCC, 0xCD,
                                      0x3E, 0xCC, 0xCC, 0xCD};
  static const uint8_t curve_tail[] = {0x3F, 0x80, 0, 0, 0x3F, 0x80, 0, 0, 0x3F, 0x80, 0, 0};
  switch (kind) {
    case SetpointKind::PRESSURE:
      sub_id = 0x0010; unit = 0x04; internal = value * 9803.921875F; trailer = pressure_tail; break;
    case SetpointKind::FLOW:
      sub_id = 0x0028; unit = 0x06; internal = value / 3600.0F; trailer = flow_tail; break;
    case SetpointKind::CURVE:
      if (value > 100 || !std::isfinite(max_rpm) || max_rpm <= 0) return false;
      sub_id = 0x000E; unit = 0xFF; internal = value / 100.0F * max_rpm; trailer = curve_tail; break;
    default: return false;
  }
  if (!std::isfinite(internal)) return false;
  std::vector<uint8_t> apdu = {0x0A, 0x9B, 0x56, static_cast<uint8_t>(sub_id >> 8),
                                static_cast<uint8_t>(sub_id), 0x01, 0x2E, 0x01, 0x00, 0x00,
                                0x12, 0x00, unit};
  append_float_be(apdu, internal);
  apdu.insert(apdu.end(), trailer, trailer + 12);
  return build_frame(apdu.data(), apdu.size(), frame);
}

static bool setting_layout(SettingKind kind, uint8_t &object, uint16_t &sub, uint8_t &opcode,
                           uint8_t &response_opcode, size_t &payload_size, size_t &enabled_offset) {
  switch (kind) {
    case SettingKind::NIGHT_SETBACK:
      object = 0x5A; sub = 0x0008; opcode = 0x8A; response_opcode = 0x08; payload_size = 7; enabled_offset = 6; break;
    case SettingKind::AIR_DETECTION:
      object = 0x58; sub = 0x0263; opcode = 0x90; response_opcode = 0x0E; payload_size = 13; enabled_offset = 6; break;
    case SettingKind::PANEL_LOCK:
      object = 0x5E; sub = 0x0002; opcode = 0x8C; response_opcode = 0x0A; payload_size = 9; enabled_offset = 6; break;
    case SettingKind::MIN_FLOW:
    case SettingKind::MAX_FLOW:
      object = 0x56; sub = kind == SettingKind::MIN_FLOW ? 0x0259 : 0x0258;
      opcode = 0x9B; response_opcode = 0x19; payload_size = 24; enabled_offset = 7; break;
    default: return false;
  }
  return true;
}

bool parse_setting(const uint8_t *frame, size_t length, SettingKind kind, SettingState &state) {
  uint8_t object, opcode, response_opcode;
  uint16_t sub;
  size_t payload_size, enabled_offset;
  if (!setting_layout(kind, object, sub, opcode, response_opcode, payload_size, enabled_offset) ||
      !validate_frame(frame, length) || frame[0] != RESPONSE_START || frame[2] != CONTROLLER_ADDRESS ||
      frame[3] != PUMP_ADDRESS || frame[4] != 0x0A || frame[5] != response_opcode ||
      length != payload_size + 9 || frame[6] != 0x00)
    return false;
  // The first response byte is an envelope byte, not part of the write payload.
  const uint8_t *payload = frame + 7;
  if (payload[enabled_offset] > 1) return false;
  if (kind == SettingKind::NIGHT_SETBACK &&
      (std::memcmp(payload, "\x01\x4D\x01\x00\x00\x01", 6) != 0)) return false;
  if (kind == SettingKind::AIR_DETECTION &&
      (std::memcmp(payload, "\x03\xCA\x01\x00\x00\x07", 6) != 0)) return false;
  if (kind == SettingKind::PANEL_LOCK &&
      (std::memcmp(payload, "\x01\x0D\x01\x00\x00\x03", 6) != 0)) return false;
  if ((kind == SettingKind::MIN_FLOW || kind == SettingKind::MAX_FLOW) &&
      (std::memcmp(payload, "\x03\x7F\x01\x00\x00\x12", 6) != 0 ||
       payload[6] != (kind == SettingKind::MIN_FLOW ? 2 : 1))) return false;
  state.payload.assign(payload, payload + payload_size);
  state.enabled = payload[enabled_offset] == 1;
  state.value = NAN;
  if (kind == SettingKind::MIN_FLOW || kind == SettingKind::MAX_FLOW) {
    std::memcpy(state.value_bytes, payload + 8, 4);
    state.value = decode_float_be(state.value_bytes) * 3600.0F;
  }
  return true;
}

bool build_setting_write(SettingKind kind, const SettingState &state, bool enabled, float value,
                         std::vector<uint8_t> &frame) {
  uint8_t object, opcode, response_opcode;
  uint16_t sub;
  size_t payload_size, enabled_offset;
  if (!setting_layout(kind, object, sub, opcode, response_opcode, payload_size, enabled_offset) ||
      state.payload.size() != payload_size) return false;
  if ((kind == SettingKind::MIN_FLOW || kind == SettingKind::MAX_FLOW) &&
      state.payload[6] != (kind == SettingKind::MIN_FLOW ? 2 : 1)) return false;
  std::vector<uint8_t> apdu = {0x0A, opcode, object, static_cast<uint8_t>(sub >> 8), static_cast<uint8_t>(sub)};
  apdu.insert(apdu.end(), state.payload.begin(), state.payload.end());
  apdu[5 + enabled_offset] = enabled ? 1 : 0;
  if (kind == SettingKind::MIN_FLOW || kind == SettingKind::MAX_FLOW) {
    if (std::isfinite(value)) {
      if (value <= 0) return false;
      std::vector<uint8_t> encoded;
      append_float_be(encoded, value / 3600.0F);
      std::copy(encoded.begin(), encoded.end(), apdu.begin() + 13);
    } else if (!std::isnan(value)) return false;
  }
  return build_frame(apdu.data(), apdu.size(), frame);
}

}  // namespace protocol
}  // namespace alpha2
}  // namespace esphome
