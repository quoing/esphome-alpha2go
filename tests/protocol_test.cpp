#include "../components/alpha2/protocol.h"

#include <cassert>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <vector>

using esphome::alpha2::protocol::build_class10_read;
using esphome::alpha2::protocol::build_class3_run;
using esphome::alpha2::protocol::build_frame;
using esphome::alpha2::protocol::decode_float_be;
using esphome::alpha2::protocol::validate_frame;

static void assert_frame(uint8_t object_id, uint16_t sub_id, const std::vector<uint8_t> &expected) {
  std::vector<uint8_t> actual;
  assert(build_class10_read(object_id, sub_id, actual));
  assert(actual == expected);
  assert(validate_frame(actual.data(), actual.size()));
}

int main() {
  assert_frame(0x5D, 0x0122, {0x27, 0x07, 0xE7, 0xF8, 0x0A, 0x03, 0x5D, 0x01, 0x22, 0x62, 0x7C});
  assert_frame(0x57, 0x0045, {0x27, 0x07, 0xE7, 0xF8, 0x0A, 0x03, 0x57, 0x00, 0x45, 0x8A, 0xCD});

  std::vector<uint8_t> start_frame;
  std::vector<uint8_t> stop_frame;
  assert(build_class3_run(true, start_frame));
  assert(build_class3_run(false, stop_frame));
  assert(start_frame.size() == 9 && start_frame[4] == 0x03 && start_frame[5] == 0x81 && start_frame[6] == 0x06);
  assert(stop_frame.size() == 9 && stop_frame[4] == 0x03 && stop_frame[5] == 0x81 && stop_frame[6] == 0x05);
  assert(validate_frame(start_frame.data(), start_frame.size()));
  assert(validate_frame(stop_frame.data(), stop_frame.size()));

  std::vector<uint8_t> object_86;
  assert(build_class10_read(0x56, 15, object_86));
  assert(object_86[6] == 0x56 && object_86[7] == 0x00 && object_86[8] == 0x0F);
  object_86[8] ^= 0x01;
  assert(!validate_frame(object_86.data(), object_86.size()));

  const uint8_t encoded_float[] = {0x3F, 0xC0, 0x00, 0x00};
  assert(std::fabs(decode_float_be(encoded_float) - 1.5F) < 0.0001F);
  using namespace esphome::alpha2::protocol;
  std::vector<uint8_t> encoded;
  append_float_be(encoded, 3.0F * 9803.921875F);
  assert((encoded == std::vector<uint8_t>{0x46, 0xE5, 0xC7, 0x88}));
  std::vector<uint8_t> response;
  const uint8_t state_apdu[] = {0x0A, 0x0E, 0x00, 0x01, 0x2F, 0x01, 0x00, 0x00,
                                 0x07, 0x00, 0x00, 0x00, 0x46, 0xE5, 0xC7, 0x88};
  assert(build_frame(state_apdu, sizeof(state_apdu), response));
  response[0] = RESPONSE_START;
  response[2] = CONTROLLER_ADDRESS;
  response[3] = PUMP_ADDRESS;
  auto crc = crc16_geni(response.data() + 1, response.size() - 3);
  response[response.size() - 2] = crc >> 8;
  response[response.size() - 1] = crc;
  OperatingState state;
  assert(parse_operating_state(response.data(), response.size(), state));
  assert(state.running && state.mode == 0 && std::fabs(state.setpoint - 29411.765625F) < 0.01F);
  std::vector<uint8_t> write;
  assert(build_run_write(state, false, write));
  assert(validate_frame(write.data(), write.size()));
  assert(write[15] == 0 && write[16] == 1 && write[18] == 0x46 && write[21] == 0x88);
  response[14] = 1;
  crc = crc16_geni(response.data() + 1, response.size() - 3);
  response[response.size() - 2] = crc >> 8;
  response[response.size() - 1] = crc;
  assert(parse_operating_state(response.data(), response.size(), state) && !state.running);
  const std::vector<uint8_t> captured_stopped_state{
      0x24, 0x12, 0xF8, 0xE7, 0x0A, 0x0E, 0x00, 0x01, 0x2F, 0x01, 0x00,
      0x00, 0x07, 0x00, 0x01, 0x00, 0x47, 0x37, 0xD2, 0xD3, 0xC9, 0x7F};
  assert(parse_operating_state(captured_stopped_state.data(), captured_stopped_state.size(), state));
  assert(!state.running && state.mode == 0);
  std::vector<uint8_t> run_read;
  assert(build_class10_read(0x56, 0x0006, run_read));
  assert(classify_response(run_read, captured_stopped_state) == ResponseType::READ_DATA);
  assert(classify_response(run_read, response) == ResponseType::READ_DATA);
  assert(build_mode_write(0x08, write) && write[17] == 0x08);
  assert(!build_mode_write(0xFF, write));
  assert(build_setpoint_write(SetpointKind::PRESSURE, 3.0F, 5837.0F, write));
  assert(validate_frame(write.data(), write.size()));
  assert(write[5] == 0x9B && write[8] == 0x10 && write[17] == 0x46 && write[20] == 0x88);
  assert(build_setpoint_write(SetpointKind::FLOW, 2.0F, 5837.0F, write));
  assert(write[8] == 0x28 && write[17] == 0x3A && write[20] == 0xB4);
  assert(build_setpoint_write(SetpointKind::CURVE, 75.1F, 5837.0F, write));
  assert(write[8] == 0x0E && write[16] == 0xFF);
  assert(!build_setpoint_write(SetpointKind::CURVE, 110.0F, 5837.0F, write));
  const uint8_t ack_apdu[] = {0x0A, 0x01, 0x04};
  assert(build_frame(ack_apdu, sizeof(ack_apdu), response));
  response[0] = RESPONSE_START;
  response[2] = CONTROLLER_ADDRESS;
  response[3] = PUMP_ADDRESS;
  crc = crc16_geni(response.data() + 1, response.size() - 3);
  response[7] = crc >> 8;
  response[8] = crc;
  uint8_t status;
  assert(parse_write_ack(response.data(), response.size(), status) && status == 4);
  assert((response == std::vector<uint8_t>{0x24, 0x05, 0xF8, 0xE7, 0x0A, 0x01, 0x04, 0xEE, 0x26}));
  assert(classify_response(write, response) == ResponseType::REJECTED_ACK);
  assert(classify_response(run_read, response) == ResponseType::REJECTED_ACK);
  const std::vector<uint8_t> successful_ack{0x24, 0x05, 0xF8, 0xE7, 0x0A, 0x01, 0x00, 0xAE, 0xA2};
  assert(parse_write_ack(successful_ack.data(), successful_ack.size(), status) && status == 0);
  assert(classify_response(write, successful_ack) == ResponseType::WRITE_ACK);
  assert(classify_response(run_read, successful_ack) == ResponseType::WRITE_ACK);
  auto corrupted_ack = successful_ack;
  corrupted_ack[6] = 4;
  assert(classify_response(write, corrupted_ack) == ResponseType::INVALID);
  // The ACK closes the write; the next request expects a data readback, not another ACK.
  assert(build_run_write(state, false, write));
  assert(describe_transaction(write).transaction == TransactionType::WRITE);
  assert(describe_transaction(write).object == 0x56 && describe_transaction(write).sub_id == 0x0006);
  assert(describe_transaction(run_read).transaction == TransactionType::READ);
  assert(classify_response(write, successful_ack) == ResponseType::WRITE_ACK);
  assert(classify_response(run_read, successful_ack) != ResponseType::READ_DATA);
  assert(build_frame(state_apdu, sizeof(state_apdu), response));
  response[0] = RESPONSE_START; response[2] = CONTROLLER_ADDRESS; response[3] = PUMP_ADDRESS;
  crc = crc16_geni(response.data() + 1, response.size() - 3);
  response[response.size() - 2] = crc >> 8; response[response.size() - 1] = crc;
  assert(classify_response(run_read, response) == ResponseType::READ_DATA);
  std::vector<uint8_t> flow_read;
  assert(build_class10_read(0x5D, 0x0122, flow_read));
  assert(classify_response(flow_read, response) == ResponseType::UNSOLICITED);
  assert(classify_response(run_read, response) == ResponseType::READ_DATA);
  assert(describe_transaction({}).transaction == TransactionType::NONE);
  std::vector<uint8_t> unknown_read;
  assert(build_class10_read(0x56, 0x000F, unknown_read));
  assert(classify_response(unknown_read, response) == ResponseType::READ_DATA);

  const uint8_t night_bytes[] = {0x24, 0x0C, 0xF8, 0xE7, 0x0A, 0x08, 0x00,
                                 0x01, 0x4D, 0x01, 0x00, 0x00, 0x01, 0x00, 0x1A, 0x69};
  SettingState setting{};
  assert(parse_setting(night_bytes, sizeof(night_bytes), SettingKind::NIGHT_SETBACK, setting));
  assert(!setting.enabled);
  assert(build_setting_write(SettingKind::NIGHT_SETBACK, setting, true, NAN, write));
  assert((write == std::vector<uint8_t>{0x27, 0x0E, 0xE7, 0xF8, 0x0A, 0x8A, 0x5A, 0x00, 0x08,
                                        0x01, 0x4D, 0x01, 0x00, 0x00, 0x01, 0x01, 0xE1, 0xDD}));
  const uint8_t air_bytes[] = {0x24, 0x12, 0xF8, 0xE7, 0x0A, 0x0E, 0x00, 0x03, 0xCA, 0x01, 0x00,
                               0x00, 0x07, 0x01, 0x39, 0x04, 0x1E, 0xDE, 0x00, 0x78, 0x88, 0xD8};
  assert(parse_setting(air_bytes, sizeof(air_bytes), SettingKind::AIR_DETECTION, setting));
  assert(build_setting_write(SettingKind::AIR_DETECTION, setting, false, NAN, write));
  assert((write == std::vector<uint8_t>{0x27, 0x14, 0xE7, 0xF8, 0x0A, 0x90, 0x58, 0x02, 0x63,
                                        0x03, 0xCA, 0x01, 0x00, 0x00, 0x07, 0x00, 0x39, 0x04,
                                        0x1E, 0xDE, 0x00, 0x78, 0x70, 0x5D}));
  const uint8_t panel_bytes[] = {0x24, 0x0E, 0xF8, 0xE7, 0x0A, 0x0A, 0x00, 0x01, 0x0D,
                                 0x01, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x05, 0x15};
  assert(parse_setting(panel_bytes, sizeof(panel_bytes), SettingKind::PANEL_LOCK, setting));
  assert(build_setting_write(SettingKind::PANEL_LOCK, setting, true, NAN, write));
  assert((write == std::vector<uint8_t>{0x27, 0x10, 0xE7, 0xF8, 0x0A, 0x8C, 0x5E, 0x00, 0x02,
                                        0x01, 0x0D, 0x01, 0x00, 0x00, 0x03, 0x01, 0x00, 0x00, 0x2D, 0x38}));
  // A readback envelope plus the captured minimum-flow structured block.
  const uint8_t min_payload[] = {0x03, 0x7F, 0x01, 0x00, 0x00, 0x12, 0x02, 0x01,
                                 0x39, 0xDA, 0x74, 0x0E, 0x3F, 0x00, 0x00, 0x00,
                                 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  std::vector<uint8_t> min_reply_apdu = {0x0A, 0x19, 0x00};
  min_reply_apdu.insert(min_reply_apdu.end(), min_payload, min_payload + sizeof(min_payload));
  assert(build_frame(min_reply_apdu.data(), min_reply_apdu.size(), response));
  response[0] = RESPONSE_START; response[2] = CONTROLLER_ADDRESS; response[3] = PUMP_ADDRESS;
  crc = crc16_geni(response.data() + 1, response.size() - 3);
  response[response.size() - 2] = crc >> 8; response[response.size() - 1] = crc;
  assert(parse_setting(response.data(), response.size(), SettingKind::MIN_FLOW, setting));
  assert(setting.enabled && std::fabs(setting.value - 1.5F) < 0.01F);
  assert(build_setting_write(SettingKind::MIN_FLOW, setting, false, setting.value, write));
  assert((write == std::vector<uint8_t>{0x27, 0x1F, 0xE7, 0xF8, 0x0A, 0x9B, 0x56, 0x02, 0x59,
                                        0x03, 0x7F, 0x01, 0x00, 0x00, 0x12, 0x02, 0x00,
                                        0x39, 0xDA, 0x74, 0x0E, 0x3F, 0x00, 0x00, 0x00,
                                        0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xD0, 0x26}));
  response[7] = 0xFF;
  assert(!parse_setting(response.data(), response.size(), SettingKind::MIN_FLOW, setting));
  response[7] = 0x03;
  response[14] = 2;
  crc = crc16_geni(response.data() + 1, response.size() - 3);
  response[response.size() - 2] = crc >> 8; response[response.size() - 1] = crc;
  assert(!parse_setting(response.data(), response.size(), SettingKind::MIN_FLOW, setting));
  assert(!build_setting_write(SettingKind::MAX_FLOW, setting, false, 1.0F, write));
  min_reply_apdu[9] = 0x01;  // Structured maximum-flow selector.
  min_reply_apdu[11] = 0x3A; min_reply_apdu[12] = 0x36;
  min_reply_apdu[13] = 0x0B; min_reply_apdu[14] = 0x61;
  assert(build_frame(min_reply_apdu.data(), min_reply_apdu.size(), response));
  response[0] = RESPONSE_START; response[2] = CONTROLLER_ADDRESS; response[3] = PUMP_ADDRESS;
  crc = crc16_geni(response.data() + 1, response.size() - 3);
  response[response.size() - 2] = crc >> 8; response[response.size() - 1] = crc;
  assert(parse_setting(response.data(), response.size(), SettingKind::MAX_FLOW, setting));
  assert(std::fabs(setting.value - 2.5F) < 0.01F);
  assert(build_setting_write(SettingKind::MAX_FLOW, setting, true, setting.value, write));
  assert((write == std::vector<uint8_t>{0x27, 0x1F, 0xE7, 0xF8, 0x0A, 0x9B, 0x56, 0x02, 0x58,
                                        0x03, 0x7F, 0x01, 0x00, 0x00, 0x12, 0x01, 0x01,
                                        0x3A, 0x36, 0x0B, 0x61, 0x3F, 0x00, 0x00, 0x00,
                                        0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x6F, 0xB7}));

  std::vector<uint8_t> maximum_apdu(253, 0x00);
  std::vector<uint8_t> maximum_frame;
  assert(build_frame(maximum_apdu.data(), maximum_apdu.size(), maximum_frame));
  assert(maximum_frame.size() == 259);
  assert(validate_frame(maximum_frame.data(), maximum_frame.size()));
  maximum_apdu.push_back(0x00);
  assert(!build_frame(maximum_apdu.data(), maximum_apdu.size(), maximum_frame));

  std::cout << "protocol tests passed\n";
  return 0;
}
