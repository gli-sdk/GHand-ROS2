// Copyright (c) 2026 GHand ROS2 Maintainers
// SPDX-License-Identifier: Apache-2.0
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>
#include "ghand_hardware_interface/communication/modbus_codec.hpp"
#include "ghand_hardware_interface/communication/product_config.hpp"

int main() {
  using namespace ghand_comm;
  const auto* l1 = find_product_config("ghandlite1");
  const auto* g5 = find_product_config("ghand5");
  assert(l1 && g5 && !find_product_config("invalid"));
  assert(l1->joints.size() == 11 && l1->active_joints.size() == 6);
  assert(l1->rxpdo_size() == 50 && g5->rxpdo_size() == 158);
  const auto addresses = l1->holding_registers();
  assert(addresses.at(JointId::THUMB_TMC_FE) == 0x0261);
  assert(addresses.at(JointId::LITTLE_MCP) == 0x0275);
  JointCommand command;
  command.id = JointId::THUMB_TMC_FE;
  command.position = 30;
  command.loop_type = JointLoopType::CLOSED_LOOP;
  auto blocks = encode_joint_command_blocks({command}, addresses, 4);
  assert(blocks.size() == 6 && blocks.at(0x0261).size() == 4);
  assert(blocks.at(0x0261)[0] == 0x0100 && blocks.at(0x0261)[1] == 300);
  assert(blocks.at(0x0265)[0] == 0x0200);
  auto pdo = encode_ethercat_command(HandControlMode::POSITION, l1->active_joints,
                                   {{command.id, command}}, 4);
  assert(pdo.size() == 50 && pdo[0] == 0 && pdo[1] == 1);
  assert(pdo[2] == 0 && pdo[3] == 1 && pdo[4] == 0x2c && pdo[5] == 1);
  uint16_t feedback[55] = {};
  feedback[2] = 123;
  feedback[52] = static_cast<uint16_t>(-321);
  auto states = parse_joints(feedback, 55, l1->joints, true);
  assert(states.size() == 18);
  assert(std::abs(states[1].position - 12.3f) < 0.001f);
  assert(std::abs(states[17].position + 32.1f) < 0.001f);
  uint16_t header[11] = {};
  header[4] = 0x2211; header[7] = 0x4433; header[10] = 25;
  auto hand = parse_hand_info(header, 11, true);
  assert(hand.error_fault[0] == 0x11 && hand.error_fault[6] == 0);
  assert(hand.error_critical[0] == 0x33 && hand.temperature == 25);
  JointCommandInput input;
  assert(make_joint_command(input, HandControlMode::POSITION, 0, &command, true));
  assert(command.loop_type == JointLoopType::SEMI_CLOSED_LOOP);
  input.loop_type = 1;
  assert(!make_joint_command(input, HandControlMode::POSITION, 0, &command, true));
  input.loop_type = NAN;
  assert(!make_joint_command(input, HandControlMode::MIT, 0, &command, true));
  assert(encode_ethercat_command(HandControlMode::POSITION, g5->active_joints, {}, 6).size() == 158);
}
