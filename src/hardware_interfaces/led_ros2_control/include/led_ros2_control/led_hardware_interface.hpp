// Copyright (c) 2024 UMD Loop
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//

#ifndef LED_ROS2_CONTROL__LED_HARDWARE_INTERFACE_HPP_
#define LED_ROS2_CONTROL__LED_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <cstdint>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "umdloop_can_library/SocketCanBus.hpp"
#include "umdloop_can_library/CanFrame.hpp"

namespace led_ros2_control
{

/**
 * @brief Hardware interface for LED control via CAN bus through ros2_control
 *
 * This is a SystemInterface for controlling status LEDs via CAN commands
 * sent to the electrical team's LED controller board.
 *
 * CAN Protocol (ID: 0x170):
 * - Turn ON:     DATA[0] = 0x60
 * - Turn OFF:    DATA[0] = 0x80
 * - Set RED:     DATA[0] = 0x90
 * - Set GREEN:   DATA[0] = 0x95
 *
 * State Interfaces (read by controllers):
 * - led_state: Current LED state (0.0=OFF, 1.0=ON, 2.0=RED, 3.0=GREEN)
 * - is_connected: Is CAN connected and ready (0.0 or 1.0)
 *
 * Command Interfaces (written by controllers):
 * - led_command: LED command (0.0=OFF, 1.0=ON, 2.0=RED, 3.0=GREEN)
 *
 * Hardware Parameters (from URDF):
 * - can_interface: CAN interface name (default: "can0")
 * - can_id: CAN ID for LED commands (default: 0x170)
 * - default_state: "off", "on", "red", or "green" (default: "off")
 */
class LEDHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(LEDHardwareInterface)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  // CAN message handler (no replies expected, but required by SocketCanBus API)
  void onCanMessage(const CANLib::CanFrame& frame);

  // Send a single-byte CAN command to the LED controller
  void sendLedCommand(uint8_t cmd_byte);

  // Map a led_command double value to its CAN command byte
  uint8_t commandValueToCanByte(double value) const;

  // Configuration parameters
  std::string can_interface_;
  uint32_t can_id_;
  double default_state_value_;  // Numeric default: 0.0/1.0/2.0/3.0

  // CAN bus
  CANLib::SocketCanBus canBus_;
  CANLib::CanFrame can_tx_frame_;
  bool can_connected_;

  // State variables (hardware -> ros2_control)
  double led_state_;      // Current state: 0.0=OFF, 1.0=ON, 2.0=RED, 3.0=GREEN
  double is_connected_;   // CAN hardware ready status

  // Command variables (ros2_control -> hardware)
  double led_command_;    // Commanded state

  // CAN command bytes
  static constexpr uint8_t CMD_ON    = 0x60;
  static constexpr uint8_t CMD_OFF   = 0x80;
  static constexpr uint8_t CMD_RED   = 0x90;
  static constexpr uint8_t CMD_GREEN = 0x95;
};

}  // namespace led_ros2_control

#endif  // LED_ROS2_CONTROL__LED_HARDWARE_INTERFACE_HPP_
