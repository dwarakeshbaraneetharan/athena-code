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

#include "led_ros2_control/led_hardware_interface.hpp"

#include <cmath>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace led_ros2_control
{

// ── Helper functions ─────────────────────────────────────────────────────────

void LEDHardwareInterface::onCanMessage(const CANLib::CanFrame& frame)
{
  // No replies expected from the LED controller board
  (void)frame;
}

void LEDHardwareInterface::sendLedCommand(uint8_t cmd_byte)
{
  if (!can_connected_) {
    return;
  }

  can_tx_frame_ = CANLib::CanFrame();
  can_tx_frame_.id = can_id_;
  can_tx_frame_.dlc = 1;
  can_tx_frame_.data[0] = cmd_byte;
  canBus_.send(can_tx_frame_);
}

uint8_t LEDHardwareInterface::commandValueToCanByte(double value) const
{
  // Round to nearest integer to handle floating-point imprecision
  int cmd = static_cast<int>(std::round(value));
  switch (cmd) {
    case 1:  return CMD_ON;
    case 2:  return CMD_RED;
    case 3:  return CMD_GREEN;
    default: return CMD_OFF;   // 0 or anything unexpected -> OFF
  }
}

// ── Lifecycle: on_init ───────────────────────────────────────────────────────

hardware_interface::CallbackReturn LEDHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Parse CAN interface name
  if (info_.hardware_parameters.count("can_interface")) {
    can_interface_ = info_.hardware_parameters.at("can_interface");
  } else {
    can_interface_ = "can0";
  }

  // Parse CAN ID (hex or decimal)
  if (info_.hardware_parameters.count("can_id")) {
    can_id_ = static_cast<uint32_t>(
      std::stoul(info_.hardware_parameters.at("can_id"), nullptr, 0));
  } else {
    can_id_ = 0x170;
  }

  // Parse default state
  default_state_value_ = 0.0;  // OFF
  if (info_.hardware_parameters.count("default_state")) {
    const std::string & ds = info_.hardware_parameters.at("default_state");
    if (ds == "on")         { default_state_value_ = 1.0; }
    else if (ds == "red")   { default_state_value_ = 2.0; }
    else if (ds == "green") { default_state_value_ = 3.0; }
    // else "off" or anything else -> 0.0
  }

  // Initialize state variables
  led_state_ = default_state_value_;
  is_connected_ = 0.0;

  // Initialize command variable to default
  led_command_ = default_state_value_;

  can_connected_ = false;

  RCLCPP_INFO(
    rclcpp::get_logger("LEDHardwareInterface"),
    "Initialized LED on CAN interface %s with ID 0x%X (default state: %.0f)",
    can_interface_.c_str(), can_id_, default_state_value_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── Lifecycle: on_configure ──────────────────────────────────────────────────

hardware_interface::CallbackReturn LEDHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("LEDHardwareInterface"),
    "Configuring LED hardware...");

  // Open CAN bus
  if (!canBus_.open(can_interface_,
      std::bind(&LEDHardwareInterface::onCanMessage, this, std::placeholders::_1)))
  {
    RCLCPP_WARN(
      rclcpp::get_logger("LEDHardwareInterface"),
      "Failed to open CAN interface %s - running in SIMULATION mode",
      can_interface_.c_str());
    can_connected_ = false;
  } else {
    can_connected_ = true;
    RCLCPP_INFO(
      rclcpp::get_logger("LEDHardwareInterface"),
      "Successfully opened CAN interface %s", can_interface_.c_str());
  }

  is_connected_ = can_connected_ ? 1.0 : 0.0;

  // Set default state on hardware
  sendLedCommand(commandValueToCanByte(default_state_value_));

  RCLCPP_INFO(
    rclcpp::get_logger("LEDHardwareInterface"),
    "LED hardware configured (%s)", can_connected_ ? "CAN MODE" : "SIMULATION");

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── Interface exports ────────────────────────────────────────────────────────

std::vector<hardware_interface::StateInterface>
LEDHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Use the gpio name from URDF
  const std::string& name = info_.gpios[0].name;

  state_interfaces.emplace_back(
    hardware_interface::StateInterface(name, "led_state", &led_state_));

  state_interfaces.emplace_back(
    hardware_interface::StateInterface(name, "is_connected", &is_connected_));

  RCLCPP_INFO(
    rclcpp::get_logger("LEDHardwareInterface"),
    "Exported %zu state interfaces", state_interfaces.size());

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
LEDHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Use the gpio name from URDF
  const std::string& name = info_.gpios[0].name;

  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(name, "led_command", &led_command_));

  RCLCPP_INFO(
    rclcpp::get_logger("LEDHardwareInterface"),
    "Exported %zu command interfaces", command_interfaces.size());

  return command_interfaces;
}

// ── Lifecycle: activate / deactivate ─────────────────────────────────────────

hardware_interface::CallbackReturn LEDHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("LEDHardwareInterface"),
    "Activating LED hardware...");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LEDHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("LEDHardwareInterface"),
    "Deactivating LED hardware...");

  // Turn LED off on deactivation (safety)
  sendLedCommand(CMD_OFF);
  led_state_ = 0.0;

  RCLCPP_INFO(
    rclcpp::get_logger("LEDHardwareInterface"),
    "LED turned OFF (deactivation)%s", can_connected_ ? "" : " (simulated)");

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── Lifecycle: cleanup / shutdown ────────────────────────────────────────────

hardware_interface::CallbackReturn LEDHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("LEDHardwareInterface"),
    "Cleaning up LED hardware...");

  // Ensure LED is OFF before closing
  sendLedCommand(CMD_OFF);
  led_state_ = 0.0;

  if (can_connected_) {
    canBus_.close();
  }

  can_connected_ = false;
  is_connected_ = 0.0;

  RCLCPP_INFO(
    rclcpp::get_logger("LEDHardwareInterface"),
    "LED hardware cleanup complete");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LEDHardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(
    rclcpp::get_logger("LEDHardwareInterface"),
    "Shutting down LED hardware...");

  return on_cleanup(previous_state);
}

// ── Read / Write ─────────────────────────────────────────────────────────────

hardware_interface::return_type LEDHardwareInterface::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  // LED state is tracked from commands; no CAN feedback to read
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type LEDHardwareInterface::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  // Round both to integers for comparison (avoid floating-point drift)
  int commanded = static_cast<int>(std::round(led_command_));
  int current   = static_cast<int>(std::round(led_state_));

  if (commanded != current) {
    uint8_t can_byte = commandValueToCanByte(led_command_);
    sendLedCommand(can_byte);

    led_state_ = static_cast<double>(commanded);

    RCLCPP_DEBUG(
      rclcpp::get_logger("LEDHardwareInterface"),
      "LED command sent: 0x%02X (state=%.0f)%s",
      can_byte, led_state_, can_connected_ ? "" : " (simulated)");
  }

  return hardware_interface::return_type::OK;
}

}  // namespace led_ros2_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  led_ros2_control::LEDHardwareInterface,
  hardware_interface::SystemInterface)
