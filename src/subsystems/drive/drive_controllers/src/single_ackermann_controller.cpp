// Copyright (c) 2025, UMDLoop
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

#include "athena_drive_controllers/single_ackermann_controller.hpp"

#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>

#include "controller_interface/helpers.hpp"
#include "rclcpp/rclcpp.hpp"

namespace drive_controllers
{
SingleAckermannController::SingleAckermannController() : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn SingleAckermannController::on_init()
{
  try
  {
    param_listener_ = std::make_shared<single_ackermann_controller::ParamListener>(get_node());
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during controller's init with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SingleAckermannController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  params_ = param_listener_->get_params();

  // Validate that all joint names are provided
  if (params_.front_left_steer_joint.empty() || params_.front_right_steer_joint.empty() ||
      params_.front_left_drive_joint.empty() || params_.front_right_drive_joint.empty() ||
      params_.rear_left_drive_joint.empty() || params_.rear_right_drive_joint.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), 
      "All joint names must be specified: front_left_steer_joint, front_right_steer_joint, "
      "front_left_drive_joint, front_right_drive_joint, rear_left_drive_joint, rear_right_drive_joint");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Validate geometry parameters
  if (params_.wheelbase <= 0.0 || params_.track_width <= 0.0 || params_.wheel_radius <= 0.0)
  {
    RCLCPP_ERROR(get_node()->get_logger(),
      "Geometry parameters must be positive: wheelbase=%.3f, track_width=%.3f, wheel_radius=%.3f",
      params_.wheelbase, params_.track_width, params_.wheel_radius);
    return controller_interface::CallbackReturn::ERROR;
  }

  // Validate max_steer_angle to prevent geometry issues
  // At max_steer_angle, turn_radius = wheelbase / tan(max_steer_angle)
  // We need turn_radius > track_width / 2 for valid geometry
  double min_turn_radius = params_.wheelbase / tan(params_.max_steer_angle);
  if (min_turn_radius <= params_.track_width / 2.0)
  {
    RCLCPP_WARN(get_node()->get_logger(),
      "max_steer_angle (%.3f rad) may cause geometry issues. Min turn radius (%.3f m) <= half track width (%.3f m). "
      "Consider reducing max_steer_angle.",
      params_.max_steer_angle, min_turn_radius, params_.track_width / 2.0);
  }

  auto subscribers_qos = rclcpp::SystemDefaultsQoS();
  subscribers_qos.keep_last(1);
  subscribers_qos.best_effort();

  ref_subscriber_ = get_node()->create_subscription<ControllerReferenceMsg>(
    "~/reference", subscribers_qos,
    std::bind(&SingleAckermannController::reference_callback, this, std::placeholders::_1));

  input_ref_.writeFromNonRT(std::make_shared<ControllerReferenceMsg>());

  RCLCPP_INFO(get_node()->get_logger(), "SingleAckermannController configured:");
  RCLCPP_INFO(get_node()->get_logger(), "  Steering joints:");
  RCLCPP_INFO(get_node()->get_logger(), "    FL: %s (invert: %s)", 
    params_.front_left_steer_joint.c_str(), params_.front_left_steer_inversion ? "true" : "false");
  RCLCPP_INFO(get_node()->get_logger(), "    FR: %s (invert: %s)", 
    params_.front_right_steer_joint.c_str(), params_.front_right_steer_inversion ? "true" : "false");
  RCLCPP_INFO(get_node()->get_logger(), "  Drive joints:");
  RCLCPP_INFO(get_node()->get_logger(), "    FL: %s (invert: %s)", 
    params_.front_left_drive_joint.c_str(), params_.front_left_drive_inversion ? "true" : "false");
  RCLCPP_INFO(get_node()->get_logger(), "    FR: %s (invert: %s)", 
    params_.front_right_drive_joint.c_str(), params_.front_right_drive_inversion ? "true" : "false");
  RCLCPP_INFO(get_node()->get_logger(), "    RL: %s (invert: %s)", 
    params_.rear_left_drive_joint.c_str(), params_.rear_left_drive_inversion ? "true" : "false");
  RCLCPP_INFO(get_node()->get_logger(), "    RR: %s (invert: %s)", 
    params_.rear_right_drive_joint.c_str(), params_.rear_right_drive_inversion ? "true" : "false");
  RCLCPP_INFO(get_node()->get_logger(), "  Geometry: wheelbase=%.3f, track_width=%.3f, wheel_radius=%.3f",
    params_.wheelbase, params_.track_width, params_.wheel_radius);
  RCLCPP_INFO(get_node()->get_logger(), "  Joystick: forward_axis=%d, steer_axis=%d, steer_inversion=%s",
    params_.forward_axis, params_.steer_axis, params_.steer_inversion ? "true" : "false");
  RCLCPP_INFO(get_node()->get_logger(), "  Limits: max_speed=%.3f m/s, max_steer_angle=%.3f rad",
    params_.max_speed, params_.max_steer_angle);

  return controller_interface::CallbackReturn::SUCCESS;
}

void SingleAckermannController::reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg)
{
  input_ref_.writeFromNonRT(msg);
}

controller_interface::InterfaceConfiguration SingleAckermannController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // Steering joints (position control)
  command_interfaces_config.names.push_back(params_.front_left_steer_joint + "/position");
  command_interfaces_config.names.push_back(params_.front_right_steer_joint + "/position");

  // Drive joints (velocity control)
  command_interfaces_config.names.push_back(params_.front_left_drive_joint + "/velocity");
  command_interfaces_config.names.push_back(params_.front_right_drive_joint + "/velocity");
  command_interfaces_config.names.push_back(params_.rear_left_drive_joint + "/velocity");
  command_interfaces_config.names.push_back(params_.rear_right_drive_joint + "/velocity");

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration SingleAckermannController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // Steering joints (position feedback)
  state_interfaces_config.names.push_back(params_.front_left_steer_joint + "/position");
  state_interfaces_config.names.push_back(params_.front_right_steer_joint + "/position");

  // Drive joints (velocity feedback)
  state_interfaces_config.names.push_back(params_.front_left_drive_joint + "/velocity");
  state_interfaces_config.names.push_back(params_.front_right_drive_joint + "/velocity");
  state_interfaces_config.names.push_back(params_.rear_left_drive_joint + "/velocity");
  state_interfaces_config.names.push_back(params_.rear_right_drive_joint + "/velocity");

  return state_interfaces_config;
}

bool SingleAckermannController::find_command_interface_index(const std::string& name, size_t& index)
{
  for (size_t i = 0; i < command_interfaces_.size(); ++i)
  {
    if (command_interfaces_[i].get_name() == name)
    {
      index = i;
      return true;
    }
  }
  return false;
}

controller_interface::CallbackReturn SingleAckermannController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Look up command interface indices by name - this is the critical fix!
  // The order of command_interfaces_ is NOT guaranteed to match our request order.

  bool all_found = true;

  if (!find_command_interface_index(params_.front_left_steer_joint + "/position", fl_steer_cmd_idx_)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Could not find command interface for %s/position", 
      params_.front_left_steer_joint.c_str());
    all_found = false;
  }
  if (!find_command_interface_index(params_.front_right_steer_joint + "/position", fr_steer_cmd_idx_)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Could not find command interface for %s/position", 
      params_.front_right_steer_joint.c_str());
    all_found = false;
  }
  if (!find_command_interface_index(params_.front_left_drive_joint + "/velocity", fl_drive_cmd_idx_)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Could not find command interface for %s/velocity", 
      params_.front_left_drive_joint.c_str());
    all_found = false;
  }
  if (!find_command_interface_index(params_.front_right_drive_joint + "/velocity", fr_drive_cmd_idx_)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Could not find command interface for %s/velocity", 
      params_.front_right_drive_joint.c_str());
    all_found = false;
  }
  if (!find_command_interface_index(params_.rear_left_drive_joint + "/velocity", rl_drive_cmd_idx_)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Could not find command interface for %s/velocity", 
      params_.rear_left_drive_joint.c_str());
    all_found = false;
  }
  if (!find_command_interface_index(params_.rear_right_drive_joint + "/velocity", rr_drive_cmd_idx_)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Could not find command interface for %s/velocity", 
      params_.rear_right_drive_joint.c_str());
    all_found = false;
  }

  if (!all_found) {
    return controller_interface::CallbackReturn::ERROR;
  }

  // Initialize all command interfaces to zero to prevent initial movement
  for (size_t i = 0; i < command_interfaces_.size(); ++i)
  {
    command_interfaces_[i].set_value(0.0);
  }

  RCLCPP_INFO(get_node()->get_logger(), 
    "SingleAckermannController activated - interface indices: FL_steer=%zu, FR_steer=%zu, "
    "FL_drive=%zu, FR_drive=%zu, RL_drive=%zu, RR_drive=%zu",
    fl_steer_cmd_idx_, fr_steer_cmd_idx_, fl_drive_cmd_idx_, fr_drive_cmd_idx_, 
    rl_drive_cmd_idx_, rr_drive_cmd_idx_);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SingleAckermannController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (size_t i = 0; i < command_interfaces_.size(); ++i)
  {
    command_interfaces_[i].set_value(0.0);
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type SingleAckermannController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  auto current_ref = input_ref_.readFromRT();
  if (!current_ref || !(*current_ref) || (*current_ref)->axes.empty())
  {
    // Set all command interfaces to zero when no input is available
    for (size_t i = 0; i < command_interfaces_.size(); ++i)
    {
      command_interfaces_[i].set_value(0.0);
    }
    return controller_interface::return_type::OK;
  }

  // Bounds check for joystick axes to prevent crash
  const auto& axes = (*current_ref)->axes;
  if (params_.forward_axis < 0 || static_cast<size_t>(params_.forward_axis) >= axes.size() ||
      params_.steer_axis < 0 || static_cast<size_t>(params_.steer_axis) >= axes.size())
  {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "Joystick axes out of bounds: forward_axis=%d, steer_axis=%d, axes.size()=%zu",
      params_.forward_axis, params_.steer_axis, axes.size());
    for (size_t i = 0; i < command_interfaces_.size(); ++i)
    {
      command_interfaces_[i].set_value(0.0);
    }
    return controller_interface::return_type::OK;
  }

  // Get joystick values and apply scaling and inversion from parameters
  double linear_vel_cmd = axes[params_.forward_axis] * params_.max_speed;
  double steer_cmd = axes[params_.steer_axis] * params_.max_steer_angle;
  if (params_.steer_inversion) {
    steer_cmd *= -1.0;
  }

  double wheelbase = params_.wheelbase;
  double track_width = params_.track_width;
  double wheel_radius = params_.wheel_radius;

  double front_left_steer_angle = 0.0;
  double front_right_steer_angle = 0.0;
  
  double front_left_vel = linear_vel_cmd;
  double front_right_vel = linear_vel_cmd;
  double rear_left_vel = linear_vel_cmd;
  double rear_right_vel = linear_vel_cmd;

  // If we are turning...
  if (std::abs(steer_cmd) > 1e-4) {
    double turn_radius = wheelbase / tan(steer_cmd);
    
    // Protect against very tight turns (turn radius too small)
    double min_turn_radius = track_width / 2.0 + 0.01;  // Add small margin
    if (std::abs(turn_radius) < min_turn_radius) {
      // Clamp turn radius to minimum safe value
      turn_radius = std::copysign(min_turn_radius, turn_radius);
      if (params_.enable_debug_logging) {
        RCLCPP_DEBUG(get_node()->get_logger(), "Turn radius clamped to minimum: %.3f", turn_radius);
      }
    }
    
    double angular_vel = std::abs(linear_vel_cmd) / std::abs(turn_radius);
    
    // Preserve the sign of linear velocity for forward/backward motion
    if (linear_vel_cmd < 0) {
      angular_vel = -angular_vel;
    }

    // Calculate magnitudes of inner and outer wheel angles using Ackermann geometry
    // Inner wheel turns MORE (tighter radius), outer wheel turns LESS (wider radius)
    double inner_angle = atan(wheelbase / (std::abs(turn_radius) - track_width / 2.0));
    double outer_angle = atan(wheelbase / (std::abs(turn_radius) + track_width / 2.0));
    
    // Calculate magnitudes of wheel speeds
    // Inner wheels travel shorter distance, outer wheels travel longer distance
    double inner_rear_vel = angular_vel * (std::abs(turn_radius) - track_width / 2.0);
    double outer_rear_vel = angular_vel * (std::abs(turn_radius) + track_width / 2.0);
    double inner_front_vel = angular_vel * sqrt(pow(wheelbase, 2) + pow(std::abs(turn_radius) - track_width / 2.0, 2));
    double outer_front_vel = angular_vel * sqrt(pow(wheelbase, 2) + pow(std::abs(turn_radius) + track_width / 2.0, 2));

    // Assign angles and velocities based on turn direction
    // Positive steer_cmd = LEFT turn (left wheel is inner)
    // Negative steer_cmd = RIGHT turn (right wheel is inner)
    if (steer_cmd > 0.0) { 
      // LEFT TURN: left wheel is INNER (tighter turn angle)
      front_left_steer_angle = -inner_angle;   // Negative = turn left
      front_right_steer_angle = -outer_angle;  // Negative = turn left (but less)
      
      front_left_vel = inner_front_vel;
      front_right_vel = outer_front_vel;
      rear_left_vel = inner_rear_vel;
      rear_right_vel = outer_rear_vel;

    } else { 
      // RIGHT TURN: right wheel is INNER (tighter turn angle)
      front_left_steer_angle = outer_angle;    // Positive = turn right (but less)
      front_right_steer_angle = inner_angle;   // Positive = turn right
      
      front_left_vel = outer_front_vel;
      front_right_vel = inner_front_vel;
      rear_left_vel = outer_rear_vel;
      rear_right_vel = inner_rear_vel;
    }

    if (params_.enable_debug_logging) {
      RCLCPP_DEBUG(get_node()->get_logger(), 
        "Turn: steer_cmd=%.3f, turn_radius=%.3f, FL_angle=%.3f, FR_angle=%.3f",
        steer_cmd, turn_radius, front_left_steer_angle, front_right_steer_angle);
    }
  }

  // Apply per-wheel steer inversion if configured
  if (params_.front_left_steer_inversion) {
    front_left_steer_angle *= -1.0;
  }
  if (params_.front_right_steer_inversion) {
    front_right_steer_angle *= -1.0;
  }

  // Convert linear wheel velocities to angular velocities (rad/s)
  double fl_wheel_ang_vel = front_left_vel / wheel_radius;
  double fr_wheel_ang_vel = front_right_vel / wheel_radius;
  double rl_wheel_ang_vel = rear_left_vel / wheel_radius;
  double rr_wheel_ang_vel = rear_right_vel / wheel_radius;

  // Apply per-wheel drive inversion if configured (for motor mounting differences)
  if (params_.front_left_drive_inversion) {
    fl_wheel_ang_vel *= -1.0;
  }
  if (params_.front_right_drive_inversion) {
    fr_wheel_ang_vel *= -1.0;
  }
  if (params_.rear_left_drive_inversion) {
    rl_wheel_ang_vel *= -1.0;
  }
  if (params_.rear_right_drive_inversion) {
    rr_wheel_ang_vel *= -1.0;
  }

  if (params_.enable_debug_logging) {
    RCLCPP_DEBUG(get_node()->get_logger(), 
      "Commands: FL_steer=%.3f, FR_steer=%.3f, FL_vel=%.3f, FR_vel=%.3f, RL_vel=%.3f, RR_vel=%.3f",
      front_left_steer_angle, front_right_steer_angle, 
      fl_wheel_ang_vel, fr_wheel_ang_vel, rl_wheel_ang_vel, rr_wheel_ang_vel);
  }

  // Set steering positions using looked-up indices (NOT hardcoded order!)
  command_interfaces_[fl_steer_cmd_idx_].set_value(front_left_steer_angle);
  command_interfaces_[fr_steer_cmd_idx_].set_value(front_right_steer_angle);

  // Set drive velocities using looked-up indices (NOT hardcoded order!)
  command_interfaces_[fl_drive_cmd_idx_].set_value(fl_wheel_ang_vel);
  command_interfaces_[fr_drive_cmd_idx_].set_value(fr_wheel_ang_vel);
  command_interfaces_[rl_drive_cmd_idx_].set_value(rl_wheel_ang_vel);
  command_interfaces_[rr_drive_cmd_idx_].set_value(rr_wheel_ang_vel);

  return controller_interface::return_type::OK;
}
}  // namespace drive_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  drive_controllers::SingleAckermannController, controller_interface::ControllerInterface)
