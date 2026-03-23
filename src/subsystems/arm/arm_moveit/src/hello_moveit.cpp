#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include "moveit/move_group_interface/move_group_interface.h"

using moveit::planning_interface::MoveGroupInterface;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  auto const logger = rclcpp::get_logger("hello_moveit");

  // Spin in the background so MoveIt action clients / TF can progress (needed for multi-step).
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() { executor.spin(); });

  static constexpr char kPlanningGroup[] = "athena_arm";

  MoveGroupInterface move_group_interface(node, kPlanningGroup);
  move_group_interface.setPlannerId("RRTConnectkConfigDefault");

  // Virtual demo: plan and execute a short sequence of named joint goals (see athena_arm.srdf).
  const std::vector<std::string> trajectory_targets = {
    "ready",
    "look_left",
    "look_right",
    "ready",
  };

  for (size_t step = 0; step < trajectory_targets.size(); ++step) {
    const std::string & target_name = trajectory_targets[step];

    move_group_interface.setStartStateToCurrentState();
    if (!move_group_interface.setNamedTarget(target_name)) {
      RCLCPP_ERROR(logger, "Unknown named target '%s' (check SRDF group_state for group %s)",
        target_name.c_str(), kPlanningGroup);
      executor.cancel();
      spin_thread.join();
      rclcpp::shutdown();
      return 2;
    }

    MoveGroupInterface::Plan plan;
    const bool planned = static_cast<bool>(move_group_interface.plan(plan));
    if (!planned) {
      RCLCPP_ERROR(logger, "Planning failed at step %zu -> '%s'", step, target_name.c_str());
      executor.cancel();
      spin_thread.join();
      rclcpp::shutdown();
      return 3;
    }

    RCLCPP_INFO(logger, "Executing step %zu / %zu -> '%s'",
      step + 1, trajectory_targets.size(), target_name.c_str());

    const auto result = move_group_interface.execute(plan);
    if (result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(logger, "Execute failed at step %zu -> '%s' (code %d)",
        step, target_name.c_str(), static_cast<int>(result.val));
      executor.cancel();
      spin_thread.join();
      rclcpp::shutdown();
      return 4;
    }

    // Brief pause between segments so logs / RViz are easier to follow (virtual testing).
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
  }

  RCLCPP_INFO(logger, "Completed %zu planned trajectories.", trajectory_targets.size());

  executor.cancel();
  spin_thread.join();
  rclcpp::shutdown();
  return 0;
}
