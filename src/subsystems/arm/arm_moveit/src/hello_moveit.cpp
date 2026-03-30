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

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() { executor.spin(); });
  auto shutdown = [&]() { executor.cancel(); spin_thread.join(); rclcpp::shutdown(); };

  static constexpr char kGroup[] = "athena_arm";
  MoveGroupInterface mg(node, kGroup);
  mg.setPlanningPipelineId("ompl");
  mg.setPlannerId("RRTConnectkConfigDefault");
  mg.setPlanningTime(15.0);
  mg.setNumPlanningAttempts(10);
  mg.setMaxVelocityScalingFactor(0.5);
  mg.setMaxAccelerationScalingFactor(0.5);

  // ── 1. Move to "ready" ──
  RCLCPP_INFO(logger, "Moving to 'ready'");
  mg.setNamedTarget("ready");
  if (mg.move() != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "Failed to reach 'ready'");
    shutdown(); return 1;
  }
  std::this_thread::sleep_for(std::chrono::seconds(2));

  // ── 2. Capture EE position at "ready" as rectangle centre ──
  const std::string ee_link = mg.getEndEffectorLink();
  const auto ref = mg.getCurrentPose(ee_link).pose.position;
  RCLCPP_INFO(logger, "EE link: %s  pos=[%.4f, %.4f, %.4f]",
    ee_link.c_str(), ref.x, ref.y, ref.z);

  // ── 3. Define corners of a 20x20 cm rectangle in the YZ-plane ──
  //
  //  Position-only targets let the gripper rotate freely as base_yaw
  //  swings to reach each Y coordinate.
  //
  //       TL ────── (+Y) ──────► TR
  //       ▲                       │
  //       (+Z)                  (-Z)
  //       │                       ▼
  //       BL ◄────── (-Y) ─────── BR
  //
  constexpr double half_w = 0.10;   // 10 cm in Y
  constexpr double half_h = 0.10;   // 10 cm in Z

  struct Corner { double y; double z; const char * label; };
  const std::vector<Corner> path = {
    { ref.y - half_w, ref.z + half_h, "TL" },
    { ref.y + half_w, ref.z + half_h, "TR" },
    { ref.y + half_w, ref.z - half_h, "BR" },
    { ref.y - half_w, ref.z - half_h, "BL" },
    { ref.y - half_w, ref.z + half_h, "TL (close)" },
  };

  RCLCPP_INFO(logger, "Rectangle: 20 cm (Y) x 20 cm (Z), centred on 'ready' EE");

  // ── 4. Visit each corner ──
  for (size_t i = 0; i < path.size(); ++i) {
    const auto & c = path[i];
    RCLCPP_INFO(logger, "[%zu/%zu] -> %s  (y=%.3f, z=%.3f)",
      i + 1, path.size(), c.label, c.y, c.z);

    bool reached = false;
    for (int attempt = 1; attempt <= 5; ++attempt) {
      mg.setPositionTarget(ref.x, c.y, c.z, ee_link);
      if (mg.move() == moveit::core::MoveItErrorCode::SUCCESS) { reached = true; break; }
      RCLCPP_WARN(logger, "Attempt %d failed for '%s', retrying...", attempt, c.label);
    }
    if (!reached) {
      RCLCPP_ERROR(logger, "Failed to reach '%s' after 5 attempts", c.label);
      shutdown(); return 2;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  // ── 5. Return to "ready" ──
  RCLCPP_INFO(logger, "Returning to 'ready'");
  mg.setNamedTarget("ready");
  mg.move();

  RCLCPP_INFO(logger, "Done: traced 20x20 cm YZ-plane rectangle.");
  shutdown();
  return 0;
}
