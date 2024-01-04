#include <hello_world/setup_mtc_wave_hand.hpp>

#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/task.h>

#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/fixed_state.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/msg/robot_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace
{
// Define a constant for the ID of the Behavior's data port.
constexpr auto kPortIDTask = "task";

// Define constants for the names of the Behavior parameters used by the SetupMTCWaveHand Behavior.
// These defaults are set to match the UR5e robot defined in the example packages.
constexpr auto kPrimaryGroupName = "manipulator";
constexpr auto kHandFrameName = "wrist_1_link";


// constexpr auto kPrimaryGroupName = "upper_arm_group";
// constexpr auto kHandFrameName = "base_link";

// Create a rclcpp Logger to use when printing messages to the console.
const rclcpp::Logger kLogger = rclcpp::get_logger("WaveHello");
}

namespace hello_world
{
SetupMTCWaveHand::SetupMTCWaveHand(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources)
  : moveit_studio::behaviors::AsyncBehaviorBase(name, config, shared_resources)
{
}

BT::PortsList SetupMTCWaveHand::providedPorts()
{
  return {
    BT::BidirectionalPort<std::shared_ptr<moveit::task_constructor::Task>>(kPortIDTask),
  };
}

fp::Result<bool> SetupMTCWaveHand::doWork()
{
  // Retrieve the "task" data port
  const auto task = getInput<moveit::task_constructor::TaskPtr>(kPortIDTask);

  // Check that all required input data ports were set
  if (const auto error = moveit_studio::behaviors::maybe_error(task); error)
  {
    RCLCPP_ERROR_STREAM(kLogger, "Failed to get required values from input data ports:\n" << error.value());
    // Task setup cannot succeed if we failed to retrieve the MTC task shared_ptr from the "task" port, so we return false.
    return tl::make_unexpected(fp::Internal("Failed to get required values from input data ports: " + error.value()));
  }

  // Create a Cartesian path planner used to perform linear moves relative to the end effector frame.
  auto cartesian_planner = std::make_shared<moveit::task_constructor::solvers::CartesianPath>();
  auto joint_interpolation_planner=std::make_shared<moveit::task_constructor::solvers::JointInterpolationPlanner>();

  // Create an MTC stage to define a motion that translates along the end effector X+ axis. Assumes that the robot is not already at its joint limits.

  // Set the direction to move along as a TwistStamped.
  geometry_msgs::msg::TwistStamped wave_first_direction;
  wave_first_direction.header.frame_id = kHandFrameName;
  wave_first_direction.twist.linear.x = 1.0;
  wave_first_direction.twist.linear.z = 0.5;

  // Create a new MTC stage
   {
    // Create a new MoveRelative stage that uses the Cartesian motion planner
    auto stage = std::make_unique<moveit::task_constructor::stages::MoveRelative>(
        std::string("Wave One Direction"), cartesian_planner);
    auto stage = std::make_unique<moveit::task_constructor::stages::MoveTo>("Wave One Direction", joint_interpolation_planner);
    stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT);
    stage->setGroup(kPrimaryGroupName);
    stage->setIKFrame(kHandFrameName);
    task.value()->add(std::move(stage));
  }
  geometry_msgs::msg::TwistStamped wave_second_direction;
  wave_second_direction.header.frame_id = kHandFrameName;
  wave_second_direction.twist.linear.x = -1.0;

  {
    auto stage = std::make_unique<moveit::task_constructor::stages::MoveRelative>(
        std::string("Wave Opposite Direction"), cartesian_planner);
     auto stage = std::make_unique<moveit::task_constructor::stages::MoveTo>("Wave One Direction", joint_interpolation_planner);
    stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT);
   
    stage->setGroup(kPrimaryGroupName);
  
    stage->setIKFrame(kHandFrameName);
    stage->setDirection(wave_second_direction);
    stage->setMaxDistance(0.2);
    task.value()->add(std::move(stage));
  }


geometry_msgs::msg::TwistStamped wave_third_direction;
  wave_second_direction.header.frame_id = kHandFrameName;
  wave_second_direction.twist.linear.z = 0.4;

  {
    auto stage = std::make_unique<moveit::task_constructor::stages::MoveRelative>(
        std::string("Wave Opposite Direction"), cartesian_planner);
    stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT);
    stage->setGroup(kPrimaryGroupName);
    stage->setIKFrame(kHandFrameName);
    stage->setDirection(wave_third_direction);
    stage->setMaxDistance(1);
    task.value()->add(std::move(stage));
  }
geometry_msgs::msg::TwistStamped wave_fourth_direction;
  wave_second_direction.header.frame_id = kHandFrameName;
  wave_second_direction.twist.linear.x = -2.0;

  {
    // Create a new MoveRelative stage that uses the Cartesian motion planner
    auto stage = std::make_unique<moveit::task_constructor::stages::MoveRelative>(
        std::string("Wave Opposite Direction"), cartesian_planner);
    stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT);
    stage->setGroup(kPrimaryGroupName);
    stage->setIKFrame(kHandFrameName);
    stage->setDirection(wave_fourth_direction);
    stage->setMaxDistance(2);
    task.value()->add(std::move(stage));
  }

  return true;
}

}  