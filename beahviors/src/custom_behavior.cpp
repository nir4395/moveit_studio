// #include <hello_world/custom_beahvior.hpp>

// #include <moveit/task_constructor/solvers.h>
// #include <moveit/task_constructor/stages.h>
// #include <moveit/task_constructor/task.h>

// #include <moveit/task_constructor/stages/move_to.h>
// #include <moveit/task_constructor/stages/fixed_state.h>
// #include <moveit/task_constructor/solvers/joint_interpolation.h>

// #include <moveit/planning_scene/planning_scene.h>
// #include <moveit_msgs/msg/robot_state.hpp>
// #include <geometry_msgs/msg/pose_stamped.hpp>

// using namespace moveit::task_constructor;
// using namespace planning_scene;
// using namespace moveit::core;
// constexpr double TAU{ 2 * M_PI };

// namespace
// {
// // Define a constant for the ID of the Behavior's data port.
// constexpr auto kPortIDTask = "task";

// // Define constants for the names of the Behavior parameters used by the CustomBeahvior Behavior.
// // These defaults are set to match the UR5e robot defined in the example packages.
// constexpr auto kPrimaryGroupName = "manipulator";
// constexpr auto kHandFrameName = "manual_grasp_link";

// // Create a rclcpp Logger to use when printing messages to the console.
// const rclcpp::Logger kLogger = rclcpp::get_logger("WaveHello");
// }

// namespace hello_world
// {


// moveit_msgs::msg::AttachedCollisionObject createAttachedObject(const std::string& id) {
// 	moveit_msgs::msg::AttachedCollisionObject aco;
// 	aco.link_name = kHandFrameName;
// 	aco.object.header.frame_id = aco.link_name;
// 	aco.object.operation = aco.object.ADD;
// 	aco.object.id = id;
// 	aco.object.primitives.resize(1, [] {
// 		shape_msgs::msg::SolidPrimitive p;
// 		p.type = p.SPHERE;
// 		p.dimensions.resize(1);
// 		p.dimensions[p.SPHERE_RADIUS] = 0.01;
// 		return p;
// 	}());
// 	aco.object.pose.position.z = 0.2;
// 	aco.object.pose.orientation.w = 1.0;
// 	aco.object.subframe_names.resize(1, "subframe");
// 	aco.object.subframe_poses.resize(1, [] {
// 		geometry_msgs::msg::Pose p;
// 		p.orientation.w = 1.0;
// 		return p;
// 	}());
// 	return aco;
// }

// CustomBeahvior::CustomBeahvior(
//     const std::string& name, const BT::NodeConfiguration& config,
//     const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources)
//   : moveit_studio::behaviors::AsyncBehaviorBase(name, config, shared_resources)
// {
// }

// BT::PortsList CustomBeahvior::providedPorts()
// {
//   return {
//     BT::BidirectionalPort<std::shared_ptr<moveit::task_constructor::Task>>(kPortIDTask),
//   };
// }

// fp::Result<bool> CustomBeahvior::doWork()
// {
//   // Retrieve the "task" data port
//   const auto task = getInput<moveit::task_constructor::TaskPtr>(kPortIDTask);

//   // Check that all required input data ports were set
//   if (const auto error = moveit_studio::behaviors::maybe_error(task); error)
//   {
//     RCLCPP_ERROR_STREAM(kLogger, "Failed to get required values from input data ports:\n" << error.value());
//     // Task setup cannot succeed if we failed to retrieve the MTC task shared_ptr from the "task" port, so we return false.
//     return tl::make_unexpected(fp::Internal("Failed to get required values from input data ports: " + error.value()));
//   }

//   // Create a Cartesian path planner used to perform linear moves relative to the end effector frame.
//   auto cartesian_planner = std::make_shared<moveit::task_constructor::solvers::CartesianPath>();
//   auto joint_interpolation_planner=std::make_shared<solvers::JointInterpolationPlanner>()

//   // Create an MTC stage to define a motion that translates along the end effector X+ axis. Assumes that the robot is not already at its joint limits.

//   // Set the direction to move along as a TwistStamped.
//   geometry_msgs::msg::TwistStamped wave_first_direction;
//   wave_first_direction.header.frame_id = kHandFrameName;
//   wave_first_direction.twist.linear.x = 1.0;
//   auto scene = std::make_shared<planning_scene::PlanningScene>(task.getRobotModel());
//   scene->processAttachedCollisionObjectMsg(createAttachedObject("attached_object"));


//   // Create a new MTC stage
//   {
//     // Create a new MoveRelative stage that uses the Cartesian motion planner
//     // auto stage = std::make_unique<moveit::task_constructor::stages::MoveRelative>(
//     //     std::string("Wave One Direction"), cartesian_planner);
//     auto stage = std::make_unique<moveit::task_constructor::stages::MoveTo>("Wave One Direction", joint_interpolation_planner);
//     stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT);
//     // Configure the stage to move the UR-5e's arm.
//     stage->setGroup(kPrimaryGroupName);
//     // Configure the stage to move relative to the UR-5e's gripper frame.
//     stage->setIKFrame(kHandFrameName);
//     // Configure the stage to move along the direction specified by the TwistStamped.
//     stage->setDirection(wave_first_direction);
//     // Set that we will move at most 0.2m along the vector.
//     stage->setMaxDistance(0.2);
//     // Add the stage to the MTC task which was retrieved from the "task" data port.
//     task.value()->add(std::move(stage));
//   }

//   // Create a second MTC stage to define a motion that translates along the end effector X- axis, which is the opposite motion from what we did in the previous stage.

//   // Set the direction to move along as a TwistStamped.
//   geometry_msgs::msg::TwistStamped wave_second_direction;
//   wave_second_direction.header.frame_id = kHandFrameName;
//   wave_second_direction.twist.linear.x = 0.7;

//   {
//     // Create a new MoveRelative stage that uses the Cartesian motion planner
//     auto stage = std::make_unique<moveit::task_constructor::stages::MoveRelative>(
//         std::string("Wave Opposite Direction"), cartesian_planner);
//     stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT);
//     // Configure the stage to move the UR-5e's arm.
//     stage->setGroup(kPrimaryGroupName);
//     // Configure the stage to move relative to the UR-5e's gripper frame.
//     stage->setIKFrame(kHandFrameName);
//     // Configure the stage to move along the direction specified by the TwistStamped.
//     stage->setDirection(wave_second_direction);
//     // Set that we will move at most 0.2m along the vector.
//     stage->setMaxDistance(0.2);
//     // Add the stage to the MTC task which was retrieved from the "task" data port.
//     task.value()->add(std::move(stage));
//   }


// // geometry_msgs::msg::TwistStamped wave_third_direction;
// //   wave_second_direction.header.frame_id = kHandFrameName;
// //   wave_second_direction.twist.linear.x = 2.0;

// //   {
// //     // Create a new MoveRelative stage that uses the Cartesian motion planner
// //     auto stage = std::make_unique<moveit::task_constructor::stages::MoveRelative>(
// //         std::string("Wave Opposite Direction"), cartesian_planner);
// //     stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT);
// //     // Configure the stage to move the UR-5e's arm.
// //     stage->setGroup(kPrimaryGroupName);
// //     // Configure the stage to move relative to the UR-5e's gripper frame.
// //     stage->setIKFrame(kHandFrameName);
// //     // Configure the stage to move along the direction specified by the TwistStamped.
// //     stage->setDirection(wave_third_direction);
// //     // Set that we will move at most 0.2m along the vector.
// //     stage->setMaxDistance(1);
// //     // Add the stage to the MTC task which was retrieved from the "task" data port.
// //     task.value()->add(std::move(stage));
// //   }
// // geometry_msgs::msg::TwistStamped wave_fourth_direction;
// //   wave_second_direction.header.frame_id = kHandFrameName;
// //   wave_second_direction.twist.linear.x = -2.0;

// //   {
// //     // Create a new MoveRelative stage that uses the Cartesian motion planner
// //     auto stage = std::make_unique<moveit::task_constructor::stages::MoveRelative>(
// //         std::string("Wave Opposite Direction"), cartesian_planner);
// //     stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT);
// //     // Configure the stage to move the UR-5e's arm.
// //     stage->setGroup(kPrimaryGroupName);
// //     // Configure the stage to move relative to the UR-5e's gripper frame.
// //     stage->setIKFrame(kHandFrameName);
// //     // Configure the stage to move along the direction specified by the TwistStamped.
// //     stage->setDirection(wave_fourth_direction);
// //     // Set that we will move at most 0.2m along the vector.
// //     stage->setMaxDistance(2);
// //     // Add the stage to the MTC task which was retrieved from the "task" data port.
// //     task.value()->add(std::move(stage));
// //   }

//   //Once the work is done, we can return true so that the node returns SUCCESS next time it is ticked
//   return true;
// }

// }  // namespace hello_world