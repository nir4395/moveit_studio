#pragma once

#include <behaviortree_cpp/action_node.h>
#include <moveit_studio_behavior_interface/shared_resources_node.hpp>
#include <moveit_studio_behavior_interface/behavior_context.hpp>
#include <moveit_studio_behavior_interface/check_for_error.hpp>
#include <moveit_studio_behavior_interface/async_behavior_base.hpp>

namespace hello_world
{
/**
* @brief The CustomBeahvior Behavior makes any robot move the end of its arm back and forth.
* @details This is an example of a Behavior that uses MoveIt Task Constructor to configure an MTC task,
* which can be planned and executed by MoveIt Studio's core PlanMTCTask and ExecuteMTCTask Behaviors.
* It is derived from AsyncBehaviorBase, an extension of the templated SharedResourcesNode type,
* which augments the core BehaviorTree.Cpp types with an additional constructor parameter
* to allow the Behavior to access a rclcpp Node owned by the Agent's ObjectiveServerNode.
* The AsyncBehaviorBase requires a user-implemented function doWork()
* which is called within an async process in a separate thread.
* It returns an fp::Result which contains a bool indicating task success
* if the process completed successfully or was canceled,
* or an error message if the process failed unexpectedly.
*/
class CustomBeahvior final : public moveit_studio::behaviors::AsyncBehaviorBase
{
public:
  /**
  * @brief Constructor for the CustomBeahvior Behavior.
  * @param name The name of a particular instance of this Behavior. This will be set by the Behavior
  * tree factory when this Behavior is created within a new Behavior tree.
  * @param config This contains runtime configuration info for this Behavior, such as the mapping
  * between the Behavior's data ports on the Behavior tree's blackboard. This will be set by the
  * Behavior tree factory when this Behavior is created within a new Behavior tree.
  * @param shared_resources A shared_ptr to a BehaviorContext that is shared among all
  * SharedResourcesNode Behaviors in the Behavior tree. This BehaviorContext is owned by the MoveIt Studio
  * Agent's ObjectiveServerNode.
  */
  CustomBeahvior(const std::string& name, const BT::NodeConfiguration& config,
                  const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources);

  /**
  * @brief Implementation of the required providedPorts() function for the CustomBeahvior Behavior.
  * @details The BehaviorTree.CPP library requires that Behaviors must implement a static function
  * named providedPorts() which defines their input and output ports. If the Behavior does not use
  * any ports, this function must return an empty BT::PortsList.
  * This function returns a list of ports with their names and port info, which is used internally
  * by the Behavior tree.
  * @return CustomBeahvior has one data port: a bidirectional port named "task", which is a shared_ptr
  * to an MTC task object. This function returns a BT::PortsList that declares this single port.
  */
  static BT::PortsList providedPorts();

private:
  /**
  * @brief Async thread for CustomBeahvior. Adds MTC stages to an MTC task provided on a data port.
  * @details This function is where the Behavior performs its work asynchronously while the Behavior tree ticks.
  * It is very important that Behaviors return from being ticked very quickly because if it blocks before returning
  * it will block execution of the entire Behavior tree, which may have undesirable consequences for other Behaviors
  * that require a fast update rate to work correctly.
  * @return An fp::Result which contains a true if the MTC stages were configured and added to the MTC task,
  * or an error if it failed to retrieve the MTC task from the "task" data port.
  */
  fp::Result<bool> doWork() override;

  /** @brief Classes derived from AsyncBehaviorBase must implement getFuture() so that it returns a shared_future class member */
  std::shared_future<fp::Result<bool>>& getFuture() override
  {
    return future_;
  }

  /** @brief Classes derived from AsyncBehaviorBase must have this shared_future as a class member */
  std::shared_future<fp::Result<bool>> future_;
};
}  // namespace hello_world