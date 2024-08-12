#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <mutex>

using std::placeholders::_1;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_tiago");

class HapticTiago : public rclcpp::Node {
private:
  std::mutex mutex;
  moveit::planning_interface::MoveGroupInterface *move_group;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription;

  void haptic_callback(const geometry_msgs::msg::PoseStamped &msg) {
    this->mutex.lock();

    RCLCPP_INFO(LOGGER, "Haptic Callback");

    geometry_msgs::msg::PoseStamped randomPose = move_group->getRandomPose();

    RCLCPP_INFO(LOGGER, "Despues del random pose");

    move_group->setPoseTarget(randomPose);

    RCLCPP_INFO(LOGGER, "Despues del setPoseTarget");

    // Now, we call the planner to compute the plan and visualize it.
    // Note that we are just planning, not asking move_group
    // to actually move the robot.
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success =
        (move_group->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    RCLCPP_INFO(LOGGER, "Plan for haptic pose: %s", success ? "" : "FAILED");

    if (success) {
      move_group->move();
    }

    this->mutex.unlock();
  }

public:
  HapticTiago(rclcpp::NodeOptions options) : Node("haptic_tiago", options) {
    rclcpp::CallbackGroup::SharedPtr my_callback_group =
        create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.callback_group = my_callback_group;

    subscription = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/phantom/pose", 10, std::bind(&HapticTiago::haptic_callback, this, _1),
        subscription_options);
  }

  ~HapticTiago() { delete move_group; }

  void setMoveGroup(moveit::planning_interface::MoveGroupInterface *m) {
    this->move_group = m;
  }

  void moveitStuff(std::shared_ptr<HapticTiago> &node) {
    // // Raw pointers are frequently used to refer to the planning group for
    // improved performance. const moveit::core::JointModelGroup*
    // joint_model_group =
    //     move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // // Getting Basic Information
    // // ^^^^^^^^^^^^^^^^^^^^^^^^^
    // //
    // // We can print the name of the reference frame for this robot.
    // RCLCPP_INFO(LOGGER, "Planning frame: %s",
    // move_group->getPlanningFrame().c_str());

    // // We can also print the name of the end-effector link for this group.
    // RCLCPP_INFO(LOGGER, "End effector link: %s",
    // move_group->getEndEffectorLink().c_str());

    // geometry_msgs::msg::PoseStamped currentPose =
    // move_group->getCurrentPose(); RCLCPP_INFO(LOGGER, "Current pose: %f %f
    // %f", currentPose.pose.position.x, currentPose.pose.position.y,
    // currentPose.pose.position.z);

    // geometry_msgs::msg::PoseStamped randomPose = move_group->getRandomPose();
    // RCLCPP_INFO(LOGGER, "Random pose: %f %f %f", randomPose.pose.position.x,
    // randomPose.pose.position.y, randomPose.pose.position.z);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);

  auto haptic_tiago_node = std::make_shared<HapticTiago>(node_options);
  const std::string PLANNING_GROUP = "arm";
  auto move_group = new moveit::planning_interface::MoveGroupInterface(
      haptic_tiago_node, PLANNING_GROUP);

  haptic_tiago_node->setMoveGroup(move_group);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(haptic_tiago_node);
  std::thread spinner = std::thread([&executor]() { executor.spin(); });

  // auto move_group_node = rclcpp::Node::make_shared("move_tiago",
  // node_options);

  // We spin up a SingleThreadedExecutor for the current state monitor to get
  // information about the robot's state.
  // rclcpp::executors::SingleThreadedExecutor executor;
  // executor.add_node(move_group_node);
  // std::thread([&executor]() { executor.spin(); }).detach();

  spinner.join();

  // Shutdown ROS
  rclcpp::shutdown();

  return 0;
}
