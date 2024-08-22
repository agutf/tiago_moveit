#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <mutex>
#include <chrono>

using std::placeholders::_1;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_tiago");

class HapticTiago : public rclcpp::Node {
private:
  std::mutex mutex;
  moveit::planning_interface::MoveGroupInterface *move_group;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription;

  std::chrono::time_point<std::chrono::high_resolution_clock> last_processed_pose;

  const double HAPTIC_X_MAX = 0.22;
  const double HAPTIC_X_MIN = -0.22;

  const double HAPTIC_Y_MAX = 0.12;
  const double HAPTIC_Y_MIN = -0.09;

  const double HAPTIC_Z_MAX = 0.2;
  const double HAPTIC_Z_MIN = -0.1;

  const double TIAGO_X_MAX = 0.5;
  const double TIAGO_X_MIN = -0.5;

  const double TIAGO_Y_MAX = 1.0;
  const double TIAGO_Y_MIN = 0.75;

  const double TIAGO_Z_MAX = 0.65;
  const double TIAGO_Z_MIN = 0.55;

  void printPose(geometry_msgs::msg::PoseStamped &pose) {
    RCLCPP_INFO(LOGGER, "*****");
    RCLCPP_INFO(LOGGER, "Position");
    RCLCPP_INFO(LOGGER, "x: %f", pose.pose.position.x);
    RCLCPP_INFO(LOGGER, "y: %f", pose.pose.position.y);
    RCLCPP_INFO(LOGGER, "z: %f", pose.pose.position.z);
    RCLCPP_INFO(LOGGER, "Orientation");
    RCLCPP_INFO(LOGGER, "x: %f", pose.pose.orientation.x);
    RCLCPP_INFO(LOGGER, "y: %f", pose.pose.orientation.y);
    RCLCPP_INFO(LOGGER, "z: %f", pose.pose.orientation.z);
    RCLCPP_INFO(LOGGER, "w: %f", pose.pose.orientation.w);
    RCLCPP_INFO(LOGGER, "*****");
  }

  void haptic_callback(const geometry_msgs::msg::PoseStamped &msg) {

    // this->mutex.lock();
    // geometry_msgs::msg::PoseStamped starting_pose = move_group->getCurrentPose();
    // this->mutex.unlock();
    // printPose(starting_pose);

    // return;


    // auto current_time = std::chrono::high_resolution_clock::now();

    // if(!&last_processed_pose) {
    //   last_processed_pose = current_time;
    // }
    
    // auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_processed_pose).count();

    // if(elapsed < 5000) {
    //   return;
    // }

    // RCLCPP_INFO(LOGGER, "Mas de 5 segundos");

    // last_processed_pose = current_time;

    // if(!last_processed_pose && std::chrono::duration_cast<std::chrono::seconds>(current_time - *last_processed_pose).count() < 1) {
    //   return;
    // }

    // RCLCPP_INFO(LOGGER, "Mas de 1 segundo");

    // last_processed_pose = &current_time;

    
    RCLCPP_INFO(LOGGER, "Antes del stop");
    move_group->stop();
    RCLCPP_INFO(LOGGER, "Despues del stop");

    this->mutex.lock();

    double haptic_range_x = HAPTIC_X_MAX - HAPTIC_X_MIN;
    double haptic_range_y = HAPTIC_Y_MAX - HAPTIC_Y_MIN;
    double haptic_range_z = HAPTIC_Z_MAX - HAPTIC_Z_MIN;

    double tiago_range_x = TIAGO_X_MAX - TIAGO_X_MIN;
    double tiago_range_y = TIAGO_Y_MAX - TIAGO_Y_MIN;
    double tiago_range_z = TIAGO_Z_MAX - TIAGO_Z_MIN;

    // NewValue = (((OldValue - OldMin) * NewRange) / OldRange) + NewMin

    geometry_msgs::msg::PoseStamped randomPose = move_group->getRandomPose();

    randomPose.pose.orientation.x = -0.5;
    randomPose.pose.orientation.y = 0.5;
    randomPose.pose.orientation.z = 0.5;
    randomPose.pose.orientation.w = 0.5;

    randomPose.pose.position.y = (((msg.pose.position.x - HAPTIC_X_MIN) * tiago_range_x) / haptic_range_x) + TIAGO_X_MIN;
    randomPose.pose.position.z = (((msg.pose.position.y - HAPTIC_Y_MIN) * tiago_range_y) / haptic_range_y) + TIAGO_Y_MIN;
    randomPose.pose.position.x = (((msg.pose.position.z - HAPTIC_Z_MIN) * tiago_range_z) / haptic_range_z) + TIAGO_Z_MIN;

    printPose(randomPose);

    move_group->setPoseTarget(randomPose);

    move_group->asyncMove();

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
