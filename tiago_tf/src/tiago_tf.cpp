#include "rclcpp/rclcpp.hpp"
#include "tiago_tf_msgs/srv/tiago_tf.hpp"

#include <memory>

void calculate(const std::shared_ptr<tiago_tf_msgs::srv::TiagoTf::Request> request,
          std::shared_ptr<tiago_tf_msgs::srv::TiagoTf::Response> response)
{
  response->pose = request->pose;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("tiago_tf_server");

  rclcpp::Service<tiago_tf_msgs::srv::TiagoTf>::SharedPtr service =
    node->create_service<tiago_tf_msgs::srv::TiagoTf>("tiago_tf", &calculate);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to tfring");

  rclcpp::spin(node);
  rclcpp::shutdown();
}