
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "quadruped_controller/leg.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/transform_broadcaster.h"

class InverseKinematicsTest : public rclcpp::Node {
public:
  InverseKinematicsTest() : Node("leg_controller_node") {
    subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "effector_position", 10,
        std::bind(&InverseKinematicsTest::position_callback, this,
                  std::placeholders::_1));

    position_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "effector_position_help", 10);
    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "joint_states", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    for (const auto &leg_name : legs_names) {
      quadruped_controller::Leg leg(leg_name);
      legs.push_back(leg);
    }
  }

private:
  void position_callback(const geometry_msgs::msg::PoseStamped pose) {
    std::vector<Eigen::Vector3d> effector_positions;
    std::vector<Eigen::Vector3d> helpers;
    auto effector_position = pose.pose.position; 
    auto calculated_msg = sensor_msgs::msg::JointState();
    calculated_msg.header.stamp = this->now();

    for (auto &leg : legs) {
      Eigen::Vector3d x;
      x << effector_position.x, effector_position.y, effector_position.z;

      Eigen::Vector3d help;
      auto q = leg.inverse_kinematics(x);
      helpers.push_back(help);
      leg.set_joints_states(q);
      std::cout << "Input: x: " << x(0) << "\ty: " << x(1) << "\tz: " << x(2) << std::endl;
      std::cout << "Control: q1: " << q(0) << "\tq2: " << q(1)
                << "\tq3: " << q(2) << std::endl;

      effector_positions.push_back(leg.forward_kinematics());

      auto joint_states = leg.get_joints_states();

      for (const auto &joint_state : joint_states) {
        calculated_msg.name.push_back(joint_state.name);
        calculated_msg.position.push_back(joint_state.position);
        calculated_msg.velocity.push_back(joint_state.velocity);
        calculated_msg.effort.push_back(joint_state.effort);
      }
    }

    publisher_->publish(calculated_msg);

    auto now = this->now();
    for (size_t i = 0; i < legs.size(); ++i) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp = now;
      transform.header.frame_id = legs_names[i] + "_first_link";
      transform.child_frame_id = legs_names[i] + "_foot";

      transform.transform.translation.x = effector_positions[i](0);
      transform.transform.translation.y = effector_positions[i](1);
      transform.transform.translation.z = effector_positions[i](2);

      tf_broadcaster_->sendTransform(transform);
    }

    auto position_msg = geometry_msgs::msg::PoseStamped();
    position_msg.header.stamp = this->now();
    position_msg.header.frame_id = legs_names[0] + "_leg_deltoid";
    position_msg.pose.position.x = helpers[0](0);
    position_msg.pose.position.y = helpers[0](1);
    position_msg.pose.position.z = helpers[0](2);

    position_publisher_->publish(position_msg);
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr position_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::vector<quadruped_controller::Leg> legs;
  std::vector<std::string> legs_names = {"front_left", "front_right",
                                         "rear_left", "rear_right"};
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InverseKinematicsTest>());
  rclcpp::shutdown();
  return 0;
}
