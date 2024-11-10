
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "quadruped_controller/leg.hpp"
#include "quadruped_msgs/msg/quadruped_control.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/transform_broadcaster.h"

class InverseKinematicsTest : public rclcpp::Node {
public:
  InverseKinematicsTest() : Node("leg_controller_node") {
    subscription_ =
        this->create_subscription<quadruped_msgs::msg::QuadrupedControl>(
            "control_quadruped", 10,

            std::bind(&InverseKinematicsTest::control_callback, this,
                      std::placeholders::_1));

    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "joint_states", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    for (const auto &leg_name : legs_names) {
      quadruped_controller::Leg leg(leg_name);
      legs.push_back(leg);
    }
  }

private:
  void control_callback(const quadruped_msgs::msg::QuadrupedControl msg) {
    using Point = geometry_msgs::msg::Point;
    std::vector<Eigen::Vector3d> foot_positions;

    auto passive_joint_state = sensor_msgs::msg::JointState();
    passive_joint_state.header.stamp = msg.header.stamp;

    std::vector<Point> reference_foot_positions = {
        msg.fl_foot_position, msg.fr_foot_position, msg.rl_foot_position,
        msg.rr_foot_position};

    for (std::size_t i = 0; i < legs.size(); ++i) {
      auto &leg = legs[i];
      auto &reference_foot_position = reference_foot_positions[i];

      Eigen::Vector3d x;
      x << reference_foot_position.x, reference_foot_position.y,
          reference_foot_position.z;

      Eigen::Vector3d help;
      auto q = leg.inverse_kinematics(x);
      leg.set_joints_states(q);
      auto foot_position = leg.forward_kinematics();
      foot_positions.push_back(foot_position);


      std::cout << i<< ": Reference: " << x.transpose() << std::endl;
      std::cout << i<< ": Inverse kinematics: " << q.transpose() << std::endl;
      std::cout << i<< ": Forward kinematics: "
                << foot_position.transpose() << std::endl;

      auto joint_states = leg.get_joints_states();

      for (const auto &joint_state : joint_states) {
        passive_joint_state.name.push_back(joint_state.name);
        passive_joint_state.position.push_back(joint_state.position);
        passive_joint_state.velocity.push_back(joint_state.velocity);
        passive_joint_state.effort.push_back(joint_state.effort);
      }
    }
    
    for (std::size_t i = 0; i < legs.size(); ++i) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp = now();
      transform.header.frame_id = legs_names[i] + "_first_link";
      transform.child_frame_id = legs_names[i] + "_foot";

      transform.transform.translation.x = foot_positions[i](0);
      transform.transform.translation.y = foot_positions[i](1);
      transform.transform.translation.z = foot_positions[i](2);

      tf_broadcaster_->sendTransform(transform);
    }

    publisher_->publish(passive_joint_state);
  }

  rclcpp::Subscription<quadruped_msgs::msg::QuadrupedControl>::SharedPtr
      subscription_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
      position_publisher_;
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
