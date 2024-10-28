
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "quadruped_controller/leg.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/transform_broadcaster.h"

class EffectorJointStatePublisher : public rclcpp::Node {
public:
  EffectorJointStatePublisher() : Node("leg_controller_node") {
    subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10,
        std::bind(&EffectorJointStatePublisher::joint_state_callback, this,
                  std::placeholders::_1));
    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "joint_states", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

private:
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (msg->name[0] != "front_left_first_joint") {
      return;
    }
    std::vector<std::string> legs_names = {"front_left", "front_right",
                                           "rear_left", "rear_right"};

    std::vector<quadruped_controller::Leg> legs;
    std::vector<std::pair<quadruped_controller::JointState, quadruped_controller::JointState>> passive_knee_joints;
    std::vector<Eigen::Vector3d> effector_positions;

    for (const auto &leg_name : legs_names) {
      quadruped_controller::Leg leg(leg_name);
      leg.get_joint_states(msg);
      legs.push_back(leg);

      effector_positions.push_back(leg.forward_kinematics());
      passive_knee_joints.push_back(leg.get_passive_knee_joints());
    }

    auto calculated_msg = sensor_msgs::msg::JointState();
    calculated_msg.header.stamp = this->now();

    for(const auto &passive_knee_joint : passive_knee_joints) {
      calculated_msg.name.push_back(passive_knee_joint.first.name);
      calculated_msg.position.push_back(passive_knee_joint.first.position);
      calculated_msg.velocity.push_back(passive_knee_joint.first.velocity);
      calculated_msg.effort.push_back(passive_knee_joint.first.effort);

      calculated_msg.name.push_back(passive_knee_joint.second.name);
      calculated_msg.position.push_back(passive_knee_joint.second.position);
      calculated_msg.velocity.push_back(passive_knee_joint.second.velocity);
      calculated_msg.effort.push_back(passive_knee_joint.second.effort);
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

  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EffectorJointStatePublisher>());
  rclcpp::shutdown();
  return 0;
}
