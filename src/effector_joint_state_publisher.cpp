
#include "quadruped_controller/leg.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

class EffectorJointStatePublisher : public rclcpp::Node {
public:
  EffectorJointStatePublisher() : Node("leg_controller_node") {
    subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10,
        std::bind(&EffectorJointStatePublisher::joint_state_callback, this,
                  std::placeholders::_1));
    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "joint_states", 10);
tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

private:
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (msg->name[0] != "front_left_adduction_joint") {
      return;
    }
    quadruped_controller::Leg leg("front_left");
    leg.get_joint_states(msg);

    auto effector_position = leg.forward_kinematics();

    auto [front, rear] = leg.get_passive_knee_joints();
    auto calculated_msg = sensor_msgs::msg::JointState();
    calculated_msg.header.stamp = this->now();
    calculated_msg.name = {
        front.name,
        rear.name,
    };
    calculated_msg.position = {front.position, rear.position};
    calculated_msg.velocity = {front.velocity, rear.velocity};
    calculated_msg.effort = {front.effort, rear.effort};

    publisher_->publish(calculated_msg);

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = this->now();
    transform.header.frame_id = "front_left_adduction_motor_link";
    transform.child_frame_id = "front_left_foot";

    transform.transform.translation.x = effector_position(0);
    transform.transform.translation.y = effector_position(1);
    transform.transform.translation.z = effector_position(2);

    tf_broadcaster_->sendTransform(transform);
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
