
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "quadruped_controller/leg.hpp"
#include "quadruped_msgs/msg/quadruped_control.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

class InverseKinematicsTest : public rclcpp::Node {
public:
  InverseKinematicsTest() : Node("leg_controller_node") {
    subscription_ =
        this->create_subscription<quadruped_msgs::msg::QuadrupedControl>(
            "control_quadruped", 10,

            std::bind(&InverseKinematicsTest::control_callback, this,
                      std::placeholders::_1));

    control_joint_state_pub_ =
        this->create_publisher<sensor_msgs::msg::JointState>(
            "control_joint_states", 10);

    trajectory_pub_ =
        this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/joint_trajectory_controller/joint_trajectory", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&InverseKinematicsTest::timer_callback, this));

    for (const auto &leg_name : legs_names) {
      quadruped_controller::Leg leg(leg_name);
      legs.push_back(leg);
    }
  }

private:
  void control_callback(const quadruped_msgs::msg::QuadrupedControl msg) {
    using Point = geometry_msgs::msg::Point;
    

    auto passive_joint_state = sensor_msgs::msg::JointState();
    passive_joint_state.header.stamp = msg.header.stamp;

    std::vector<Point> reference_foot_positions = {
        msg.fl_foot_position, msg.fr_foot_position, msg.rl_foot_position,
        msg.rr_foot_position};

    trajectory_msgs::msg::JointTrajectoryPoint point;
    auto l = joint_trajectory.points.size();
    double sec= (l+1)* 0.05;
    point.time_from_start = rclcpp::Duration(0, sec*1e9);

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

      std::cout << i << ": Reference: " << x.transpose() << std::endl;
      std::cout << i << ": Inverse kinematics: " << q.transpose() << std::endl;
      std::cout << i << ": Forward kinematics: " << foot_position.transpose()
                << std::endl;

      auto joint_states = leg.get_active_joint_states();

      for (const auto &joint_state : joint_states) {
        if (joint_trajectory.joint_names.size() < 12) {
          joint_trajectory.joint_names.push_back(joint_state.name);
        }
        point.positions.push_back(joint_state.position);
      }

      for (const auto &joint_state : joint_states) {
        passive_joint_state.name.push_back(joint_state.name);
        passive_joint_state.position.push_back(0.0 /*joint_state.position*/);
        passive_joint_state.velocity.push_back(joint_state.velocity);
        passive_joint_state.effort.push_back(joint_state.effort);
      }
    }

    joint_trajectory.points.push_back(point);

    control_joint_state_pub_->publish(passive_joint_state);
  }

  void timer_callback() {
    if (joint_trajectory.points.size() < 40) {
      return;
    }

    joint_trajectory.header.stamp = now();
    trajectory_pub_->publish(joint_trajectory);
    joint_trajectory.points.clear();
  }

  rclcpp::Subscription<quadruped_msgs::msg::QuadrupedControl>::SharedPtr
      subscription_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
      control_joint_state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
      position_control_joint_state_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr
      trajectory_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::vector<quadruped_controller::Leg> legs;
  std::vector<std::string> legs_names = {"front_left", "front_right",
                                         "rear_left", "rear_right"};

  trajectory_msgs::msg::JointTrajectory joint_trajectory;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InverseKinematicsTest>());
  rclcpp::shutdown();
  return 0;
}
