#ifndef QUADRUPED_CONTROLLER_LEG_HPP
#define QUADRUPED_CONTROLLER_LEG_HPP

#include <cmath>
#include <limits>

#include <Eigen/Dense>

#include <sensor_msgs/msg/joint_state.hpp>

namespace quadruped_controller {

struct JointState {
  std::string name;
  double position;
  double velocity;
  double effort;
};

const double UPPER_BONE_LENGTH = 0.15;
const double LOWER_BONE_LENGTH = 0.35;
const double FOOT_LENGTH = 0.05;

class Leg {
public:
  Leg(const std::string &name);

  void get_joint_states(const sensor_msgs::msg::JointState::SharedPtr msg);

  Eigen::Vector3d forward_kinematics(const Eigen::Vector3d &q);
  Eigen::Vector3d forward_kinematics() {
    return forward_kinematics(
        Eigen::Vector3d(adduction.position, hip_.position, knee_.position));
  }

  Eigen::Vector3d invert_kinematics(const Eigen::Vector3d &x);

  std::pair<JointState, JointState> get_passive_knee_joints() const {
    return {passive_knee_front_, passive_knee_rear_};
  }

  double get_distance_to_effector() const { return distance_to_effector_; }

private:
  const double l1 = UPPER_BONE_LENGTH;
  const double l2 = UPPER_BONE_LENGTH;
  const double l3 = LOWER_BONE_LENGTH;
  const double l4 = LOWER_BONE_LENGTH;
  const double l5 = FOOT_LENGTH;

  JointState adduction;
  JointState hip_;
  JointState knee_;
  JointState passive_knee_front_;
  JointState passive_knee_rear_;

  void calculate_passive_joints(const Eigen::Vector3d &effector_position,
                                double th4);
  Eigen::Vector3d get_effector_position(double q3);
  Eigen::Matrix4d denavite_hartenberg(double alpha, double a, double d,
                                      double theta);
  double distance_to_effector_;
};

} // namespace quadruped_controller

#endif // QUADRUPED_CONTROLLER_LEG_HPP
