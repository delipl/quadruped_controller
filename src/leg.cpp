
#include "quadruped_controller/leg.hpp"

#include <iostream>

namespace quadruped_controller {
Leg::Leg(const std::string &name) {
  adduction.name = name + "_adduction_joint";
  hip_.name = name + "_hip_joint";
  knee_.name = name + "_knee_joint";

  passive_knee_front_.name = name + "_thigh_to_tibia_front_joint";
  passive_knee_rear_.name = name + "_thigh_to_tibia_back_joint";
  //   adduction.name = name + "_hip_side";
  //   hip_.name = name + "_hip_forward";
  //   knee_.name = name + "_knee";
}

void Leg::get_joint_states(const sensor_msgs::msg::JointState::SharedPtr msg) {
  for (size_t i = 0; i < msg->name.size(); ++i) {
    if (msg->name[i] == adduction.name) {
      adduction.position = msg->position[i];
      //   std::cout << "Hip side position: " << adduction.position <<
      //   std::endl; adduction.velocity = msg->velocity[i]; adduction.effort =
      //   msg->effort[i];
    } else if (msg->name[i] == hip_.name) {
      hip_.position = msg->position[i];
      //   std::cout << "Hip forward position: " << hip_.position
      //             << std::endl;
      //   hip_.velocity = msg->velocity[i];
      //   hip_.effort = msg->effort[i];
    } else if (msg->name[i] == knee_.name) {
      knee_.position = msg->position[i];

      //   std::cout << "Knee position: " << knee_.position << std::endl;
      //   knee_.velocity = msg->velocity[i];
      //   knee_.effort = msg->effort[i];
    }
  }
}

Eigen::Matrix4d Leg::denavite_hartenberg(double theta, double d, double a,
                                         double alpha) {
  Eigen::Matrix4d T;

  T << cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha),
      a * cos(theta), sin(theta), cos(theta) * cos(alpha),
      -cos(theta) * sin(alpha), a * sin(theta), 0, sin(alpha), cos(alpha), d, 0,
      0, 0, 1;

  return T;
}

Eigen::Vector3d Leg::forward_kinematics(const Eigen::Vector3d &q) {
  const auto d1 = 0.1;
  auto effector_position = get_effector_position(q(2));

  auto A01 = denavite_hartenberg(0.0, 0.0, 0.13, q(0));
  auto A12 = denavite_hartenberg(M_PI_2, 0.0, 0.2, q(1));

  Eigen::Vector4d effector_position_homogeneous;
  effector_position_homogeneous << effector_position(0), effector_position(1),
      effector_position(2), 1;
  Eigen::Vector4d nono;
  nono << 0, 0, 0, 1;
  return (A01 * A12 * effector_position_homogeneous).head(3);
}

// Page 87 of
// http://160592857366.free.fr/joe/ebooks/Mechanical%20Engineering%20Books%20Collection/THEORY%20OF%20MACHINES/machines%20and%20mechanisms.pdf
Eigen::Vector3d Leg::get_effector_position(double q3) {
  const double th2 = q3;
  const auto c2 = std::cos(th2);
  const auto s2 = std::sin(th2);

  const auto BD = std::sqrt(l1 * l1 + l2 * l2 - 2 * l1 * l2 * c2);
  const auto gamma = std::acos((l3 * l3 + l4 * l4 - BD * BD) / (2 * l3 * l4));
  const auto sg = std::sin(gamma);
  const auto cg = std::cos(gamma);

  const auto th3 =
      2 * std::atan2(-l2 * s2 + l4 * sg, l1 + l3 - l2 * c2 - l4 * cg);
  const auto th4 =
      2 * std::atan2(l2 * s2 - l3 * sg, l2 * c2 + l4 - l1 - l3 * cg);

  const auto c3 = std::cos(th3);
  const auto s3 = std::sin(th3);
  const auto c4 = std::cos(th4);
  const auto s4 = std::sin(th4);

  const auto xe1 = l2 * c2 + l3 * c3;
  const auto ye1 = l2 * s2 + l3 * s3;

  const auto nan = std::numeric_limits<double>::quiet_NaN();

  const auto x = xe1 + l5 * c3;
  const auto y = ye1 + l5 * s3;

  calculate_passive_joints({0.0, xe1, ye1}, th4);

  // For the origin frame x and y is at plate on up Z axis
  return {0.0, x, y};
}

void Leg::calculate_passive_joints(const Eigen::Vector3d &effector_position,
                                   double th4) {
  const auto e = effector_position.norm();
  const auto cos = (l3 * l3 + l2 * l2 - e * e) / (2 * l2 * l3);
  const auto alpha = std::acos(cos);

  const auto beta = M_PI - th4;
  passive_knee_rear_.position = alpha - M_PI;
  passive_knee_front_.position = M_PI - beta;
}

} // namespace quadruped_controller
