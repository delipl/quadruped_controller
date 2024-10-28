
#include "quadruped_controller/leg.hpp"

#include <iostream>

namespace quadruped_controller {
Leg::Leg(const std::string &name) {

  if (name == "front_left") {
    z_axis_q1_direction_ = 1.0;
    z_axis_q2_direction_ = -1.0;
    passive_side_multiplier_ = -1.0;

  } else if (name == "front_right") {
    z_axis_q1_direction_ = -1.0;
    z_axis_q2_direction_ = 1.0;
    passive_side_multiplier_ = 1.0;
  } else if (name == "rear_left") {
    z_axis_q1_direction_ = -1.0;
    z_axis_q2_direction_ = 1.0;
    passive_side_multiplier_ = 1.0;
  } else if (name == "rear_right") {
    z_axis_q1_direction_ = -1.0;
    z_axis_q2_direction_ = 1.0;
    passive_side_multiplier_ = -1.0;
  }

  // if (name == "rear_right") {
  //   z_axis = -1.0;
  // }

  first_.name = name + "_first_joint";
  second_.name = name + "_second_joint";
  third_.name = name + "_third_joint";

  forth_.name = name + "_forth_joint";
  fifth_.name = name + "_fifth_joint";
}

void Leg::get_joint_states(const sensor_msgs::msg::JointState::SharedPtr msg) {
  for (size_t i = 0; i < msg->name.size(); ++i) {
    if (msg->name[i] == first_.name) {
      first_.position = msg->position[i];
      //   std::cout << "Hip side position: " << first_.position <<
      //   std::endl; first_.velocity = msg->velocity[i]; first_.effort =
      //   msg->effort[i];
    } else if (msg->name[i] == second_.name) {
      second_.position = msg->position[i];
      //   std::cout << "Hip forward position: " << second_.position
      //             << std::endl;
      //   second_.velocity = msg->velocity[i];
      //   second_.effort = msg->effort[i];
    } else if (msg->name[i] == third_.name) {
      third_.position = msg->position[i];

      //   std::cout << "Knee position: " << third_.position << std::endl;
      //   third_.velocity = msg->velocity[i];
      //   third_.effort = msg->effort[i];
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

  update_effector_position(q(2));

  // TODO: warning this pi will be negative in another side
  auto A01 = denavite_hartenberg(0.0, 0.0, 0.031 + 0.064, q(0));
  auto A12 =
      denavite_hartenberg(z_axis_q1_direction_ * (q(1) + M_PI),
                          z_axis_q1_direction_ * (0.023 + 0.0555), -0.125, 0.0);
  auto A23 = denavite_hartenberg(z_axis_q2_direction_ * fifth_.position,
                                 z_axis_q1_direction_ * 0.018, 0.28, 0.0);

  Eigen::Vector4d versor;
  versor << 0, 0, 0, 1;
  return (A01 * A12 * A23 * versor).head(3);
}

// Page 87 of
// http://160592857366.free.fr/joe/ebooks/Mechanical%20Engineering%20Books%20Collection/THEORY%20OF%20MACHINES/machines%20and%20mechanisms.pdf
void Leg::update_effector_position(double q3) {
  const double th2 = M_PI - passive_side_multiplier_ * q3;
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


  fifth_.position = -passive_side_multiplier_ * (M_PI - th2 + th3);
  forth_.position = passive_side_multiplier_ * (M_PI - th4);
}

// https://mecheng.iisc.ac.in/~asitava/NPTEL/module4.pdf
//  page 50
Eigen::Vector3d Leg::inverse_kinematics(const Eigen::Vector3d &x) {
  Eigen::Vector3d q;

  const double xe = x(0);
  const double ye = x(1);

  const auto e = std::sqrt(xe * xe + ye * ye);

  const auto cos_theta1_gamma =
      std::cos((l3 * l3 - l1 * l1 - e * e) / (2 * l1 * e));

  const auto beta = std::atan2(ye, xe);
  const auto gamma = 2 * M_PI - beta;
  const auto theta1 = gamma + std::acos(cos_theta1_gamma);

  const auto cos_alpha = (l1 * l1 - e * e + l3 * l3) / (2 * l1 * l3);

  q(2) = std::acos((2 * l1 * l1 - 2 * l1 * l3 * cos_alpha) / (2 * e * l1));
  q(1) = 2 * M_PI - theta1 + q(2);
  q(0) = 0.0;

  return q;
}

} // namespace quadruped_controller
