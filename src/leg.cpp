
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
    z_axis_q1_direction_ = 1.0;
    z_axis_q2_direction_ = -1.0;
    passive_side_multiplier_ = -1.0;
  }

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
                          z_axis_q1_direction_ * (0.023 + 0.0555), -l1, 0.0);
  auto A23 = denavite_hartenberg(z_axis_q2_direction_ * fifth_.position,
                                 z_axis_q1_direction_ * 0.018, l4 + l5, 0.0);

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

  const auto x = xe1 + l5 * c3;
  const auto y = ye1 + l5 * s3;

  fifth_.position = -passive_side_multiplier_ * (M_PI - th2 + th3);
  forth_.position = passive_side_multiplier_ * (M_PI - th4);
}

// Szrek PhD thesis
Eigen::Vector3d Leg::inverse_kinematics(const Eigen::Vector3d &x) {
  Eigen::Vector3d q;

  const double xe = x(0);
  const double ye = x(1);

  const double l_BE = l4 + l5;
  const double l_AB = l1;

  const double xe2 = xe * xe;
  const double ye2 = ye * ye;

  const double Q = (l_AB * l_AB - l_BE * l_BE + ye2 + xe2) / (2 * ye);
  const double W = Q * Q - l_AB * l_AB;
  const double T = -(2 * Q * xe) / (ye);
  const double V = 1 + (xe2) / (ye2);

  const double delta = T * T - 4 * V * W;
  const double sqrt_delta = std::sqrt(delta);

  // FIXME: It cannot derive y < 0.0
  const double yb1 = (-T + sqrt_delta) / (2 * V);
  const double yb2 = (-T - sqrt_delta) / (2 * V);
  const double xb1 = std::sqrt(l_AB * l_AB - yb1 * yb1);
  const double xb2 = std::sqrt(l_AB * l_AB - yb2 * yb2);


  double xb = yb1;
  double yb = xb1;

  const Eigen::Vector2d b = {xb, yb};
  const Eigen::Vector2d e = {xe, ye};
  const Eigen::Vector2d be = e - b;

  const double gamma = std::atan2(be(1), (be(0)));
  const double l_BC = l4;

  Eigen::Vector2d C =
      b + Eigen::Vector2d{std::cos(gamma) * l_BC, std::sin(gamma) * l_BC};

  const auto theta_b = std::atan2(b(1), b(0));
  const auto c = C.norm();
  const auto alpha = std::acos((c * c + l2 * l2 - l3 * l3) / (2 * c * l2));
  const auto beta = std::acos((c * c + l1 * l1 - l4 * l4) / (2 * c * l1));

  q(0) = 0.0;
  q(1) = z_axis_q1_direction_* theta_b ;
  q(2) = z_axis_q2_direction_ *(M_PI - beta - alpha);
  return q;
}

} // namespace quadruped_controller

// ros2 topic pub /effector_position  geometry_msgs/msg/PoseStamped  "{ header:
// {stamp: now,  frame_id: front_left_second_link},  pose: { position: { x:
// 0.064, y: 0.2, z: 0.0785 } }}"