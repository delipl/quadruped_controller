cmake_minimum_required(VERSION 3.5)
project(quadruped_controller)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package (Eigen3 3.3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(quadruped_controller src/controller.cpp)

ament_target_dependencies(quadruped_controller
  rclcpp
  sensor_msgs
  trajectory_msgs
  control_msgs
  rclcpp_action
  Eigen3
  geometry_msgs
  tf2_ros
)

add_executable(effector_joint_state_publisher src/leg.cpp src/effector_joint_state_publisher.cpp)
target_link_libraries (effector_joint_state_publisher Eigen3::Eigen)

target_include_directories(effector_joint_state_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(effector_joint_state_publisher rclcpp sensor_msgs    geometry_msgs
tf2_ros)

add_executable(inverse_kinematics_test src/leg.cpp src/inverse_kinematics_test.cpp)
target_link_libraries (inverse_kinematics_test Eigen3::Eigen)

target_include_directories(inverse_kinematics_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(inverse_kinematics_test rclcpp sensor_msgs    geometry_msgs
tf2_ros)


install(TARGETS
  quadruped_controller effector_joint_state_publisher inverse_kinematics_test

  DESTINATION lib/${PROJECT_NAME})

ament_package()