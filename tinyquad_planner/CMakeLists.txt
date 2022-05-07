cmake_minimum_required(VERSION 3.0.2)
project(tinyquad_planner)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  gazebo_ros
  gazebo_ros_control
  joint_state_publisher
  robot_state_publisher
  roscpp
  rospy
  rviz
  xacro
  kdl_parser
  tf
  sensor_msgs
  std_msgs
  trac_ik_lib
  visualization_msgs
  geometry_msgs
)
find_package(Boost REQUIRED COMPONENTS date_time)
find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
   CATKIN_DEPENDS controller_manager gazebo_ros gazebo_ros_control joint_state_publisher robot_state_publisher roscpp rospy rviz xacro trac_ik_lib
   DEPENDS Boost orocos_kdl
   DEPENDS EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(tq_ik_test_node src/tinyquad_ik_test.cpp)
target_link_libraries(tq_ik_test_node ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

add_executable(bezier_node src/bezier_node.cpp src/bezier_generator/bezier_generator.cpp)
target_link_libraries(bezier_node ${catkin_LIBRARIES})

add_executable(leg_kinematics_node src/legKinematics/legKinematics.cpp src/legKinematics_node.cpp)
target_link_libraries(leg_kinematics_node ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

add_executable(body_kinematics_node src/body_kinematics/BodyKinematics.cpp src/BodyKinematics_node.cpp)
target_link_libraries(body_kinematics_node ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

add_executable(kinematics_test_node src/leg_kinematics_test/leg_kinemtics_test.cpp)
target_link_libraries(kinematics_test_node ${catkin_LIBRARIES})
