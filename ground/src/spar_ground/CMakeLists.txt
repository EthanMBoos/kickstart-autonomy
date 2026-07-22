cmake_minimum_required(VERSION 3.16)
# C, not just CXX: rosidl_generate_interfaces below also builds a C typesupport.
project(spar_ground C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
find_package(behaviortree_cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Detection.msg"
  DEPENDENCIES std_msgs geometry_msgs)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# The tree engine is BehaviorTree.CPP (the same library Nav2 itself runs
# on); this package only supplies the node types and the tree it composes
# them into. The executive ticks it, plus the leaves that talk to Nav2.
add_executable(bt_executive
  src/bt_executive.cpp
  src/leaves/navigate_leaf.cpp)
target_include_directories(bt_executive PRIVATE src)
target_link_libraries(bt_executive PRIVATE
  behaviortree_cpp::behaviortree_cpp
  ament_index_cpp::ament_index_cpp
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  tf2_ros::tf2_ros
  ${nav2_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${std_msgs_TARGETS}
  "${cpp_typesupport_target}")

# Perception: the simplest detector that can support a behavior.
add_executable(anomaly_detector src/anomaly_detector.cpp)
target_link_libraries(anomaly_detector PRIVATE
  rclcpp::rclcpp
  cv_bridge::cv_bridge
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
  opencv_core
  opencv_imgproc
  ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  "${cpp_typesupport_target}")

add_executable(battery_sim src/battery_sim.cpp)
target_link_libraries(battery_sim PRIVATE
  rclcpp::rclcpp
  tf2_ros::tf2_ros
  ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${std_msgs_TARGETS})

install(TARGETS bt_executive battery_sim anomaly_detector
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY behavior_trees/ DESTINATION share/${PROJECT_NAME}/behavior_trees)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # BT.CPP's own engine is tested upstream; these cover the logic that's
  # still ours: staleness, hysteresis, cooldowns.
  ament_add_gtest(test_bt_nodes test/test_bt_nodes.cpp)
  target_include_directories(test_bt_nodes PRIVATE src)
  target_link_libraries(test_bt_nodes
    behaviortree_cpp::behaviortree_cpp
    ${geometry_msgs_TARGETS})
endif()

ament_package()
