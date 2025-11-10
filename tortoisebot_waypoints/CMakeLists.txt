cmake_minimum_required(VERSION 3.0.2)
project(tortoisebot_waypoints)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  actionlib
  tortoisebot_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/waypoints_test.test)
endif()

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs nav_msgs actionlib tortoisebot_msgs
)