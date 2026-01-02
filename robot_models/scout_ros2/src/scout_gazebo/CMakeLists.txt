cmake_minimum_required(VERSION 3.5)
project(scout_gazebo)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find ament_cmake
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install Gazebo worlds
install(
  DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}/
)

# Install Gazebo URDF/Xacro files
install(
  DIRECTORY gazebo_urdf
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()
