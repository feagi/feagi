cmake_minimum_required(VERSION 3.5)
project(cobot_arm)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Install Python modules
ament_python_install_package(src)

# Install Python executables
install(PROGRAMS
  src/mycobot.py
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  src/configuration.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()

