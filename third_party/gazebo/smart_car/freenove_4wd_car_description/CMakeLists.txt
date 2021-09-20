cmake_minimum_required(VERSION 3.5)
project(freenove_4wd_car_description)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)


set(dependencies
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "sensor_msgs"
)


install(
  DIRECTORY
    launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  DIRECTORY
    rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

install(
  DIRECTORY
    models/
  DESTINATION share/${PROJECT_NAME}/models
)

# Install Python modules
ament_python_install_package(src/)


# Install Python executables
install(PROGRAMS
  src/controller.py
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  src/range.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()

