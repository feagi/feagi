cmake_minimum_required(VERSION 3.5)
project(freenove_4wd_car_description)

find_package(ament_cmake REQUIRED)
find_package(image_transport REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ros_ign_bridge REQUIRED)
find_package(std_msgs REQUIRED)

find_package(ignition-transport8 REQUIRED)
  set(IGN_TRANSPORT_VER ${ignition-transport8_VERSION_MAJOR})

find_package(ignition-msgs5 REQUIRED)
set(IGN_MSGS_VER ${ignition-msgs5_VERSION_MAJOR})


set(dependencies
  "image_transport"
  "ros_ign_bridge"
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "sensor_msgs"
)

set(executable
  image_bridge
)

add_executable(${executable}
  src/image_bridge.cpp
)

target_link_libraries(${executable}
  ${catkin_LIBRARIES}
  ignition-msgs${IGN_MSGS_VER}::core
  ignition-transport${IGN_TRANSPORT_VER}::core
)

ament_target_dependencies(${executable}
  "image_transport"
  "ros_ign_bridge"
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
  src/router.py
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  src/configuration.py
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  src/ultrasonic.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()

