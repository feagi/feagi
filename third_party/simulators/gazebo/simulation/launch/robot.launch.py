"""
Copyright 2016-2022 The FEAGI Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================
"""
# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    pkg_ros_ign_gazebo_demos = get_package_share_directory('simulation')
    pkg_ros_ign_gazebo = get_package_share_directory('simulation')

    ign_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py')),
        launch_arguments={
            'ign_args': '-r environments/free_world.sdf'
        }.items(),
    )
    
    # RQt
    rqt = Node(
        package='rqt_plot',
        executable='rqt_plot',
        # FIXME: Why isn't the topic being populated on the UI? RQt issue?
        arguments=['--force-discover',
                   '/model/freenove_smart_car/battery/linear_battery/state'],
        condition=IfCondition(LaunchConfiguration('rqt'))
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_ros_ign_gazebo_demos, 'rviz', 'freenove_smart_car.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    
        # RQt
    rqt = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        arguments=['/camera'],
        condition=IfCondition(LaunchConfiguration('rqt'))
    )

    # Bridge
    bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=['/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU',
                   '/x3_uav/gazebo/command/twist@geometry_msgs/msg/Twist@ignition.msgs.Twist',
                   '/ultrasonic0@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
                   '/Camera0/image@sensor_msgs/msg/Image@ignition.msgs.Image',
                   '/IR1/image@sensor_msgs/msg/Image@ignition.msgs.Image',
                   '/IR2/image@sensor_msgs/msg/Image@ignition.msgs.Image',
                   '/IR0/image@sensor_msgs/msg/Image@ignition.msgs.Image',
                   '/S0@std_msgs/msg/Float64@ignition.msgs.Double',
                   '/S1@std_msgs/msg/Float64@ignition.msgs.Double',
                   '/M0@std_msgs/msg/Float64@ignition.msgs.Double',
                   '/M1@std_msgs/msg/Float64@ignition.msgs.Double',
                   '/M2@std_msgs/msg/Float64@ignition.msgs.Double',
                   '/M3@std_msgs/msg/Float64@ignition.msgs.Double',
                   '/model/freenove_smart_car/battery/linear_battery/state@sensor_msgs/msg/BatteryState@ignition.msgs.BatteryState'],
        output='screen'
    )

    #controller
    robot_interface = Node(
        package='simulation',
        executable='robot.py',
        # FIXME: Why isn't the topic being populated on the UI? RQt issue?
        output='screen',
    )

    return LaunchDescription([
        ign_gazebo,
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Open RViz.'),
	DeclareLaunchArgument('rqt', default_value='true',
                              description='Open RQt.'),
        bridge,
        #rviz,
        #rqt
        robot_interface
    ])

