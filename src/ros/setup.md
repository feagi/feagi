# ROS <--> FEAGI using Sonar sensor on the local interface
## Local build
To set up with everything at once to run sonar_sensor:
1) Plug the UNO arduino board in.
2) Type it in your terminal, `./test.sh` in the `feagi-core/src/ros/`

# ROS <--> FEAGI turtlebot3/LIDAR inteface

## **Local build**

To build the turtlebot3 and ROS workspaces necessary for running the turtlebot3/LIDAR demonstration locally on your machine (Ubuntu 20.04), run the shell script (`ws_setup.sh`) found in `feagi-core/src/>

Next, open a new terminal window and run:
- `$ ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py`

This will open a Gazebo world complete with turtlebot and obstacles. To start the laser scan interface, open another (new) terminal window and run:
- `$ ros2 run py_topic ros_laser_scan`

If you want to control the turtlebot via keyboard, run (in another new terminal window):
- `$ ros2 run turtlebot3_teleop teleop_keyboard`
- **Note:** this terminal window must be the active/top-level window on the display for keyboard input to control the turtlebot.
___
## **Containerized build**

This build deploys FEAGI and ROS as networked Docker containers, which eliminates the need for manual installation and configuration. In the `feagi-core/src/ros/` directory, run:
- `$ docker-compose build`

Once the build successfully completes, run:
- `$ docker-compose up`

To access the VNC ros-foxy desktop, open a web browser and, in the address bar, enter:
- `http://127.0.0.1:6080`

Run the commands listed above in separate terminal windows to launch the demonstration in the virtual desktop.
