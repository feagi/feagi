FROM ros:foxy AS foxy-ws
SHELL ["/bin/bash", "-c"]

# install tools
RUN groupadd -r micro_ros && \
    apt-get update && \
    apt-get install -y --no-install-recommends python3-pip wget

WORKDIR /root/
COPY ./ /root/

RUN chmod u+x start_sonar.sh && \
    cd ~  && \
    mypath=`pwd` && \
    pip3 install zmq && \
    source /opt/ros/foxy/setup.bash && \
    mkdir -p ~/ros2_ws/src && \
    cd ~/ros2_ws/src || exit && \
    ros2 pkg create --build-type ament_python py_topic && \
    sudo cp /root/ros_laser_scan.py /root/ros_teleop.py ~/ros2_ws/src/py_topic/py_topic/ && \
    sed '9i\  <buildtool_depend>ament_python</buildtool_depend>\n  <exec_depend>rclpy</exec_depend>\n  <exec_depend>geometry_msgs</exec_depend>' ~/ros2_ws/src/py_topic/package.xml > changed.txt && mv changed.txt ~/ros2_ws/src/py_topic/package.xml && \
    sed '23i\             "ros_laser_scan = py_topic.ros_laser_scan:main",\n             "ros_teleop = py_topic.ros_teleop:main"' ~/ros2_ws/src/py_topic/setup.py > changed.txt && mv changed.txt ~/ros2_ws/src/py_topic/setup.py && \
    cp ~/setup.py ~/ros2_ws/src/py_topic/ && \
    cd ~/ros2_ws/ || exit && \
    colcon build && \
    source /opt/ros/foxy/setup.bash

#Install Arduino CLI
#FROM foxy-ws AS arduino-cli

#RUN echo '                                                                       **** Installing Arduino CLI ****' && \
#    date && \
#    cd ~ && \
#    pip3 install pyserial && \
#    git clone https://github.com/arduino/arduino-cli.git && \
#    cd arduino-cli/ && \
#    export PATH=$PATH:/root/$USER/arduino-cli/bin && \
#    ./install.sh && \
#    arduino-cli config init && \
#    arduino-cli core update-index && \
#    arduino-cli core install arduino:samd && \
#    arduino-cli core install arduino:sam && \
#    arduino-cli core install arduino:avr && \
#    cd ~/.arduino15/packages/arduino/hardware/sam/1.6.12/ && \
#    date

#FROM ubuntu:focal
#COPY --from=foxy-ws
#COPY --from=arduino-cli

ENTRYPOINT [ "./start_sonar.sh" ]
