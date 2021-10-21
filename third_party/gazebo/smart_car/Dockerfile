FROM tiryoh/ros2-desktop-vnc:foxy
# FROM ros:foxy

ENV GAZEBO_CONTAINER=true
ENV OPENBOX_ARGS='--startup "/opt/source-code/freenove_4wd_car_description/start_controller.sh"'

# install initial tools/dependencies
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions \
        lsb-release \
        wget \
        git \
        python3-zmq \
        xterm

# install Ignition Citadel and dependencies
RUN echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list && \
    wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add - && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        ignition-citadel \ 
        ros-foxy-ros-ign

COPY ./setup_simulation.sh /home/ubuntu
RUN chmod a+x /home/ubuntu/setup_simulation.sh


RUN mkdir -p /opt/source-code/freenove_4wd_car_description
WORKDIR /opt/source-code/freenove_4wd_car_description
COPY ./freenove_4wd_car_description/ ./start_controller.sh ./
RUN chmod a+x ./start_controller.sh


