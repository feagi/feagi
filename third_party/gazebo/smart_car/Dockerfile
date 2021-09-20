FROM tiryoh/ros2-desktop-vnc:foxy
# FROM ros:foxy

# ARG REPO="https://github.com/feagi/feagi-simulate.git"
# ARG BRANCH="feature-ign-foxy"

ENV GAZEBO_CONTAINER=true

# install initial tools/dependencies
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions \
        lsb-release \
        wget \
        git \
        python3-zmq \
        xvfb \
        iputils-ping

# install Ignition Citadel and dependencies
RUN echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list && \
    wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add - && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        ignition-citadel \ 
        ros-foxy-ros-ign

# clone simulation repo, copy files and set up environment 
# RUN mkdir -p /opt/source-code/feagi-simulate && \
#     git clone $REPO /opt/source-code/feagi-simulate

# WORKDIR /opt/source-code/feagi-simulate
# RUN git checkout $BRANCH

RUN mkdir -p /opt/source-code/freenove_4wd_car_description
WORKDIR /opt/source-code/freenove_4wd_car_description
COPY ./freenove_4wd_car_description/ ./
