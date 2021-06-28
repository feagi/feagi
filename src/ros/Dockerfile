FROM ros:foxy

SHELL ["/bin/bash", "-c"]

# install tools
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        wget \
        ros-foxy-gazebo-ros-pkgs \
        gazebo11 \
        python3-pip \
        iproute2 \
        iputils-ping \
        net-tools

# set up workspaces
WORKDIR /root/
COPY setup_container.sh ros_laser_scan.py launch_in_container.sh ros_teleop.py /root/
RUN chmod +x setup_container.sh && \
    chmod +x launch_in_container.sh && \
    ./setup_container.sh

ENV TURTLEBOT3_MODEL=burger
EXPOSE 2000/tcp
ENTRYPOINT [ "./launch_in_container.sh" ]
