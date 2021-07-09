FROM ros:foxy

SHELL ["/bin/bash", "-c"]

ENV TURTLEBOT3_MODEL=burger

# install tools
RUN groupadd -r ros_gazebo && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        wget \
        ros-foxy-gazebo-ros-pkgs \
        gazebo11 \
        python3-pip

# set up workspaces
WORKDIR /root/
COPY ./ /root/
RUN chmod u+x setup_turtlebot.sh && \
    chmod u+x start_turtlebot.sh && \
    chmod u+x setup_ros_workspace.sh && \
    ./setup_ros_workspace.sh && \
    ./setup_turtlebot.sh

ENTRYPOINT [ "./start_turtlebot.sh" ]
