FROM neuraville/ros-foxy_gazebo-ign

ENV GAZEBO_CONTAINER=true
ENV OPENBOX_ARGS='--startup "/opt/source-code/feagi_robot/start_controller.sh"'

RUN apt install -y wmctrl

WORKDIR /etc/xdg/openbox
RUN sed -i "s|<number>4</number>|<number>1</number> |g" rc.xml
RUN mkdir -p /opt/source-code/feagi_robot
WORKDIR /opt/source-code/feagi_robot
COPY ./ ./
RUN chmod a+x ./start_controller.sh
RUN chmod 0755 /opt/source-code/feagi_robot


