FROM neuraville/ros-foxy_gazebo-ign

ENV GAZEBO_CONTAINER=true
ENV OPENBOX_ARGS='--startup "/opt/source-code/simulation/start_controller.sh"'

WORKDIR /etc/xdg/openbox
RUN sed -i "s|<number>4</number>|<number>1</number> |g" rc.xml
RUN mkdir -p /opt/source-code/simulation
WORKDIR /opt/source-code/simulation
COPY ./ ./
COPY gazebo.png /usr/local/share/doro-lxde-wallpapers/bg1.jpg
RUN chmod a+x ./start_controller.sh
RUN chmod 0755 /opt/source-code/simulation


