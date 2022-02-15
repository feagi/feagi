FROM neuraville/ros-foxy_gazebo-ign

ENV GAZEBO_CONTAINER=true
ENV OPENBOX_ARGS='--startup "/opt/source-code/freenove_4wd_car_description/start_controller.sh"'


COPY ./setup_simulation.sh /root/
RUN chmod a+x /root/setup_simulation.sh
COPY ./start_simulation_no-vnc.sh /root/
COPY ./start_controller.sh ./start_controller.sh /root/
RUN chmod a+x /root/start_controller.sh


RUN mkdir -p /opt/source-code/freenove_4wd_car_description
WORKDIR /opt/source-code/freenove_4wd_car_description
COPY ./freenove_4wd_car_description/ ./start_controller.sh ./start_simulation_no-vnc.sh ./
RUN chmod a+x ./start_controller.sh
RUN chmod a+x ./start_simulation_no-vnc.sh
RUN chmod 0755 /opt/source-code/freenove_4wd_car_description/

