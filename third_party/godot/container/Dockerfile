FROM neuraville/feagi-godot

RUN sudo apt-get update
RUN sudo apt-get -y install python3-pip
RUN pip3 install websockets
COPY ./* /root/
RUN chmod a+x /root/*
ENV OPENBOX_ARGS='--startup "/root/FEAGI_initalize.sh"'
