FROM dorowu/ubuntu-desktop-lxde-vnc

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        lsb-release \
        wget \
        git \
        python3-zmq \
        xterm \
        python3-pip 

#RUN mkdir -p /home/ubuntu/
COPY ./* /root/
RUN chmod a+x /root/*
#RUN chmod a+x /home/ubuntu/FEAGI_initalize.sh
#RUN chmod a+x /home/ubuntu/feagi_sub_test.py

ENV OPENBOX_ARGS='--startup "/root/FEAGI_initalize.sh"'
