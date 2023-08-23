FROM dorowu/ubuntu-desktop-lxde-vnc

RUN wget -q -O - https://dl.google.com/linux/linux_signing_key.pub | sudo apt-key add -
RUN sudo apt-get update
RUN sudo apt install lsb-release -y \
        git \
        python3-zmq
RUN apt-get install -y python3-pip
RUN apt install -y wget
RUN apt-get install -y libusb-1.0-0-dev portaudio19-dev libasound2-dev
RUN apt-get install -y autoconf libtool-bin swig freeglut3 freeglut3-dev
RUN apt-get install -y libgl1-mesa-dev libglu1-mesa-dev libgstreamer-plugins-base1.0-dev
RUN apt install -y libcairo2-dev
RUN apt install -y libgirepository1.0-dev
RUN pip3 install neptune-cli -U
RUN pip3 install attrdict
RUN wget https://extras.wxpython.org/wxPython4/extras/linux/gtk3/ubuntu-20.04/wxPython-4.1.1-cp38-cp38-linux_x86_64.whl
RUN pip3 install wxPython-4.1.1-cp38-cp38-linux_x86_64.whl
RUN pip3 install psychtoolbox
RUN pip3 install psychopy
RUN pip3 install PyGObject
RUN pip3 install feagi-agent
RUN groupadd --force psychtoolbox
RUN usermod -a -G psychtoolbox root
RUN echo "@psychtoolbox   -  nice       -20" > /etc/security/limits.d/99-psychopylimits.conf
RUN echo "@psychtoolbox   -  rtprio      50" >> /etc/security/limits.d/99-psychopylimits.conf
RUN echo "@psychtoolbox   -  memlock     unlimited" >> /etc/security/limits.d/99-psychopylimits.conf
COPY ./ ./Desktop
CMD ["/bin/sh", "-ec", "sleep 100000"]