FROM dorowu/ubuntu-desktop-lxde-vnc

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        lsb-release \
        wget \
        git \
        python3-zmq \
        xterm \
        python3-pip \
        unzip

RUN sudo apt-get install -y build-essential scons pkg-config libx11-dev libxcursor-dev libxinerama-dev \
    libgl1-mesa-dev libglu-dev libasound2-dev libpulse-dev libudev-dev libxi-dev libxrandr-dev yasm
RUN mkdir -p ./.local/share/godot/templates/3.2.stable/linux_x11_64_release/
RUN wget https://downloads.tuxfamily.org/godotengine/3.4.2/Godot_v3.4.2-stable_x11.64.zip
RUN unzip Godot_v3.4.2-stable_x11.64.zip && rm Godot_v3.4.2-stable_x11.64.zip
