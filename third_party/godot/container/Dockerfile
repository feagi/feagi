FROM dorowu/ubuntu-desktop-lxde-vnc

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        lsb-release \
        wget \
        git \
        python3-zmq \
        xterm \
        python3-pip \ 
        godot3
        
RUN sudo apt-get install -y build-essential scons pkg-config libx11-dev libxcursor-dev libxinerama-dev \
    libgl1-mesa-dev libglu-dev libasound2-dev libpulse-dev libudev-dev libxi-dev libxrandr-dev yasm
#RUN scons -j8 platform=x11
#RUN scons platform=x11 tools=no target=release bits=64
RUN mkdir -p ./.local/share/godot/templates/3.2.stable/linux_x11_64_release/
RUN wget https://downloads.tuxfamily.org/godotengine/3.2.2/Godot_v3.2.2-stable_export_templates.tpz 
RUN mv Godot_v3.2.2-stable_export_templates.tpz ./.local/share/godot/templates/3.2.stable/linux_x11_64_release/
COPY ./* /root/
RUN chmod a+x /root/*

ENV OPENBOX_ARGS='--startup "/root/FEAGI_initalize.sh"'
