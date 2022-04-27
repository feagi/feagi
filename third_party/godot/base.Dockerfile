FROM ubuntu:20.04

RUN apt update
RUN apt install -y python3
RUN apt install -y python3-pip
RUN pip3 install websockets
RUN pip3 install pyzmq
RUN pip3 install requests
RUN apt install -y wget
RUN DEBIAN_FRONTEND=noninteractive apt-get install -y npm
RUN npm cache clean -f
RUN npm install -g n
RUN n stable
RUN hash -r
RUN curl -qL https://www.npmjs.com/install.sh | sh
RUN npm install --global http-server

WORKDIR /root
RUN mkdir -p /root/godot_source
RUN mkdir -p /root/html
RUN mkdir -p /root/src

WORKDIR /root/godot_source
RUN apt install -y wget 
RUN apt install -y unzip
RUN wget https://downloads.tuxfamily.org/godotengine/3.4.4/Godot_v3.4.4-stable_linux_headless.64.zip
RUN wget https://downloads.tuxfamily.org/godotengine/3.4.4/Godot_v3.4.4-stable_export_templates.tpz
RUN unzip Godot_v3.4.4-stable_linux_headless.64.zip
RUN mkdir -p /root/.local/share/godot/templates/3.4.4.stable/
RUN mv Godot_v3.4.4-stable_export_templates.tpz /root/.local/share/godot/templates/3.4.4.stable/
WORKDIR /root/.local/share/godot/templates/3.4.4.stable/
RUN unzip Godot_v3.4.4-stable_export_templates.tpz
WORKDIR templates/
RUN mv * ..
