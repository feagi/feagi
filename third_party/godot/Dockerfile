FROM neuraville/feagi-godot-html

WORKDIR /root
RUN mkdir -p /root/godot_source
RUN mkdir -p /root/html
RUN mkdir -p /root/src
COPY ./godot_source/* /root/godot_source/
#COPY ./html/ /root/html/
COPY ./src/* /root/src/
COPY ./feagi_initalize.sh /root/src/
RUN chmod a+x /root/*

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
#RUN ./Godot_v3.4.4-stable_linux_server.64 

WORKDIR /root/src
CMD ["bash", "feagi_initalize.sh"]
