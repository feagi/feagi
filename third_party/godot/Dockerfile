FROM --platform=linux/amd64 neuraville/feagi-godot-html

COPY ./godot_source/* /root/godot_source/
COPY /godot_source/menu_assets/ /root/godot_source/menu_assets/
COPY /godot_source/menu_assets/image/ /root/godot_source/menu_assets/image/
COPY ./html/ /root/html/
COPY ./html/ /root/html_backup/
COPY ./src/* /root/src/
COPY ./feagi_initalize.sh /root/src/
RUN chmod a+x /root/*

RUN npm install live-server -g

WORKDIR /root/src
CMD ["bash", "feagi_initalize.sh"]
