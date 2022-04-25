FROM neuraville/feagi-godot-html

WORKDIR /root
RUN mkdir -p /root/godot_source
RUN mkdir -p /root/html
RUN mkdir -p /root/src
COPY ./godot_source/* /root/godot_source
COPY ./html/* /root/html
COPY ./src/* /root/src
COPY ./feagi_initalize.sh /root/src
RUN chmod a+x /root/*
WORKDIR /root/src
CMD ["bash", "feagi_initalize.sh"]

