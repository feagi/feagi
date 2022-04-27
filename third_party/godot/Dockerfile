FROM neuraville/feagi-godot-html

COPY ./godot_source/* /root/godot_source/
#COPY ./html/ /root/html/
COPY ./src/* /root/src/
COPY ./feagi_initalize.sh /root/src/
RUN chmod a+x /root/*

WORKDIR /root/src
CMD ["bash", "feagi_initalize.sh"]
