FROM ubuntu

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
#RUN npm install http-server
#RUN apt install -y npm
#RUN npm -v
#RUN npm install
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
#CMD ["/bin/sh", "-ec", "sleep 1000"]
