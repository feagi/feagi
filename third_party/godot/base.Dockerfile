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