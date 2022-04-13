FROM buildpack-deps:bullseye

#RUN apt update
#RUN apt install -y python3
#RUN apt install -y python3-pip
#RUN pip3 install websockets
#RUN apt install -y npm
#RUN npm -v
#RUN npm install
COPY ./* /root/
RUN chmod a+x /root/*
CMD ["/bin/sh", "-ec", "sleep 1000"]
