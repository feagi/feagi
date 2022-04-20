FROM ubuntu

RUN apt update
RUN apt install -y python3
RUN apt install -y python3-pip
RUN DEBIAN_FRONTEND=noninteractive apt install -y  python3-dbus
RUN apt install -y bluez libbluetooth-dev
RUN pip3 install bluezero
RUN pip3 install pyzmq
COPY ./configuration.py /root
COPY ./controller.py /root
COPY ./router.py /root
RUN chmod a+x /root/*
WORKDIR /root
#CMD ["python3", "controller.py"]
CMD ["/bin/sh", "-ec", "sleep 1000"]
