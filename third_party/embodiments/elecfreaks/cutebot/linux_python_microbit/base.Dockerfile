FROM python:3.8.3

RUN apt-get update

RUN DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends libboost-all-dev
RUN DEBIAN_FRONTEND=noninteractive apt-get install -y bluez \
  bluetooth \
  pkg-config \
  libboost-python-dev \
  libboost-thread-dev \
  libbluetooth-dev \
  libglib2.0-dev \
  python3-dev \
  libgirepository1.0-dev \
  gcc \
  libcairo2-dev \
  gir1.2-gtk-3.0
  
RUN apt-get update && apt-get install -yq --no-install-recommends dbus bluez libbluetooth-dev

RUN apt install -y python3
RUN apt install -y python3-pip
RUN apt install -y libdbus-1-3 libdbus-1-dev
RUN apt install rfkill
RUN DEBIAN_FRONTEND=noninteractive apt install -y  python3-dbus
RUN apt install -y bluez libbluetooth-dev
RUN pip3 install bluezero
RUN pip3 install pyzmq
RUN pip3 install dbus-python
RUN pip3 install PyGObject
RUN pip3 install requests
