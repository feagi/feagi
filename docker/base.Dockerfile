FROM python:3.8-buster

ENV PATH="${PATH}:/sbin;/bin"
RUN apt-get update
RUN apt-get install wget git

# Python
RUN pip3 install --upgrade pip && \
    pip3 install pyzmq==22.1.0 && \
    pip3 install Cython==0.29.21 && \
    pip3 install numpy==1.19.5 && \
    pip3 install watchdog==2.1.2 && \
    pip3 install pymongo==3.11.2 && \
    pip3 install influxdb_client==1.18.0 && \
    pip3 install pyzmq==22.1.0