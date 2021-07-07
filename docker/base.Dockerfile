FROM diefans/python3.8-alpine-cython:0.1.0

# For Alpine Image
ENV PATH="${PATH}:/sbin;/bin"
RUN apk update
RUN apk add wget git build-base libzmq musl-dev python3 python3-dev zeromq-dev
RUN apk add git

# Python
RUN pip3 install --upgrade pip && \
    pip3 install pyzmq==22.1.0 && \
    pip3 install Cython==0.29.21 && \
    pip3 install numpy==1.19.5 && \
    pip3 install watchdog==2.1.2 && \
    pip3 install pymongo==3.11.2 && \
    pip3 install influxdb_client==1.18.0 && \
    pip3 install pyzmq==22.1.0

#RUN apk del build-base musl-dev python3-dev zeromq-dev