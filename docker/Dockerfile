FROM diefans/python3.8-alpine-cython:0.1.0

ARG REPO="https://github.com/feagi/feagi-core.git"

# Git

# For Debian Image
#RUN apt-get update && \
#    apt-get install -y  --no-install-recommends \
#        git \
#        wget

# For Alpine Image
ENV PATH="${PATH}:/sbin;/bin"
RUN apk update
RUN apk add wget git build-base libzmq musl-dev python3 python3-dev zeromq-dev
RUN apk add git


# FEAGI
RUN mkdir -p /opt/source-code/feagi-core/ && \
    git clone $REPO /opt/source-code/feagi-core
WORKDIR /opt/source-code/feagi-core/
RUN git checkout feature-dockerized-mongo

# MNIST
# RUN wget http://yann.lecun.com/exdb/mnist/train-images-idx3-ubyte.gz -P /opt/source-code/feagi-core/raw/MNIST/ -q && \
#     wget http://yann.lecun.com/exdb/mnist/train-labels-idx1-ubyte.gz -P /opt/source-code/feagi-core/raw/MNIST/ -q && \
#     wget http://yann.lecun.com/exdb/mnist/t10k-images-idx3-ubyte.gz -P /opt/source-code/feagi-core/raw/MNIST/ -q && \
#     wget http://yann.lecun.com/exdb/mnist/t10k-labels-idx1-ubyte.gz -P /opt/source-code/feagi-core/raw/MNIST/ -q

ENV Running_In_CONTAINER Yes

# Python
RUN pip3 install --upgrade pip && \
    pip3 install -r /opt/source-code/feagi-core/requirements.txt

#RUN pip3 install pyzmq


# Cythonize
WORKDIR /opt/source-code/feagi-core
RUN python3 ./src/cython/cython_setup.py build_ext --inplace && \
    mkdir /opt/source-code/feagi-core/connectome

RUN apk del build-base musl-dev python3-dev zeromq-dev

WORKDIR /opt/source-code/feagi-core/src
CMD ["python3", "main.py"]
