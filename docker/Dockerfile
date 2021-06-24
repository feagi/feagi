FROM python:3.8-buster
LABEL maintainer="Eric Miller"
ARG REPO="https://github.com/feagi/feagi-core.git"

# Git 
RUN apt-get update && \
    apt-get install -y git && \
    apt-get install wget

# FEAGI
RUN mkdir -p /opt/source-code/feagi-core/ && \
    git clone $REPO /opt/source-code/feagi-core
WORKDIR /opt/source-code/feagi-core/
RUN git checkout develop
# MNIST
# RUN wget http://yann.lecun.com/exdb/mnist/train-images-idx3-ubyte.gz -P /opt/source-code/feagi-core/raw/MNIST/ -q && \
#     wget http://yann.lecun.com/exdb/mnist/train-labels-idx1-ubyte.gz -P /opt/source-code/feagi-core/raw/MNIST/ -q && \
#     wget http://yann.lecun.com/exdb/mnist/t10k-images-idx3-ubyte.gz -P /opt/source-code/feagi-core/raw/MNIST/ -q && \
#     wget http://yann.lecun.com/exdb/mnist/t10k-labels-idx1-ubyte.gz -P /opt/source-code/feagi-core/raw/MNIST/ -q

# Python
RUN pip3 install --upgrade pip && \
    pip3 install -r /opt/source-code/feagi-core/requirements.txt

# Cythonize
WORKDIR /opt/source-code/feagi-core
RUN python3 ./src/cython_libs/cython_setup.py build_ext --inplace && \
    mkdir /opt/source-code/feagi-core/connectome

WORKDIR /opt/source-code/feagi-core/src
CMD ["python3", "main.py"]
