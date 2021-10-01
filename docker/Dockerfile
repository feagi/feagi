FROM neuraville/feagi-base-os

# Git
ARG REPO="https://github.com/feagi/feagi-core.git"

# FEAGI
RUN mkdir -p /opt/source-code/feagi-core/
RUN git clone $REPO /opt/source-code/feagi-core
WORKDIR /opt/source-code/feagi-core/
RUN git checkout feature-controller-cleanup

# MNIST
# RUN wget http://yann.lecun.com/exdb/mnist/train-images-idx3-ubyte.gz -P /opt/source-code/feagi-core/raw/MNIST/ -q && \
#     wget http://yann.lecun.com/exdb/mnist/train-labels-idx1-ubyte.gz -P /opt/source-code/feagi-core/raw/MNIST/ -q && \
#     wget http://yann.lecun.com/exdb/mnist/t10k-images-idx3-ubyte.gz -P /opt/source-code/feagi-core/raw/MNIST/ -q && \
#     wget http://yann.lecun.com/exdb/mnist/t10k-labels-idx1-ubyte.gz -P /opt/source-code/feagi-core/raw/MNIST/ -q

ENV Running_In_CONTAINER Yes

# Cythonize
WORKDIR /opt/source-code/feagi-core/src/cython_lib
RUN python3 ./cython_setup.py build_ext --inplace

WORKDIR /opt/source-code/feagi-core
#RUN mkdir /opt/source-code/feagi-core/connectome
#RUN pip3 install RPi.GPIO==0.7.0 && \
#    pip3 install smbus==1.1.post2 && \
#    pip3 install rpi-ws281x==4.3.0

WORKDIR /opt/source-code/feagi-core/src
CMD ["python3", "main.py"]
