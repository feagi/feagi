FROM neuraville/feagi:0.0.2

# Git
ARG REPO="https://github.com/feagi/feagi-core.git"

# FEAGI
RUN mkdir -p /opt/source-code/feagi-core/
RUN git clone $REPO /opt/source-code/feagi-core
WORKDIR /opt/source-code/feagi-core/

# MNIST
# RUN wget http://yann.lecun.com/exdb/mnist/train-images-idx3-ubyte.gz -P /opt/source-code/feagi-core/raw/MNIST/ -q && \
#     wget http://yann.lecun.com/exdb/mnist/train-labels-idx1-ubyte.gz -P /opt/source-code/feagi-core/raw/MNIST/ -q && \
#     wget http://yann.lecun.com/exdb/mnist/t10k-images-idx3-ubyte.gz -P /opt/source-code/feagi-core/raw/MNIST/ -q && \
#     wget http://yann.lecun.com/exdb/mnist/t10k-labels-idx1-ubyte.gz -P /opt/source-code/feagi-core/raw/MNIST/ -q

ENV Running_In_CONTAINER Yes

# Cythonize
WORKDIR /opt/source-code/feagi-core
RUN python3 ./src/cython/cython_setup.py build_ext --inplace && \
    mkdir /opt/source-code/feagi-core/connectome

WORKDIR /opt/source-code/feagi-core/src
CMD ["python3", "main.py"]
