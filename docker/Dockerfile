FROM neuraville/feagi-base-os

ARG REPO="https://github.com/feagi/feagi.git"
ENV Running_In_CONTAINER Yes

RUN mkdir -p /opt/source-code/feagi/
RUN git clone $REPO /opt/source-code/feagi
WORKDIR /opt/source-code/feagi
RUN git checkout feature-battery-opu

WORKDIR /opt/source-code/feagi/src/cython_lib
RUN python3 ./cython_setup.py build_ext --inplace

WORKDIR /opt/source-code/feagi/src
CMD ["python3", "main.py"]
