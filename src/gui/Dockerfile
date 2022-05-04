FROM neuraville/feagi-base-gui

ARG REPO="https://github.com/feagi/feagi.git"
ARG BRANCH="main"

ENV Running_In_CONTAINER Yes

RUN mkdir -p /opt/source-code/feagi/
RUN git clone $REPO /opt/source-code/feagi
WORKDIR /opt/source-code/feagi
RUN git fetch --all
RUN git checkout $BRANCH

WORKDIR /opt/source-code/feagi/src/gui
RUN npm install
RUN npm start

