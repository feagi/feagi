FROM neuraville/feagi-base-os2

ARG REPO="https://github.com/feagi/feagi.git"
ARG BRANCH="feature-updates-godot4-debug-branch"

ENV Running_In_CONTAINER Yes

RUN mkdir -p /opt/source-code/feagi/
RUN git clone $REPO /opt/source-code/feagi
WORKDIR /opt/source-code/feagi
RUN git fetch --all
RUN git checkout $BRANCH

WORKDIR /opt/source-code/feagi/src
ENV PYTHONPATH "${PYTHONPATH}:/opt/source-code/feagi"
CMD ["python3", "main.py"]
