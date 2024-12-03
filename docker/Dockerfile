FROM neuraville/feagi-base-os2

ARG REPO="https://github.com/feagi/feagi.git"
ARG BRANCH="main"

ENV Running_In_CONTAINER Yes

RUN mkdir -p /opt/source-code/feagi/
RUN echo "branch: " $BRANCH
RUN git clone $REPO -b $BRANCH /opt/source-code/feagi

WORKDIR /opt/source-code/feagi/src
ENV PYTHONPATH "${PYTHONPATH}:/opt/source-code/feagi"
CMD ["python3", "main.py"]
