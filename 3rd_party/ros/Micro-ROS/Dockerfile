FROM ros:foxy

SHELL ["/bin/bash", "-c"]

# install tools
RUN groupadd -r micro_ros && \
    apt-get update && \
    apt-get install -y --no-install-recommends python3-pip

# set up workspaces
WORKDIR /root/
COPY ./ /root/
RUN chmod u+x setup_ros_workspace.sh && \
    chmod u+x setup_micro_ros.sh && \
    chmod u+x test.sh && \
    ./setup_micro_ros.sh

ENTRYPOINT [ "./test.sh" ]
