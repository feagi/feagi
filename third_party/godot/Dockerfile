FROM python:3.8-slim
# FROM python:3.8
RUN pip3 install websockets
RUN pip3 install pyzmq
RUN pip3 install requests

COPY ./src/* /root/src/
WORKDIR /root/src
CMD ["python3", "bridge_godot_python.py"]
