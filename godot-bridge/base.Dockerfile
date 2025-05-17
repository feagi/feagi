FROM python:3.12-bullseye

RUN pip3 install websockets
RUN pip3 install pyzmq
RUN pip3 install requests