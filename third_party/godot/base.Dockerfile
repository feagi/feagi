FROM python:3.8-slim

RUN pip3 install websockets
RUN pip3 install pyzmq
RUN pip3 install requests
