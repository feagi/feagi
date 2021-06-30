FROM python:slim-buster
COPY ./listen.py ./
RUN pip3 install zmq
ENTRYPOINT ["python3", "listen.py"]