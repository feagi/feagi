FROM python:slim-buster
COPY ./talk.py ./
RUN pip3 install zmq
CMD [ "listen", "21001" ]
ENTRYPOINT [ "python3", "talk.py" ]