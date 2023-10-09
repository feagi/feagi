FROM python:3.8-buster

ENV PATH="${PATH}:/sbin;/bin"
RUN apt-get update


# Python
RUN pip3 install --upgrade pip && \
    pip3 install feagi_agent
RUN pip3 install websockets
RUN pip3 install fastapi
RUN pip3 install lz4
RUN pip3 install opencv-python==4.7.0.68
RUN apt install -y libgl1-mesa-glx

ENV Running_In_CONTAINER Yes
WORKDIR /root/

COPY controller.py /root/
COPY configuration.py /root/
CMD ["python3", "controller.py"]
