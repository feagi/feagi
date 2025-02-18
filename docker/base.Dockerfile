FROM python:3.12-bullseye

ENV PATH="${PATH}:/sbin;/bin"
RUN apt update
RUN apt install wget git -y

# Python
RUN pip3 install --upgrade pip && \
    pip3 install pyzmq==26.0.3 && \
    pip3 install Cython==0.29.21 && \
    pip3 install numpy==2.0.0 && \
    pip3 install watchdog==4.0.1 && \
    pip3 install pymongo==4.8.0 && \
    pip3 install influxdb_client==1.44.0 && \
    pip3 install psutil==5.8.0 && \
    pip3 install uvicorn==0.17.5 && \
    pip3 install fastapi==0.95.2 && \
    pip3 install pydantic==1.10.17 && \
    pip3 install requests==2.27.1 && \
    pip3 install python-multipart==0.0.5 && \
    pip3 install typing_extensions==4.1.1 && \
    pip3 install xxhash==3.4.1 && \
    pip3 install lz4==4.3.3 && \
    pip3 install starlette==0.27.0  && \
    pip3 install sympy==1.13.3
