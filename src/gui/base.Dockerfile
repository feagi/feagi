FROM python:3.8-buster

ENV PATH="${PATH}:/sbin;/bin"
RUN apt update
RUN apt install wget git
RUN apt install -y npm nodejs
