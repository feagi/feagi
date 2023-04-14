FROM python:3.7-slim-buster

RUN mkdir -p templates
RUN mkdir -p assets/
COPY assets/ assets/
COPY configuration.py .
# COPY nginx.conf .
COPY aliens.py .
COPY video_capture.py .
COPY templates/ templates/

# Downloading now
RUN apt update
RUN pip3 install pygame
RUN pip3 install fastapi
RUN apt-get install -y python3-numpy
RUN pip3 install opencv-python==4.7.0.68
RUN apt install -y libgl1-mesa-glx
RUN pip3 install feagi_agent
RUN pip3 install jinja2
RUN pip3 install uvicorn
RUN apt-get install -y libglib2.0-0


CMD ["python3", "video_capture.py"]
