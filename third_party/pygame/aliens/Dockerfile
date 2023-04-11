

ENV Running_In_CONTAINER Yes
RUN mkdir -p templates
RUN mkdir -p data/
COPY data/ data/
COPY configuration.py .
# COPY nginx.conf .
COPY aliens.py .
COPY video_capture.py .
COPY templates/ templates/

CMD ["python3", "video_capture.py"]
