COPY ./configuration.py /root
COPY ./controller.py /root
COPY ./router.py /root
RUN chmod a+x /root/*
WORKDIR /root
#CMD ["python3", "controller.py"]
CMD ["/bin/sh", "-ec", "sleep 1000"]
