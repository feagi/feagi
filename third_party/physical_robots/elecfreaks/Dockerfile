FROM neuraville/feagi-microbit

COPY ./configuration.py /root
COPY ./microbit_initalizer.sh /root
COPY ./controller.py /root
COPY ./router.py /root
RUN chmod a+x /root/*
WORKDIR /root
CMD ["bash", "microbit_initalizer.sh"]
#CMD ["/bin/sh", "-ec", "sleep 100000"]
