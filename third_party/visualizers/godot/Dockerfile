FROM neuraville/bridge-godot-base

RUN pip3 install feagi_agent
COPY ./src/* /root/src/
WORKDIR /root/src
CMD ["python3", "bridge_godot_python.py"]
