FROM neuraville/bridge-godot-base:latest

RUN apt-get update
RUN apt install -y libgl1-mesa-glx libglib2.0-0
RUN pip3 install -U feagi_connector
COPY configuration.json /root/src/
COPY requirements.txt /root/src/
COPY bridge_godot_python.py /root/src/
COPY version.py /root/src/
COPY godot_bridge_functions.py /root/src/
COPY network_configuration.py /root/src/
RUN mkdir -p /root/src/FEAGIByteStructures
COPY FEAGIByteStructures/* /root/src/FEAGIByteStructures

WORKDIR /root/src
CMD ["python3", "bridge_godot_python.py"]
