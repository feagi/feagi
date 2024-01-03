# Work in process (Completely DRAFT)
## Install brain in Microbit
1) Introduction Microbit
Microbit was intrdouced in 2016. This can be used in various projects and it is often used in robots too. Microbit is very easy and effective while it kept 
the minimum memory usage. So with FEAGI, it allows you to control the robot effectively. FEAGI is the Framework for Evolutionary Artificial General Intelligence
and it will control the robot. You can also control the robot through Godot using FEAGI as well. 

2) FEAGI and microbit use-case
WIP
3) Quick example between them
WIP
4) Requirements
There will be more specific in some areas such as software operating but this is the general requirement. We will come to the specific for your own case.
- FEAGI
- Bluetooth
- Python3.7+
- Docker 
5) Quick start

6) More depth using microbit and FEAGI after quick start

## Microbit's full information
1) Programming used
- Microbit used makecode mix with the python so the `controller.py` communicates between Microbit and FEAGI. 
Here is the diagram of workflow:
![image](_static/Microbit_flowchart.png)
2) Docker container

3) Configuration inside microbit
4) Bluetooth
5) How to set things up outside of docker


# Qucik set up
1) Open the browser: https://makecode.microbit.org/
2) Drag `microbit-initalizer.sh` to the site
3) Wait for it to complete the load.
4) Plug your microbit with your computer
5) Click the download bottom right
![download_image](_static/download_image.png)
6) Once it's complete, you can unplug the usb off your microbit.

# Qucik start instruction:
1) `sudo systemctl stop bluetooth.service` on your terminal
2) `sudo systemctl stop bluetooth` on your terminal
3) Navigate to `feagi/src/feagi_configuration.ini`
4) Edit that file
5) Change from `gazebo` and `ros-gazebo` under [Socket] to 127.0.0.1 or your computer's IP
6) Navigate to `feagi/third_party/godot/src`
7) edit configuration.py
8) Replace "feagi" next to feagi_host to 127.0.0.1 or the computer where FEAGI is on.
9) Turn Microbit on
10) Navigate to feagi/docker/
11) `docker compose -f microbit_feagi.yml build` 
12) `docker compose -f microbit_feagi.yml up`
13) Wait until you see Microbit's led saying 'C', it's ready


