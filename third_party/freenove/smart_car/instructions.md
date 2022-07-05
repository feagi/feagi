#### Download the imager: https://www.raspberrypi.com/software/
#### Select OS: Ubuntu Server 20.04 LTS (RPI 3,4,400) arm64 architecture
#### Once it's complete, install the sd card in your raspberry pi on freenove 
#### To set wifi up: https://huobur.medium.com/how-to-setup-wifi-on-raspberry-pi-4-with-ubuntu-20-04-lts-64-bit-arm-server-ceb02303e49b
#### Verify if network is connected.
#### Wait for update to complete if your sd is brand new or has been formatted recently which is usually 30 minutes or so.

- sudo apt update

#### If the command updated without any issue, you may continue to the next command. If the above gave you some errors, wait for another five minutes.

- git clone https://github.com/feagi/feagi.git

- cd ~/feagi/third_party/freenove/smart_car

- ./setup.py

- sudo reboot

- cd ~/feagi/third_party/freenove/smart_car

- sudo python3 controller.py
