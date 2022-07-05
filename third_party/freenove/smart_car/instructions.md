## Abstract
This instruction is for freenove 4WD smart car with Raspberry PI. This setup shows how to integrate FEAGI with your robot and control wheels. Currently, this has been tested on 20.04 LTS server and Raspberry PI 3/4. This instruction will show you how to connect FEAGI on your computer with your robot. To learn more about docker, [click here](https://github.com/Neuraville/feagi/blob/staging/DEPLOY.md) Format your SD card, get Freenove smart car folder, add all necessary apt repo and set the wifi up on the raspberry pi are included in this instructions.


#### Download the imager: https://www.raspberrypi.com/software/
#### Select OS: Ubuntu Server 20.04 LTS (RPI 3,4,400) arm64 architecture
#### Once it's complete, install the sd card in your raspberry pi on freenove 
#### To set wifi up: https://huobur.medium.com/how-to-setup-wifi-on-raspberry-pi-4-with-ubuntu-20-04-lts-64-bit-arm-server-ceb02303e49b
#### Verify if network is connected.
#### Wait for update to complete if your sd is brand new or has been formatted recently which is usually 30 minutes or so.

- ps aux | grep lock

#### If you see terminal like this:
![image](https://user-images.githubusercontent.com/65916520/177386671-f02bbb70-462d-42ee-8892-956f4d7fa109.png)
#### Wait for 5 more minutes, but if you see terminal like this:
![image](https://user-images.githubusercontent.com/65916520/177390926-e2e95ca8-ebd5-41a5-afc9-d47c87c46cf6.png)
#### Where `/bin/sh /usr/lib/apt/apt.systemd.daily lock_is_held install` is disappeared, then you can move to the next.

- git clone https://github.com/feagi/feagi.git

- cd ~/feagi/third_party/freenove/smart_car

- ./setup.py

- Update the IP which computer the FEAGI is on in the configuration.py. Replace the "feagi" next to `"feagi_host":` under network_settings dictionary with the IP. 

- sudo python3 controller.py
