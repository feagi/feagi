#!/bin/bash

# Copyright 2019 The FEAGI Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================

echo "Starting the setup in five seconds..."
echo "Press ctrl + c to stop the setup immediately"
sleep 5
echo "Installing..."
sudo apt-get install -y python3-dev python3-rpi.gpio
sudo apt-get install -y python3-smbus
sudo apt-get install -y libatlas-base-dev
sudo apt install -y python3-pip
sudo pip3 install zmq
sudo pip3 install feagi-agent
sudo pip3 install rpi_ws281x
sudo pip3 install opencv-python==4.6.0.66
sudo pip3 install -U numpy
sudo pip3 install lz4
sudo groupadd gpio
sudo usermod -aG gpio "${USER}"
echo "Setup is complete!"