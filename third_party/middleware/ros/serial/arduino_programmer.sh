#Install Arduino CLI
echo '**** Installing Arduino CLI ****'
date
cd ~
pip3 install pyserial
git clone https://github.com/arduino/arduino-cli.git
cd arduino-cli/
export PATH=$PATH:/root/$USER/arduino-cli/bin
./install.sh
export PATH=$PATH:/root/$USER/arduino-cli/bin
arduino-cli config init
arduino-cli core update-index
#arduino-cli core install arduino:samd
#arduino-cli core install arduino:sam
arduino-cli core install arduino:avr
#mkdir micro-ros_publisher
#cd micro-ros_publisher
#cp ~/micro-ros_publisher.ino ~/arduino-cli/micro-ros_publisher/
#cd ~/.arduino15/packages/arduino/hardware/sam/1.6.12/
curl https://raw.githubuse