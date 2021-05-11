# Micro-ROS on FEAGI
## Setup on linux
1. Plug the micro USB to Teensy 4.1 (the board should have an orange blinking)
2. Press the button on the board.(The blinking should stop)
3. Open terminal and type "nano 00-teensy.rules" then  paste this in the file from [this](https://www.pjrc.com/teensy/00-teensy.rules)
4. Save and exit.
5. Type this in the terminal, "sudo cp 00-teensy.rules /etc/udev/rules.d/"
6. Download the corresponding Teensyduino [installer](https://www.pjrc.com/teensy/td_download.html).
7. Type this in the terminal, "chmod 755 TeensyduinoInstall.linux64" (Be sure to type it in the same path where you downloaded it to)
8. Run this, "./TeensyduinoInstall.linux64"









# More information:
1. https://www.youtube.com/watch?v=ze-HiCr5s60&ab_channel=pk
2. https://www.pjrc.com/teensy/td_download.html
