# Quick start
`pip3 install feagi_agent_freenove`

`python3 -m feagi_agent_freenove --ip 127.0.0.1`

# What is feagi_agent_freenove?
Feagi_agent_freenove is a package that allows you to connect with the Freenove Smartcar, which you can purchase on Amazon: [here on amazon](https://www.amazon.com/Freenove-Raspberry-Tracking-Avoidance-Ultrasonic/dp/B07YD2LT9D). 

It checks all the settings as the first step to ensure that you have all the necessary packages installed. 
After that, it runs requirements.txt and then starts the Freenove smartcar. 

At first, it may be a little slow because it is checking for all the necessary packages. However, after the first scan, it will be much quicker. This package enables you to connect with Feagi automatically, allowing it to see things, move, or share sensors in real-time.

# Requirements.txt
These are the requirements which will be updated over time. The script will use this requirement to scan.

Here: [requirements.txt](https://github.com/feagi/feagi/blob/staging/peripherals/feagi_agent_freenove/feagi_agent_freenove/requirements.txt)

