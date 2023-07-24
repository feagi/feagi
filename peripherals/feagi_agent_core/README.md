# What is Feagi-agent?
The Feagi-agent is the core of the Feagi agent and it serves multiple purposes. It works behind the scenes and serves as an API for Feagi.
Keep in mind, you will need Feagi to connect with feagi_agent. Feagi can be running on a website, Docker, or locally on your computer. 

If you would like to learn more about how to run Feagi, please visit: https://github.com/feagi/feagi/tree/staging

# Quick Start:
You can start with any of the quick start options. The easiest and simplest method is to use feagi_agent_video_capture, which focuses solely on video capture functionality. Once you have installed it using pip3 install feagi_agent_video_controller, simply type python3 -m feagi_agent_video_controller --ip 127.0.0.1 to run it.

If you would like to create your own code using the Feagi API and call `feagi_interface` or `retina`, you can use the following code:

`from feagi_agent import feagi_interface`

`from feagi_agent import retina`

# Feagi Agent Packages:
Currently, we have more than 2 packages: `feagi_agent_video_controller`, `feagi_agent_freenove`, and `feagi_agent_mycobot`. You can find more packages here: https://github.com/feagi/feagi/tree/feature-mycobot-updated/peripherals
The requirements for `feagi-agent` can be found here: [feagi-agent requirements](https://github.com/feagi/feagi/blob/staging/peripherals/feagi_agent_core/feagi_agent/requirements.txt)