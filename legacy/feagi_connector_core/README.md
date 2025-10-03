# What is feagi-connector?
The feagi-connector is the core of the Feagi agent and it serves multiple purposes. It works behind the scenes and serves as an API for Feagi.
Keep in mind, you will need Feagi to connect with feagi_connector. Feagi can be running on a website, Docker, or locally on your computer. 

If you would like to learn more about how to run Feagi, please visit: https://github.com/feagi/feagi/tree/staging

# Quick Start:
You can start with any of the quick start options. The easiest and simplest method is to use feagi_connector_video_capture, which focuses solely on video capture functionality. Once you have installed it using pip3 install feagi_connector_video_controller, simply type python3 -m feagi_connector_video_controller --ip 127.0.0.1 to run it.

If you would like to create your own code using the Feagi API and call `feagi_interface` or `retina`, you can use the following code:

`from feagi_connector import feagi_interface`

`from feagi_connector import retina`

# Feagi Agent Packages:
Currently, we have more than 2 packages: `feagi_connector_video_controller`, `feagi_connector_freenove`, and `feagi_connector_mycobot`. You can find more packages here: https://github.com/feagi/feagi/tree/feature-mycobot-updated/peripherals
The requirements for `feagi-connector` can be found here: [feagi-connector requirements](https://github.com/feagi/feagi/blob/staging/peripherals/feagi_connector_core/feagi_connector/requirements.txt)