# Quick start using feagi_agent
1) `git clone https://github.com/feagi/feagi.git`
2) `cd feagi/docker`
3) `docker compose -f feagi.yml build`
4) Wait until #3 step is complete.
5) `docker compose -f feagi.yml up`
6) load it in your preferred browser: `http://localhost:3000/genome/mode`
7) Click `Sample Genomes`
8) See Gazebo and brain activity loaded. Feel free to play with your robot!

Any issue? Needs detailed documentation about docker? [Deployment documentation](https://github.com/feagi/feagi/wiki/Deployment)


# Where is feagi_agent? 
`feagi_agent` allows you to integrate with our FEAGI with your preferred robots. 
`feagi_agent` has been tested on <i>freenove_smart_car, Gazebo, Godot, Tello, and Psychopy.</i> 
You can use it on your computer or docker.

# What is inside feagi_agent?
There are two large library: feagi_interface and retina. `feagi_interface.py` handles the bridge between your project and FEAGI. It allows FEAGI to communicate/controls your robot.
`retina.py` is the vision where it can see things through any type of camera. 

# configuration.py is REQUIRED
You will need to have your own configuration where you can adjust setting to see the different results. If you don't have one, use the template below:
```
app_name = 'embodiment'

network_settings = {
    "feagi_host": "feagi",
    "feagi_api_port": "8000",
    'TTL': 2,
    'last_message': 0,
}

capabilities = {
    "vision": {
        "type": "ipu",
        "disabled": False,
        "count": 1,
        "width": 8,
        "height": 8,
        "deviation_threshold": 0.05,
        "retina_width_percent": 60,
        "retina_height_percent": 40,
        "central_vision_compression": [64, 64],
        "peripheral_vision_compression": [8, 8],
        "previous_data": {}
    }
}

message_to_feagi = {"data": {}}
```


See examples here:

[Tello's configuration](https://github.com/feagi/feagi/tree/feature-refactor-vision/third_party/physical_robots/tello)

[Freenove_smart_car](https://github.com/feagi/feagi/tree/feature-refactor-vision/third_party/physical_robots/freenove/smart_car)

[Gazebo](https://github.com/feagi/feagi/tree/feature-refactor-vision/third_party/gazebo/simulation/src)

# Where is feagi_agent being used?
It is being used in python code.

Just install through pip.

Windows:
`pip install feagi-agent`  

Linux/Mac:
`pip3 install feagi-agent`

and after that, you can just use `from feagi-agent import feagi_interface`

See examples:
[Tellos' code](https://github.com/feagi/feagi/blob/feature-refactor-vision/third_party/physical_robots/tello/tello.py#L6)