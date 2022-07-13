**This page may be outdated or incomplete. Please refer to the [deployment guide](https://github.com/feagi/feagi/wiki/Deployment) in our wiki for the most up to date information.**



# Introduction
FEAGI platform is consisted of core component known as FEAGI-core that is at large a Python package and responsible for 
the creation, development, and operation of the artificial brain processes. Furthermore, FEAGI leverages many open-source 
libraries as part of its ecosystem to provide visualization, data analytics, 3D simulation, and robot integration 
capabilities. We have containerized all the needed software packages and provided a simple method to deploy the entire
software using Docker. This document describes system requirements and the steps to deploy FEAGI platform. 


# Requirements
## Minimum System Requirements
* Free storage:  5 GB
* Memory:        16 GB
* CPU:           4 Cores

## Supported Operating Systems
* **macOS**: High Sierra (10.13)
* **Windows**: 10
* **Linux**: Ubuntu (18.04)

## Dependencies
* **Docker**: Desktop 4.7.1+, Engine 20.10.14+, Compose v2.4.1+
* **Python**: 3.7+
* **Git**:    Latest


# Deployment
## 1. Clone the repo
    git clone git@github.com:feagi/feagi.git

## 2. Configure (Optional)
<details>
<summary>FEAGI Configuration</summary>
Many of the environmental variables can be configured to enable FEAGI to have system adaptation flexibility. 

### Port mapping
Port mapping is done in two different files:
* /src/feagi_configruation.ini
* /docker/feagi.yml

feagi_configuration defines lower-level port mapping for FEAGI independent of the docker containers influence.
feagi.yml defines the port mapping as a wrapper above the FEAGI internal settings and would be the first place to make adjustments if there is a port conflict on the system.

### Volume mapping
Volume mapping is used as part of the .yml container deployment recipes to help you overwrite files where the most notable one is the genome. Depending on the application, the volume mapping needs to be place under the proper section withing the .yml file.
    
Default mapping shipped with FEAGI:

```
feagi:
    volumes:
  - ../src/evo/static_genome.py:/opt/source-code/feagi/src/evo/static_genome.py
  - ../src/feagi_configuration.ini:/opt/source-code/feagi/src/feagi_configuration.ini
```


Mapping example to load custom genome and custom environment:
```
 ros-gazebo:
    volumes:
      - /your/local/path/smart-car/simulation/freenove_smart_car.sdf:/opt/source-code/freenove_4wd_car_description/models/sdf/freenove_smart_car.sdf
      - /your/local/path/smart-car/simulation/meshes:/opt/source-code/freenove_4wd_car_description/models/sdf/meshes  
 
 
feagi:
    volumes:
  - /your/local/path/smart-car/genome/custom_genome.py:/opt/source-code/feagi/src/evo/static_genome.py
  - ../src/feagi_configuration.ini:/opt/source-code/feagi/src/feagi_configuration.ini
```

</details>

## 3. Build
`
cd [path to where feagi repo was cloned to]/feagi/docker
`

`
docker compose -f feagi.yml build
`

## 4. Run
`
docker compose -f feagi.yml up -d
`

To use the application, open browser at 127.0.0.1:3000

## 5. re-Run
`
docker compose -f feagi.yml down
`

`
docker compose -f feagi.yml up -d
`
