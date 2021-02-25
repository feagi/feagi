# Introduction
The framework is known to run stably on OS X and Ubuntu, with deployment support for more operating systems (Windows, etc.) anticipated in the future, if needed. Users may extensively configure the framework according to their preferences by editing the various parameters in the `/src/feagi_configuration.ini` file. It is important that configuration parameters representing a path to a file or directory relevant to FEAGI execution be updated to accurately reflect the user's local environment (see `InitData` parameters in `feagi_configuration.ini`). FEAGI data output is, by default, stored in the host system's temporary directory. To customize this parameter, enter the desired output path in `working_directory` under `InitData` in `feagi_configuration.ini`.

# Supported Operating Systems
* **macOS**: High Sierra (10.13)
* **Linux**: Ubuntu (18.04)

# Setup Guide (manual)
## Virtual Environment Setup
It is recommended that FEAGI Python dependencies be installed in a virtual environment. 

To create a virtual environment in either Ubuntu or macOS, open a terminal and enter (`environName` is the desired environment name): 
* `$ virtualenv -p /usr/bin/python3 environName`

To activate the newly-created virtual environment, run (if successful, environment name should appear in parentheses next to terminal command prompt): 
* `$ source ./environName/bin/activate`

Install the FEAGI Python dependencies in the active virtual environment (assuming the working directory is `~/FEAGI/`): 
* `$ pip3 install -r requirements.txt`

## Ramdisk Setup
Using a ramdisk when running FEAGI can enhance execution speed by reducing read/write latency when interacting with genome/connectome data; whether this is necessary/useful depends on local physical drive performance specifications. If using a ramdisk, users should monitor its space when running FEAGI successively; each framework execution generates a new directory containing updated brain connectome data in `JSON` format. **Note:** If using a ramdisk for FEAGI execution, update the `working_directory` parameter in `InitData` of `feagi_configuration.ini` with the path to the mounted disk (typically in `/Volumes/` on macOS and `/mnt/` on Ubuntu).

### High Sierra (10.13)
Create/mount ramdisk (`diskSize` is user-defined disk size specified in MB): 
* `$ diskutil partitionDisk $(hdiutil attach -nomount ram://$((2048*diskSize))) GPTFormat HFS+ 'ramdisk' '100%'`

Eject/unmount ramdisk (all data on the disk will be lost): 
* `$ diskutil eject /Volumes/ramdisk`

### Ubuntu (18.04)
Create ramdisk mount point: 
* `$ sudo mkdir /mnt/ramdisk`

Mount ramdisk (`diskSize` is specified in gigabytes [ex: `2G`] or megabytes [ex: `2000M`]): 
* `$ sudo mount -t tmpfs -o rw,size=diskSize tmpfs /mnt/ramdisk`

Confirm disk is mounted:
* `$ df -h`

Eject/unmount ramdisk (all data on the disk will be lost): 
* `$ sudo umount /mnt/ramdisk`

## Cythonize Code
The directory `/FEAGI/src/cython_libs` contains a Python function (`neuron_functions_cy.pyx`) for updating postsynaptic neuron membrane potential, which is used extensively throughout FEAGI artificial brain creation and learning. Heavy usage of this function requires performance specifications that exceed those of Python in order for FEAGI to run efficiently. This Python code must be compiled into C-like code (i.e. Cythonized) to achieve the necessary performance optimization. To Cythonize the code, run (assuming the working directory is `~/FEAGI/`): 
* `$ python3 ./src/cython_libs/cython_setup.py build_ext --inplace`

## Database Setup
FEAGI relies on [MongoDB](https://www.mongodb.com/) and [InfluxDB](https://www.influxdata.com/) to store/retrieve genome and time series data, respectively.

### **MongoDB**
**macOS**    
Installation of MongoDB using a package manager such as [Homebrew](https://brew.sh/#install) is recommended. Visit [Install MongoDB on Mac](https://docs.mongodb.com/manual/tutorial/install-mongodb-on-os-x/) for more detailed installation and configuration instructions. Open a terminal and follow these steps:

Download official MongoDB formula: 
* `$ brew tap mongodb/brew`

Install the (currently) latest version of MongoDB: 
* `$ brew install mongodb-community@4.4`

Start MongoDB as a macOS service: 
* `$ brew services start mongodb-community@4.4`

Confirm MongoDB service has started: 
* `$ brew services list`

**Ubuntu (18.04)**    
The current stable release of MongoDB (4.4) only supports 64-bit versions of Ubuntu platforms and can be installed via the `apt` package manager. **Note**: The MongoDB package provided by Ubuntu is not official and causes conflicts when installed concurrently with the official version. Visit [Install MongoDB on Linux](https://docs.mongodb.com/manual/tutorial/install-mongodb-on-ubuntu/) for more detailed installation and configuration options.

Import the public GPG key (this operation should respond with `OK`): 
* `$ wget -qO - https://www.mongodb.org/static/pgp/server-4.4.asc | apt-key add -`

Create a list file for MongoDB: 
* `$ echo "deb [ arch=amd64,arm64 ] https://repo.mongodb.org/apt/ubuntu bionic/mongodb-org/4.4 multiverse" | sudo tee /etc/apt/sources.list.d/mongodb-org-4.4.list`

Reload the local package database: 
* `$ sudo apt-get update`

Install MongoDB packages: 
* `$ sudo apt-get install -y mongodb-org`

Start MongoDB (via `systemd`): 
* `$ sudo systemctl start mongod`

Verify that MongoDB is running: 
* `$ sudo systemctl status mongod`

Enable startup following system reboot: 
* `$ sudo systemctl enable mongod`

### **InfluxDB**
**macOS**    
As with MongoDB, installation of InfluxDB via Homebrew is recommended. More detailed installation and configuration instructions are available at [Install InfluxDB](https://docs.influxdata.com/influxdb/v1.8/introduction/install/).

Install InfluxDB: 
* `$ brew install influxdb`

Launch InfluxDB: 
* `$ influxd -config /usr/local/etc/influxdb.conf`

**Ubuntu (18.04)**

Import the public GPG key: 
* `$ wget -qO- https://repos.influxdata.com/influxdb.key | sudo apt-key add -`

Get Ubuntu distribution info:
* `$ source /etc/lsb-release`

Add the InfluxDB repository: 
* `$ echo "deb https://repos.influxdata.com/${DISTRIB_ID,,} ${DISTRIB_CODENAME} stable" | sudo tee /etc/apt/sources.list.d/influxdb.list`

Update the local package database: 
* `$ sudo apt-get update`

Install InfluxDB: 
* `$ sudo apt-get install influxdb`

Start InfluxDB (via `systemd`): 
* `$ sudo systemctl unmask influxdb.service && sudo systemctl start influxdb`

## Run FEAGI
To start FEAGI, open a terminal, ensure the FEAGI virtual environment is active and run (assuming the working directory is `~/FEAGI/src/`): 
* `$ python3 main.py`

# Setup Guide (containerized)
Users also have the option to deploy FEAGI via Docker containers, thereby automating or eliminating the need for many of the steps listed in the above manual setup. Containerized deployment creates a relatively lightweight environment containing all of the dependencies needed to run FEAGI. The current image (`docker/Dockerfile`) is based on a Debian (Buster) parent image using Python 3.7. Support for a more lightweight parent image (ex: Alpine Linux) to accommodate running FEAGI on resource-constrained devices (such as Raspberry Pi) is expected soon. 

Ensure that Docker and docker-compose are installed on your machine by opening a terminal application and entering: `$ docker --version` and `$ docker-compose --version`. If the outputs of running these commands are **not** _similar_ to `________ version _._._, build _______`, you may need to install Docker and docker-compose. Visit https://www.docker.com/get-started for more information.

**If Docker and docker-compose are installed**:    
Navigate to `FEAGI/docker/` via the terminal and build the Docker image by running: 
* `$ docker-compose build --build-arg REPO=link_to_target_repo --no-cache`, where `link_to_target_repo` is replaced by the target repository URL (https://github.com/feagi/feagi-core.git). **Note**: You should only need to rebuild the image if introducing changes to the Dockerfile or after deleting the existing image.   

Confirm the newly-built image exists:
* `$ docker images`

Start/run the containers: 
* `$ docker-compose up`

Data stored in MongoDB and InfluxDB containers will persist locally in `/docker/mongodb` and `/docker/influxdb`, respectively.
***