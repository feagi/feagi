# **Introduction**
FEAGI is known to run stably on macOS, Ubuntu and Windows, with deployment support for more operating systems anticipated in the future, if needed. Users may extensively configure the framework according to their preferences by editing the various parameters in the `/src/feagi_configuration.ini` file. It is important that configuration parameters representing a path to a file or directory relevant to FEAGI execution be updated to accurately reflect the user's local environment (see `InitData` parameters in `feagi_configuration.ini`). FEAGI data output is, by default, stored in the host system's temporary directory. To customize this parameter, enter the desired output path in `working_directory` under `InitData` in `feagi_configuration.ini`.

&nbsp;
# **Setup Guide (containerized)**
Users have the option to deploy FEAGI via a Docker container, thereby automating or eliminating the need for many of the steps listed in the manual setup. Containerized deployment creates a relatively lightweight environment containing all of the dependencies needed to run FEAGI and any other integrated services. FEAGI images can be created as standalone or networked with other service images (ex: ROS2, Gazebo, mongoDB, InfluxDB) via `docker-compose`. 

Ensure that Docker and docker-compose are installed on your machine by opening a terminal application and entering: `$ docker --version` and `$ docker-compose --version`. If the outputs of running these commands are **not** _similar_ to `________ version _._._, build _______`, you may need to install Docker and docker-compose. Visit https://www.docker.com/get-started for more information.

**If Docker and docker-compose are installed**:    
To build a standalone FEAGI image and start the container, navigate to `~/feagi-core/docker` via the terminal and run:
* `docker build -f Dockerfile . -t <image_name>`    
(where `-t <image_name>` is an optional way to give the image a user-defined name - if this is not specified, Docker will give the resulting image a random name that must be used when running the container in the next step).
* `docker run -it <image_name>`

To build a FEAGI image for use with other service images (ROS2, Ignition Gazebo, Grafana, InfluxDB) using `docker-compose`, navigate to `~/feagi-core/docker` and run:
* `$ docker-compose -f feagi.yml build`   
* `$ docker-compose -f feagi.yml up`

&nbsp;
# **Setup Guide (manual)**    

 

## **Virtual Environment Setup**
It is recommended that FEAGI Python dependencies be installed in a virtual environment. 

<details>
  <summary>macOS/Ubuntu</summary>
  To create a virtual environment in either Ubuntu or macOS (assuming `virtualenv` is installed), open a terminal and enter (`environment_name` is the desired environment name):

  * `$ virtualenv -p /usr/bin/python3 environment_name`

  To activate the newly-created virtual environment, run (if successful, environment name should appear in parentheses next to terminal command prompt): 
  * `$ source ./<environment_name>/bin/activate`

  Install the FEAGI Python dependencies in the active virtual environment (assuming the working directory is `~/feagi-core/`): 
  * `$ pip3 install -r requirements.txt`
</details>

<details>
  <summary>Windows</summary>
To create a virtual environment in Windows (assuming `virtualenv` is installed), open a terminal, navigate to `~\feagi-core\` and run (`environment_name` is the desired environment name):
* `virtualenv environment_name`

Activate the newly-created virtual environment by running:
* `.\environment_name\Scripts\activate`

Install the Python dependencies:
* `pip3 install -r requirements.txt`
</details>

&nbsp;
## **Cythonize Code**
The directory `/feagi-core/src/cython_lib` contains a Python function (`neuron_functions_cy.pyx`) for updating postsynaptic neuron membrane potential, which is used extensively throughout FEAGI artificial brain creation and learning. Heavy usage of this function requires performance specifications that exceed those of Python in order for FEAGI to run efficiently. This Python code must be compiled into C-like code (i.e. Cythonized) to achieve the necessary performance optimization. To Cythonize the code, run (assuming the working directory is `~/feagi-core/src/cython_lib`): 
* `$ python3 cython_setup.py build_ext --inplace`

&nbsp;
## **Database Setup**
FEAGI relies on [MongoDB](https://www.mongodb.com/) and [InfluxDB](https://www.influxdata.com/) to store/retrieve genome and time series data, respectively.
`test`
<details>
<summary>MongoDB</summary>     
  Installation of MongoDB using a package manager such as [Homebrew](https://brew.sh/#install) is recommended. Visit [Install MongoDB on Mac](https://docs.mongodb.com/manual/tutorial/install-mongodb-on-os-x/) for more detailed installation and configuration instructions. Open a terminal and follow these steps:    

  Download official MongoDB formula: 
  * `$ brew tap mongodb/brew`

  Install the (currently) latest version of MongoDB: 
  * `$ brew install mongodb-community@4.4`

  Start MongoDB as a macOS service: 
  * `$ brew services start mongodb-community@4.4`

  Confirm MongoDB service has started: 
  * `$ brew services list`


<details>
  <summary>Ubuntu</summary>

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
</details>

<details>
  <summary>Windows</summary>

Normally, `pip install -r requirements.txt` will be done everything for this.
You can download MongoDB only through the pip install. Doing so by run pip install Pymongo:
* `pip install PyMongo`


</details>
</details>

<details>
  <summary>InfluxDB</summary>

**macOS**  
<details>
  <summary>Click here to install on MacOs</summary>
As with MongoDB, installation of InfluxDB via Homebrew is recommended. More detailed installation and configuration instructions are available at [Install InfluxDB](https://docs.influxdata.com/influxdb/v1.8/introduction/install/).

Install InfluxDB: 
* `$ brew install influxdb`

Launch InfluxDB: 
* `$ influxd -config /usr/local/etc/influxdb.conf`

</details>

**Ubuntu**
<details>
  <summary>Click here to install on Ubuntu</summary>

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

</details>

**Windows**   
<details>
  <summary>Click here to install on Windows</summary>

This should be able to done by `pip install -r requirements.txt`
* `pip install influxdb_client`

</details>
</details>

[//]: # (## Run FEAGI)
<details>
  <summary>Run FEAGI</summary>

To start FEAGI, open a terminal, ensure the FEAGI virtual environment is active and run (assuming the working directory is `~/feagi-core/src/`): 
* `$ python3 main.py` **in Ubuntu/MacOs only**
* `python main.py` **in Windows only**

</details>

&nbsp;
## Stuck?
Here is the full and detailed steps available in the [FAQ](Deployment_FAQ.md).
