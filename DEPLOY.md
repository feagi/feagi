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
Ensure Python 3 (3.7+) is installed: [Download Python 3](https://www.python.org/downloads/)

Assuming `git` is [installed](https://git-scm.com/book/en/v2/Getting-Started-Installing-Git), clone the FEAGI repository by opening a terminal and running:
* `git clone https://github.com/feagi/feagi-core.git`
* **NOTE:** If using Windows, prior to cloning the FEAGI repository, run the following command to ensure proper conversion of file line-endings when committing:
  * `git config --global core.autocrlf input`

* **Additional Windows dependencies:**    
  * Download and install [Visual Studio Build Tools](https://visualstudio.microsoft.com/downloads/).
  * When prompted, select `Desktop Development with C++` and install the accompanying tools.

&nbsp;
## **Virtual Environment Setup**
It is recommended that FEAGI Python dependencies be installed in a virtual environment. 

<details>
  <summary>macOS/Ubuntu</summary>

To create a virtual environment in either Ubuntu or macOS (assuming Python 3 and `virtualenv` are installed), open a terminal and enter (`environment_name` is the desired environment name):

* `$ virtualenv -p /usr/bin/python3 environment_name`

To activate the newly-created virtual environment, run (if successful, environment name should appear in parentheses next to terminal command prompt): 
* `$ source ./<environment_name>/bin/activate`

Install the FEAGI Python dependencies in the active virtual environment (assuming the working directory is `~/feagi-core/`): 
* `$ pip3 install -r requirements.txt`
</details>

<details>
  <summary>Windows</summary>

To create a virtual environment in Windows (assuming `virtualenv` is installed), open a terminal, navigate to `~\feagi-core\` and run (`environment_name` is the desired environment name):    

* `$ virtualenv environment_name`

Activate the newly-created virtual environment by running:
* `$ .\environment_name\Scripts\activate`

Install the Python dependencies:
* `$ pip3 install -r requirements.txt`
</details>

&nbsp;
## **Cythonize Code**
  The directory `/feagi-core/src/cython_lib` contains a Python function (`neuron_functions_cy.pyx`) for updating postsynaptic neuron membrane potential, which is used extensively throughout FEAGI artificial brain creation and learning. Heavy usage of this function requires performance specifications that exceed those of Python in order for FEAGI to run efficiently. This Python code must be compiled into C-like code (i.e. Cythonized) to achieve the necessary performance optimization. To Cythonize the code, run (assuming the working directory is `~/feagi-core/src/cython_lib`): 
  * `$ python3 cython_setup.py build_ext --inplace`

&nbsp;
## **Database Setup**
FEAGI relies on [MongoDB](https://www.mongodb.com/) and [InfluxDB](https://www.influxdata.com/) to store/retrieve genome and time series data, respectively.

<details>
  <summary>MongoDB</summary>     

### **macOS**    
Installation of MongoDB using a package manager such as [Homebrew](https://brew.sh/#install) is recommended. Visit [Install MongoDB on Mac](https://docs.mongodb.com/manual/tutorial/install-mongodb-on-os-x/) for more detailed installation and configuration instructions. Open a terminal and follow these steps:    

Download official MongoDB formula: 
* `$ brew tap mongodb/brew`

Install the (currently) latest version of MongoDB: 
* `$ brew install mongodb-community@4.4`

Start MongoDB as a macOS service: 
* `$ brew services start mongodb-community@4.4`

Confirm MongoDB service has started: 
* `$ brew services list`

&nbsp;
### **Ubuntu**
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

&nbsp;
### **Windows**
To install and configure MongoDB in a Windows environment, view the [installation instructions](https://docs.mongodb.com/manual/tutorial/install-mongodb-on-windows/#install-mongodb-community-edition).
</details>

<details>
  <summary>InfluxDB</summary>

### **macOS**  
As with MongoDB, installation of InfluxDB via Homebrew is recommended. More detailed installation and configuration instructions are available at [Install InfluxDB](https://docs.influxdata.com/influxdb/v1.8/introduction/install/).

Install InfluxDB: 
* `$ brew install influxdb`

Launch InfluxDB: 
* `$ influxd -config /usr/local/etc/influxdb.conf`

&nbsp;
### **Ubuntu**
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

&nbsp;
### **Windows**   
Visit https://portal.influxdata.com/downloads/ and select `Windows Binaries (64-bit)` in the platform dropdown, then run the generated command (found below the platform dropdown) using PowerShell.
</details>

&nbsp;
## **Run FEAGI**
To start FEAGI, open a terminal, ensure the virtual environment where dependencies were installed is active and run (assuming the working directory is `~/feagi-core/src/`): 
* `$ python3 main.py`
