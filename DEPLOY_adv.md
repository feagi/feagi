

# Advanced Deployment



==============================================

# **Introduction**
It is highly recommended for all users to deploy FEAGI leveraging docker containers using instructions located ***[here](./DEPLOY.md)***. 

If you are interested in a custom deployment of FEAGI for an environment not supporting docker containers follow 
instructions on this page.


Advanced deployment involves the manual installation of FEAGI and all associated third-party softwares independently and 
having them all configured to work together. 


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
* **npm**:    8+


# Deployment
## 1. Clone the repo  
Ensure Python 3 (3.7+) is installed: [Download Python 3](https://www.python.org/downloads/)

Assuming `git` is [installed](https://git-scm.com/book/en/v2/Getting-Started-Installing-Git), clone the FEAGI repository by opening a terminal and running:
* `git clone https://github.com/feagi/feagi.git`



**Windows Users:** Prior to cloning the FEAGI repository, run the following command to ensure proper conversion of file line-endings when committing:
  * `git config --global core.autocrlf input`

  * **Additional Windows dependencies:**    
    * Download and install [Visual Studio Build Tools](https://visualstudio.microsoft.com/downloads/).
    * When prompted, select `Desktop Development with C++` and install the accompanying tools.

&nbsp;
## **2. Setup Virtual Environment**
It is recommended that FEAGI Python dependencies be installed in a virtual environment. 

<details>
  <summary>macOS/Ubuntu</summary>

To create a virtual environment in either Ubuntu or macOS (assuming Python 3 and `virtualenv` are installed), open a terminal and enter (`environment_name` is the desired environment name):

* `$ virtualenv -p /usr/bin/python3 environment_name`

To activate the newly-created virtual environment, run (if successful, environment name should appear in parentheses next to terminal command prompt): 
* `$ source ./<environment_name>/bin/activate`

Install the FEAGI Python dependencies in the active virtual environment (assuming the working directory is `~/feagi/`): 
* `$ pip3 install -r requirements.txt`
</details>

<details>
  <summary>Windows</summary>

To create a virtual environment in Windows (assuming `virtualenv` is installed), open a terminal, navigate to `~\feagi\` and run (`environment_name` is the desired environment name):    

* `$ virtualenv environment_name`

Activate the newly-created virtual environment by running:
* `$ .\environment_name\Scripts\activate`

Install the Python dependencies:
* `$ pip3 install -r requirements.txt`
</details>

&nbsp;
## **3. Setup Databases(Optional)**
FEAGI utilizes [MongoDB](https://www.mongodb.com/) for storing genomes as part of its evolutionary features and utilizes 
[InfluxDB](https://www.influxdata.com/) to store time-series data for monitoring artificial brain activities. If you are 
not planning to use either of these feature you can skip this step.

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
## **4. Configure FEAGI launch parameters(Optional)**
FEAGI run-time operations can be configured using `src/feagi_configuration.ini`


&nbsp;
## **5. Setup Godot Game Engine(Optional)**


&nbsp;
## **6. Setup ROS + Gazebo(Optional)**


&nbsp;
## **7. Setup Grafana(Optional)**



&nbsp;
## **8. Start FEAGI Services**
To start FEAGI, open a terminal, ensure the virtual environment where dependencies were installed is active and run (assuming the working directory is `~/feagi/src/`): 
* `$ python3 main.py`


&nbsp;
## **9. Launch FEAGI**
### REST API
1. Open 127.0.0.1:8000
2. Start FEAGI processes by triggering one of the genome upload endpoints.

### GUI
1. navigate to /src/gui
2. `npm install`
3. `npm run`
4. Open 127.0.0.1:3000
5. Follow the GUI workflow