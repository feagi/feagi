**This page may be outdated or incomplete. Please refer to the [deployment guide](https://github.com/feagi/feagi/wiki/Deployment) in our wiki for the most up to date information.**

# Overview
* [Requirements](#Requirements)
  * [Minimum System Requirements](#Minimum-System-Requirements)
  * [Supported Operating Systems](#Supported-Operating-Systems)
  * [Dependencies](#Dependencies)
* [Deployment](#Deployment)
* [Troubleshooting](#Troubleshooting)


# Introduction
The Framework for Artificial General Intelligence (FEAGI) platform consists of core components known as FEAGI-core that is at large a Python package and is responsible for the creation, development, and operation of the artificial brain processes. FEAGI leverages many open-source libraries as part of its ecosystem to provide data visualization, data analytics, 3D simulation, and robot integration 
capabilities. All the needed software packages are containerized to provided a simple method to deploy the software using Docker. This document describes system requirements and the steps to deploy FEAGI platform. 

# Requirements
### Minimum System Requirements
* Free storage:  5 GB
* Memory:        16 GB
* CPU:           4 Cores

### Supported Operating Systems
* **macOS**: Montery (12.3.1) -- Apple and Intel chips are supported
* **Windows**: 10
* **Linux**: Ubuntu (18.04)

### Dependencies
Click on each dependency for installation instructions
* **[Docker](https://docs.docker.com/get-docker/)**: Desktop 4.7.1+, Engine 20.10.14+, Compose v2.4.1+
* **[Python](https://www.python.org/downloads/)**: 3.7+
* **[Git](https://git-scm.com/book/en/v2/Getting-Started-Installing-Git)**: Latest


# Deployment
## 1. Clone the repository to your machine
* Open the terminal or command line and change to the directory in which you would like to clone feagi. 
* Run the command `git clone https://github.com/feagi/feagi.git`

## 2. Launch Docker Desktop
There are multiple ways to launch docker desktop. The easiest is to search for the docker desktop application on your system and open the application. You should see a window that looks like this:  
<img src="https://user-images.githubusercontent.com/52722031/176945282-3fdfc4b7-f82c-4a2b-a8a1-0d6c86ac8acf.png"  width=50% height=50%>


## 3. Build
In the command line/terminal run the following commands:
* `cd [path to where feagi repo was cloned]/feagi/docker`
* `docker compose -f feagi.yml build`

Note: If running on a Mac with Apple chipset, substitute feagi.yml with feagi_apple_silicone.yml throughout this deployment guide.

## 4. Run
* In the feagi/docker folder in the command line, run `docker compose -f feagi.yml up -d ` 
* **NOTE:** The -d prevents the application logs from being displayed. If you would like to see the logs, don't include the -d parameter. 
* To launch the application, open browser to http://localhost:3000. You should see the GUI launch page that looks like this: 
<img src="https://user-images.githubusercontent.com/52722031/176946338-aabb9ab5-7bdb-4103-bb95-c580fde64b04.png"  width=50% height=50%>

## 5. Stopping the Application
Because the application is running in the background, please make sure to always make sure to stop the application when done. There are two ways to do this.
1. Run the command `docker compose -f feagi.yml down` inside the feagi/docker folder on your local system. You will see logs indicating that the containers have been stopped. 
![Screen Shot 2022-07-01 at 2 03 41 PM](https://user-images.githubusercontent.com/52722031/176947412-6ebc1f3a-eb3d-4acf-999c-b6b40c20df77.png)
2. Stop the containers in the docker desktop application. 

# Troubleshooting
This section displays errors that users have encountered when trying to install feagi. If you encounter an error not listed here, please create an issue and we will do our best to help resolve it. Additionally, if you encounter any errors not listed here and you resolve them yourself, please add them here to help others.

### Errors encountered during building

If you encounter any of the following errors, please ensure that docker desktop is installed, launched, and running. These two errors were created when docker desktop was not yet installed or not launched. To launch docker desktop, follow the instructions listed under [Running Docker Desktop](#3.-Running-Docker-Desktop)
1. `Got permission denied while trying to connect to the Docker daemon socket at unix:///var/run/docker.sock: Get "http://%!F(MISSING)var%!F(MISSING)run%!F(MISSING)docker.sock/v1.24/version": dial unix /var/run/docker.sock: connect: permission denied: driver not connecting`

2. ![Screen Shot 2022-07-01 at 1 29 23 PM](https://user-images.githubusercontent.com/52722031/176945407-1b7f3770-d0b2-4cf5-bb40-62aa02f907e7.png)

**To confirm docker desktop is running:**
* On Mac:  Clicking on the docker icon in the top right corner of your screen and ensure that you see "Docker Desktop is running"
<img src="https://user-images.githubusercontent.com/52722031/176945111-016fbeae-5935-4d72-9ec3-20bba7803ee5.png"  width=40% height=40%>

### Errors encountered during running

* If you receive an error indicating you have a port conflict, please follow [these](https://github.com/feagi/feagi/wiki/Customizing-the-Environment#port-mapping) instructions on how to override FEAGI's default port mappings.
