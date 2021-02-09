# Introduction
The framework is known to run stably on OS X and Ubuntu, with deployment support for more operating systems (Windows, etc.) anticipated in the future, if needed. Users may extensively configure the framework according to their preferences by editing the various parameters in the `/src/feagi_configuration.ini` file. It is important that configuration parameters representing a path to a file or directory relevant to FEAGI execution be updated to accurately reflect the user's local environment (see `InitData` parameters in `feagi_configuration.ini`).

# Supported Operating Systems
* **macOS**: High Sierra (10.13)
* **Linux**: Ubuntu (18.04)

# Setup Guide (manual)
## Virtual Environment Setup
It is recommended that FEAGI Python dependencies be installed in a virtual environment. 
* To create a virtual environment in either Ubuntu or macOS, open a terminal and enter: `$ virtualenv -p /usr/bin/python3 environName`, where `environName` is the desired environment name.

* To activate the newly-created virtual environment, run: `$ source ./environName/bin/activate` (if successful, the name of the environment should appear in parentheses next to the terminal command prompt).

* Install the FEAGI Python dependencies in the active virtual environment: `$ pip3 install -r requirements.txt` (assuming the working directory is `~/FEAGI/`).

## Ramdisk Setup
Using a ramdisk when running FEAGI can enhance execution speed by reducing read/write latency when interacting with genome/connectome data; whether this is necessary/useful depends on local physical drive performance specifications. If using a ramdisk, users should monitor its space when running FEAGI successively; each framework execution generates a new directory containing updated brain connectome data in `JSON` format.

### High Sierra (10.13)
* **Create/mount ramdisk**: `$ diskutil partitionDisk $(hdiutil attach -nomount ram://$((2048*diskSize))) GPTFormat HFS+ 'ramdisk' '100%'`, where `diskSize` is user-defined disk size (specified in MB)

* **Eject/unmount ramdisk**: `$ diskutil eject /Volumes/ramdisk` (all data on the disk will be lost)
### Ubuntu (18.04)
* **Create ramdisk mount point**: `$ sudo mkdir /mnt/ramdisk`

* **Mount ramdisk**: `$ sudo mount -t tmpfs -o rw,size=diskSize tmpfs /mnt/ramdisk`, where `diskSize` is specified in gigabytes (ex: `2G`) or megabytes (ex: `10240000M`) - (run `$ df -h` and confirm disk is mounted)

* **Eject/unmount ramdisk**: `$ sudo umount /mnt/ramdisk` (all data on the disk will be lost)

## Cythonize Code
* The directory `/FEAGI/src/cython_libs` contains a Python function (`neuron_functions_cy.pyx`) for updating postsynaptic neuron membrane potential, which is used extensively throughout FEAGI artificial brain creation and learning. Heavy usage of this function requires performance specifications that exceed those of Python in order for FEAGI to run efficiently. This Python code must be compiled into C-like code (i.e. Cythonized) to achieve the necessary performance optimization. To Cythonize the code, run: `$ python3 ./src/cython_libs/cython_setup.py build_ext --inplace` (assuming the working directory is `~/FEAGI/`).

* The `physiology` module (`/src/npu/physiology.py`) imports the Cythonized function from the previous step. This may raise an `ImportError` if running FEAGI in its current state, requiring the addition of `import pyximport` (included in Cython installation) followed by `pyximport.install()` before the `from cython_libs import neuron_functions_cy as cy` statement.

## Run FEAGI
* To start FEAGI, open a terminal, ensure the FEAGI virtual environment is active and run: `$ python3 main.py ./connectome/` (assuming the working directory is `~/FEAGI/src/`).

# Setup Guide (containerized)
Users also have the option to deploy FEAGI via Docker container, thereby automating or eliminating the need for many of the steps listed in the above manual setup. Containerized deployment creates a relatively lightweight environment containing all of the dependencies needed to run FEAGI. The current image (`docker/buster.Dockerfile`) is based on a Debian (Buster) parent image using Python 3.7. Support for a more lightweight parent image (ex: Alpine Linux) to accommodate running FEAGI on resource-constrained devices (such as Raspberry Pi) is expected soon. 

Ensure that Docker is installed on your machine by opening a terminal application and entering: `$ docker --version`. If the output of running this command is **not** _similar_ to `Docker version _._._, build _______`, you may need to install Docker. Visit https://www.docker.com/get-started for more information.

**If Docker is installed**:
* Navigate to `FEAGI/docker/` via the terminal.

* Build the Docker image by running: `$ docker build --no-cache -t feagi:0.2 --build-arg REPO=link_to_target_repo .`, where `link_to_target_repo` is replaced by the target repository URL (https://github.com/feagi/feagi-core.git). **Note**: You should only need to rebuild the image if introducing changes to the Dockerfile or after deleting the existing image.   

* Run `$ docker images` and confirm that the newly-built image (with tag `feagi:0.2`) exists in the list.

* Start/run the container by entering: `$ docker run -t feagi:0.2` and confirm standard output in the terminal indicates that FEAGI is running.
***