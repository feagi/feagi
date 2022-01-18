# Frequently Asked Questions with FEAGI
## Abstract

This documentation will be focus on each OS and share the detail information how to load FEAGI on your OS. There will be three different softwares. We will start with the Windows, Mac and Ubuntu respectively. This will include all detail steps to ensure you were not stuck somewhere in the step.

Here is the table of progress on everything.

| OS            | Local     | Docker    | Documentation                                  | 
|--------------|-----------|-----------|---------------
| Windows 10   | In progress   | Supported | in progress   
| Ubuntu 20.04 | Supported | Supported | README.md                                
| Mac OS x     |  -        | -         | - 

## Q: Can you show me how to deploy them from the very beginning?
**Answer**: Here is how to deploy them from the very beginning. Select the OS you are using and the type of deployment.
<details>
<summary>Windows Deployment</summary>

[//]: # (## Windows deployment)
<details>
  <summary>Download requirement</summary>

 1. [Github](https://desktop.github.com)
 2. [Docker](https://hub.docker.com/editions/community/docker-ce-desktop-windows)
 3. [Git](https://git-scm.com/downloads)

 Download them then follow the steps next.
</details>

    
### Deploy container on FEAGI
<details>
  <summary>Windows</summary>

**Important information:**
Before you pull feagi-core from github, be sure to run this in git:
`git config --global core.autocrlf input` So this way, you can disable the convert from CRFL to LF

 After that, just run in git using commands:
 1. Type this in git, `git clone git@github.com:feagi/feagi-core.git`
 2. Navigate to feagi-core/docker/
 3. Type this in the git, `docker-compose -f feagi.yml build --no-cache`
 4. Once it's complete, run this: `docker-compose -f feagi.yml up`
 5. Press ctrl and click 3 different links: [ROS/Gazebo](http://127.0.0.1:6080/), [Godot](http://127.0.0.1:6081/), and [Grafana](http://localhost:3000/)
 6. Feel free to adjust the window or tab to view all at once.

</details>

### Deploy FEAGI on local
<details>
  <summary>Windows </summary>

 **Important information:**
 Before you pull feagi-core from github, be sure to run this in git:
 `git config --global core.autocrlf input` So this way, you can disable the convert from CRFL to LF
 
 Requirement: 
 1. [Download Python 3.8](https://www.python.org/downloads/)
 2. [Build Tools for Visual Studio 2022](https://visualstudio.microsoft.com/downloads/)
 3. Download C++ inside the Visual Studio installer


 This will be a little different from the container where you can simply load feagi in your local using the localhost. Type this in cmd
 1. Type this in git, `git clone git@github.com:feagi/feagi-core.git`
 2. Navigate to feagi-core/src/
 3. Type this in git, `notepad feagi_configuration.ini`
 4. Search the keyword, "ros_gazebo" and replace it to "127.0.0.1" without quotes and save the file.
 5. Navigate to feagi-core/
 6. Type this each line below in git:
 ```
 pip3 install virtualenv
 virtualenv env
 ./env/script/activate
 pip3 install -r requirements.txt
 cd src/cython_lib
 python cython_setup.py build_ext --inplace
 cd ..
 python main.py
 ``` 

 **Be sure to run main.py inside feagi-core/src/**

</details>
</details>

<details>
<summary>Linux Deployment</summary>

<details>
<summary>Download requirement </summary>

</details>
