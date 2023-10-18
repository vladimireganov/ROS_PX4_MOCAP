# ROS_PX4_MOCAP
ROS px4 and mocap code for flying and connecting.

<!-- > PS. if you need developer's guide click [here](./dev_guide.md) -->

> **Note** '$' - this symbol means that commnad should be executed in the console

> **Note** currently working code is in [px4 folder](https://github.com/vladimireganov/ROS_PX4_MOCAP/blob/f22f6e953b3ab82bf9780bbde9cd918131332230/px4)

## Table of Contents
0. [Required Components and Sowftware](#required-components-and-sowftware)
1. [Installation Guide](#installation-guide).
2. [Configuration Guide](#configuration-guide)
3. [Running Guide](#running-code)
4. [Working with a tool.](#working-with-a-tool)

### Required Components and Sowftware:

#### Components:

- Raspberry Pi
- Pixhawk


#### Software:

Motive (link)

QGroundControl (QGC)


### Installation Guide:
- [Installation of VM](#installation-of-vm-and-linux).
- [Installation of ROS on Desktop](#installing-ros-on-linux-desktop)
- [Installation of ROS on Rpi](#installing-ros-on-rpi)
- [Configuring the system](#configuration-guide)

#### Installation of VM and linux:
> **Note** Skip this step if you already have linux machine set up
- Download virtual box [link](https://www.virtualbox.org/wiki/Downloads).
- Download Linux image [link](https://www.linux.org/pages/download/)

- Install Linux inside virtual box


#### Installing ROS on linux desktop
- Open virtual box
- Install ROS noetic packages following guidelines [link](http://wiki.ros.org/noetic/Installation/Ubuntu)
- There are some caveats, when installing, there might not be available keys for ROS, so check sources

> Or simply run  `$ sh install_ros.sh` .  This is script that automatically installs 'ROS' on the desktop computer

#### Install mocap package for ROS
> `$ sudo apt-get install ros-noetic-mocap-optitrack`

> Ros – means ros package
 Noetic – means ros version, I used “noetic”, it can also be “melodic”


#### Install mavros package for ROS

>`$ sudo apt-get install ros-noetic-mavros ros-noetic-mavros-extras`
- Here we install 2 packages for ros, which contain necessary datatypes

<!-- ``` Bash
$ sudo apt-get install ros-noetic-mocap-optitrack
$ sudo apt-get install ros-noetic-mocap-optitrack
$ sudo apt-get install ros-noetic-mocap-optitrack
$ sudo apt-get install ros-noetic-mocap-optitrack
$ sudo apt-get install ros-noetic-mocap-optitrack
``` -->
#### Installing ROS on RPi
- follow this guide [link](http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Melodic%20on%20the%20Raspberry%20Pi)
> **Note** 


`$ mkdir -p ~/catkin_ws/src `

`$ cd ~/catkin_ws `

`$ catkin init `

`$ wstool init src `

`$ rosinstall_generator --rosdistro melodic mavlink | tee /tmp/mavros.rosinstall `

`$ rosinstall_generator --upstream-development mavros mavros_extras mavros_msgs test_mavros sensor_msgs control_toolbox realtime_tools tf tf2_ros python_orocos_kdl urdf |tee -a /tmp/ mavros.rosinstall `

`$ wstool merge -t src /tmp/mavros.rosinstall `

`$ wstool update -t src -j4 `

`$ rosdep install --from-paths src --ignore-src -y `
  or
`rosdep install --from-paths src --ignore-src --rosdistro melodic -y -r --os=debian:buster`

`$ sudo ./src/mavros/mavros/scripts/install_geographiclib_datasets.sh `

`$ catkin build -DCATKIN_ENABLE_TESTING=0 `


Some keys might not be available.
Time consuming, it can take few hours to install.
Some cmds might not work from the first time, be sure to check that every cmd executed properly.
Choose bare bones or minimal version of ROS, it will take least time to install, also some packages for graphical interface might not be compatible for RPi.
Check that everything is compatible with Hardware.
In general, it is the hardest and most time-consuming part.

#### Compilig code

in order to compile you have to be in root directory of the project

`$ catkin_make`

> **Note** it compiles the whole directory
    

# Configuration Guide

### Setting ROS upon multiple machines
Desktop

`$ export ROS_MASTER_URI=‘http://172.19.90.34:11311’`

`$ export ROS_IP=172.19.90.34`

RPi

`$ export ROS_MASTER_URI=‘http://172.19.90.34:11311’`

`$ export ROS_IP=172.19.90.64`

> Host will have both ip addresses the same, client will have ROS_MASTER_URI the same as host, ROS_IP as the client ip address

> To get ip adress `$ ip a`

### Data streaming

- Enable data streaming in motive
[picture]
- Configure mocap.yaml to match streaming id and name

### Configuring mocap.yaml

First, move to the directory of "mocap.yaml" file

>`$ cd` 

>`$ roscd mocap_optitrack`

>`$ nano mocap.yaml`


### Configuring px4 settings

## px4 and mocap
Using QGC connect to px4, then go to EKF2 settings
![img1](https://github.com/vladimireganov/ROS_PX4_MOCAP/blob/main/images/QGC.png)

# Running code
On main machine (in separate tabs each command)
 
 `$ roscore`
 
 `$ roslaunch mocap_optitrack mocap.launch –screen`
 
 `$ rosrun listener test.py` python code for data streaming from mocap into proper ROS node, it also sets the proper streaming rate
 
 `$ rosrun arm test_trajectory_NED`
 
On Rpi

 `$ roslaunch mavros px.launch`

# Working with a tool

### Code structure
- include/

    - api.hpp – contains main class for working with drone

    - File_read.hpp – contains class to read trajectory from “trajectory.scv” file

    - root_finder.hpp and traj_min_snap.hpp – contains classes and methods for calculating trajectory using minimum snap/jerk

    - file_write.hpp – contains class for saving logs from main code

 - src/

    - test_trajectory.cpp – main code




### Links


link for [motion_capture](https://docs.px4.io/main/en/tutorials/motion-capture.html).
link for [PX4 Autopilot GitHub](https://github.com/PX4/PX4-Autopilot).
link for [mavros](http://wiki.ros.org/mavros).
link for [mavros_extras](http://wiki.ros.org/mavros_extras).(Extends mavros)


> add links to github for px4
> add links for other important links
