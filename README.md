# ROS_PX4_MOCAP
ROS px4 and mocap code for flying and connecting.

> PS. if you need developer's guide click [here](./dev_guide.md)

## Table of Contents
1. [Installation Guide](#installation-guide).
2. Configuration Guide
3. Running Guide.
4. Working with a tool.

### Installation Guide:
- [Installation of VM](#installation-of-vm).
- Big Step 2

#### Installation of VM:
- Download virtual box [link](https://www.virtualbox.org/wiki/Downloads).
- Downloads – Oracle VM VirtualBox
- Download Linux image
  * Download Linux | Linux.org
  * Download Linux | Linux.org
  * Download Linux | Linux.org
  * Download Linux | Linux.org
- Install Linux inside virtual box
> **Note** bla bla bla

#### Installing ROS on linux desktop
- Open virtual box
- Install ROS noetic packages following guidelines noetic/Installation/Ubuntu - ROS Wiki
- There are some caveats, when installing, there might not be available keys for ROS, so check sources
- Install mocap package for ROS
- `$ sudo apt-get install ros-noetic-mocap-optitrack`
- Ros – means ros package
- Noetic – means ros version, I used “noetic”, it can also be “melodic”
- Install mavros package for ROS
- $ sudo apt-get install ros-kinetic-mavros ros-kinetic-mavros-extras
- Here we install 2 packages for ros, which also contain necessary datatypes
