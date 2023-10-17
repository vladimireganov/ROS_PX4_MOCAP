# Developer's Guide
This guide is meant for the developer's & maintainers of the project.


## Project folders:

- listener - folder that contains ROS package for coordinate transformation in python
- px4 - folder with main code for PX4 written in C++
- arm2 - folder with another C++ for PX4
- trajectories - folder that contains coordinates of several trajectories
- images - folder with images for GitHub to look pretty
- analysis - folder with python code that allows to do post flight analysis 
- large_scale_traj_optimizer - 2 folders, from repository that contains minimum snap(jerk) algorithm implemennted in C++ and ROS
- .vscode - settings for Visual Studio Code

## Files in root directory of GitHub:

- extract_data.sh - Bash script to effectively copy data from ?Flash drive into folder
- install_ros.sh - Bash script to install ROS from scratch (outdated)
- ros_info_mavros.txt - list with result of running `$rosinfo` cmd 
- rosservice.txt - list with all services available for mavros
- rostopics.txt - list with all topics available for mavros



## Oops, there is no guide :(
![SpongeBob on Fire](https://c.tenor.com/iDYg-7xD7M4AAAAC/burning-office-spongebob.gif)