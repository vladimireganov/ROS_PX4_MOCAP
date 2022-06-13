#include <chrono>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>


#include "api.h"
#include <string>

int main(int argc, char **argv){

    ros::init(argc, argv, "offb_node"); // needed to init ros node

    api my_drone = api(argc, argv); //initialize class to work with drone
    
    my_drone.set_home(); //
    my_drone.refresh_set_point(); // init set_point to current state

    for (int i = 0; i < 100; i++) // send a few initial set points before continueing
    {
        my_drone.take_off(0.5); // go .5 m up
        my_drone.march(); //send cmds
    }
    
    my_drone.arm(); // try to arm
    my_drone.set_mode(std::string("OFFBOARD")); //try to ransition into offboard mode


    while(ros::ok()){ //while loop for main program
        my_drone.march();//spin code (publish set points)
        if (my_drone.reached_point()){ //check if point was reached
            ROS_INFO(" reached target position  \n");
        }
    }
    return 0;
}