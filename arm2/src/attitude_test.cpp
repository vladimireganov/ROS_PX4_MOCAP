#include <chrono>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>


#include "api.h"
#include <string>
///
/*
//logic for square flight
int points_x = [1,0,-1,0];
int points_y = [0,1,0,-1];
*/
///
int main(int argc, char **argv){

    ros::init(argc, argv, "offb_node"); // needed to init ros node

    api my_drone = api(argc, argv); //initialize class to work with drone

    my_drone.set_attitude(0,0,30);
    my_drone.set_point_NED(1.0,1.0,1.0);
    for (int i = 0; i < 100; i++) // send a few initial set points before continueing
    {
        
        my_drone.march_NED(); //send cmds
    }
    my_drone.set_mode(std::string("OFFBOARD"));

    while(ros::ok()){ //while loop for main program
        my_drone.march_NED();//spin code (publish set points)
    }


    return 0;
}