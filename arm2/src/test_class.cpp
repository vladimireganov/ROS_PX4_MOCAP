#include <chrono>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>


#include "api.h"
#include <string>

int main(){
    
    api my_drone;
    
    my_drone.set_home();
    my_drone.refresh_set_point();

    for (int i = 0; i < 100; i++)
    {
        my_drone.take_off(1);
        my_drone.march();
    }
    
    my_drone.arm();
    my_drone.set_mode("OFFBOARD");


    while(ros::ok()){ //while loop for main program
        my_drone.march();
    }
    return 0;
}