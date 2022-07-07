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
    float x,y,z,yaw;

    // my_drone.set_home(); //
    my_drone.refresh_set_point_NED(); // init set_point to current state
    my_drone.get_position_ret(x,y,z,yaw);

    my_drone.get_position();
    my_drone.init_heading();
    my_drone.set_heading_global(yaw);

    for (int i = 0; i < 100; i++) // send a few initial set points before continueing
    {
        my_drone.march_NED(); ///send cmds
    }

    my_drone.arm(); // try to arm
    my_drone.set_mode(std::string("OFFBOARD")); //try to transition into offboard mode

    // my_drone.set_heading_offset(1.5708);
    my_drone.set_timer(25.0);
    while(ros::ok() && ! my_drone.check_timer()){ //while loop for main program
        my_drone.march_NED();//spin code (publish set points)
    }

    my_drone.landing();
    my_drone.land();

    my_drone.set_timer(5.0);
    while ( ! my_drone.check_timer() &&ros::ok()){}
    ROS_INFO("Landed:\n");
    my_drone.disarm();




    return 0;
}