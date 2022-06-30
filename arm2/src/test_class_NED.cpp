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
    
    // my_drone.set_home(); //
    my_drone.refresh_set_point_NED(); // init set_point to current state
    my_drone.set_attitude(0);
    // my_drone.take_off_NED(1);
    // my_drone.set_attitude(0,0,0);
    // my_drone.refresh_set_point();

    // my_drone.take_off_2(1);
    for (int i = 0; i < 100; i++) // send a few initial set points before continueing
    {
        // my_drone.march();
        my_drone.march_NED(); ///send cmds
        // my_drone.set_attitude();
    }
    // my_drone.set_attitude(0,0,0);
    my_drone.arm(); // try to arm
    my_drone.set_mode(std::string("OFFBOARD")); //try to transition into offboard mode
    ROS_INFO("Take off\n");
    my_drone.take_off_NED(1); // go .5 m up
    my_drone.set_attitude(0);
    while(ros::ok() && !my_drone.reached_point_NED()){ //while loop for main program
        my_drone.march_NED();//spin code (publish set points)
    }
    ROS_INFO("Take off completed\n");

    my_drone.set_attitude(1.57);
    ROS_INFO("Rotate\n");
    my_drone.set_timer(5.0);
    while ( ! my_drone.check_timer() &&ros::ok()){my_drone.march_NED();}

    my_drone.set_point_NED(1,0);
    my_drone.set_attitude(0);
    while(ros::ok() && !my_drone.reached_point_NED()){ //while loop for main program
        my_drone.march_NED();//spin code (publish set points)
    }
    ROS_INFO("Reached first point:\n");
    my_drone.set_point_NED(0,1);
    my_drone.set_attitude(0);
    while(ros::ok() && !my_drone.reached_point_NED()){ //while loop for main program
        my_drone.march_NED();//spin code (publish set points)
    }
    ROS_INFO("Reached second point:\n");

    my_drone.set_point_NED(-1,0);
    my_drone.set_attitude(0);
    while(ros::ok() && !my_drone.reached_point_NED()){ //while loop for main program
        my_drone.march_NED();//spin code (publish set points)
    }
    ROS_INFO("Reached third point:\n");

    my_drone.set_point_NED(0,-1);
    my_drone.set_attitude(0);
    while(ros::ok() && !my_drone.reached_point_NED()){ //while loop for main program
        my_drone.march_NED();//spin code (publish set points)
    }
    ROS_INFO("Reached third point:\n");

    my_drone.landing();
    my_drone.land();
    // while(ros::ok() && !my_drone.reached_point()){ //while loop for main program
    //     my_drone.march();//spin code (publish set points)
    // }
    my_drone.set_timer(5.0);
    while ( ! my_drone.check_timer() &&ros::ok()){}
    ROS_INFO("Landed:\n");
    my_drone.disarm();




    return 0;
}