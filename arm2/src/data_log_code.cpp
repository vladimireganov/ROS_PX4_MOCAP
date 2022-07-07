#include <chrono>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>


#include "api.h"
#include <string>

/*
* add file write
* Syncronization?
* Will be performed via logging time from header
*/

namespace log_data{

static void state_cb(const mavros_msgs::State::ConstPtr& msg); //callback function for current state
static void get_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg); // callback function for current positon

static void get_pos_drone_cb(const mavros_msgs::State::ConstPtr& msg); //callback function for current state
static void get_pos_target_cb(const mavros_msgs::PositionTarget::ConstPtr& msg); // callback function for current positon
/*
    * call back function to get state of the vehicle
*/
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
/*
    * call back function to get position of the drone
*/
void get_pos(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_position = *msg;
}
}


int main(int argc, char **argv){

    ros::init(argc, argv, "log_node");

    ros::Subscriber state_sub;
    ros::Subscriber pos_sub;
    ros::Subscriber pos_drone_sub;
    ros::Subscriber set_point_raw_sub;


    state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, log_data::state_cb);

    pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose",10,log_data::get_pos_cb);

    pos_drone_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose",10,log_data::get_pos_drone_cb);

    set_point_raw_sub = nh.subscribe<mavros_msgs::PositionTarget> 
            ("mavros/setpoint_raw/local", 10, log_data::get_pos_target_cb);

}