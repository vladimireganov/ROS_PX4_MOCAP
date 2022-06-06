#include <chrono>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

class api
{
private:
    mavros_msgs::State current_state;
    geometry_msgs::PoseStampe position;
    geometry_msgs::PoseStampe current_position;
    geometry_msgs::PoseStampe setpoint_position;
    /* data */
    void state_cb(const mavros_msgs::State::ConstPtr& msg);
    void get_pos(const geometry_msgs::PoseStamped& msg);

    ros::init(argc, argv, "offb_node"); //create node
    ros::NodeHandle nh; //create node handling

    mavros_msgs::CommandBool arm_cmd; //variable for arming

    ros::Subscriber state_sub;
    ros::Subscriber pos;
    ros::Publisher local_pos_pub;
    ros::ServiceClient arming_client;
    ros::ServiceClient set_mode_client;


public:
    api(/* args */);
    ~api();
    bool arm();
    bool disarm();
};
/// functions
/*
- arm
- disarm
- switch to off board
- take off
- landing
- set point point
- set point velocity
/- set point acceleration
- state?

*/
///

/*
* class to work with px4
*/
api::api(/* args */)
{
    // subcribe to state of drone
    state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    // subcribe to px4 position
    pos = nh.subscribe<geometry_msgs::PoseStamped>("/mocap_node/drone_3/pose",10,get_pos);

    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);

    arming_client = nh.serviceClient<mavros_msgs::CommandBool> //client to arm
            ("mavros/cmd/arming");

    set_mode_client = nh.serviceClient<mavros_msgs::SetMode> //client to set mode
            ("mavros/set_mode");

}

api::~api()
{
}

/*
*
*/
void api::state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
    ROS_INFO("%f\n",current_state);
}
void api::get_pos(const geometry_msgs::PoseStamped& msg){
    position = *msg;
}

bool api::arm(){
    arm_cmd.request.value = true;
    if( arming_client.call(arm_cmd) && arm_cmd.response.success){
        ROS_INFO("Vehicle armed");
        return true;
    }
    else return false;
}

bool api::disarm(){
    arm_cmd.request.value = false;
    if( arming_client.call(arm_cmd) && arm_cmd.response.success){
        ROS_INFO("Vehicle armed");
        return true;
    }
    else return false;
}