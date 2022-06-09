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
    geometry_msgs::PoseStamped position;
    geometry_msgs::PoseStamped current_position;
    geometry_msgs::PoseStamped setpoint_position;

    geometry_msgs::PoseStamped home; // home position

    mavros_msgs::SetMode offb_set_mode; // setting mode

    /* data */
    void state_cb(const mavros_msgs::State::ConstPtr& msg); //callback function for current state
    void get_pos(const geometry_msgs::PoseStamped& msg); // callback function for current positon

    ros::init(argc, argv, "offb_node"); //create node
    ros::NodeHandle nh; //create node handling

    mavros_msgs::CommandBool arm_cmd; //variable for arming

    // ros topics and services
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
    bool set_mode(string mode);

    void take_off(float altitude);
    void landing(); //updates altitude for landing
    void set_point(float x, float y , float z);
    void set_point(float x, float y); // for horizontal flight

    void set_home();
    void refresh_set_point();
    void reset();
    void march();  /// applying all changes anf flying
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
    /// set streaming rate
    ros::Rate rate(20.0);
    ///

    // subcribe to state of drone
    state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    // subcribe to px4 position
    pos = nh.subscribe<geometry_msgs::PoseStamped>("/mocap_node/drone_3/pose",10,get_pos);// subsribe to topi with proper coordinate system

    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);

    arming_client = nh.serviceClient<mavros_msgs::CommandBool> //client to arm
            ("mavros/cmd/arming");

    set_mode_client = nh.serviceClient<mavros_msgs::SetMode> //client to set mode
            ("mavros/set_mode");

    ///setpoint_raw - topic to test
    ///mission/reached - topic to test

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
    current_position = *msg;
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
        ROS_INFO("Vehicle disarmed");
        return true;
    }
    else return false;
}


/// function to change mode
bool api::set_mode(string mode){
    offb_set_mode.request.custom_mode = "OFFBOARD";
    if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
                return true;
    }
    else{
        return false;
    }
}

void api::take_off(float altitude){
    setpoint_position.pose.position.z = altitude;

}

void api::landing(){
    setpoint_position.pose.position.z = home.pose.position.z;
}

void api::march(){
    local_pos_pub.publish(setpoint_position);
    ros::spinOnce();
    rate.sleep();
}

void api::refresh_set_point(){
    setpoint_position = current_position;
}

void api::set_point(float x, float y , float z){
    setpoint_position.pose.position.x = x;
    setpoint_position.pose.position.y = y;
    setpoint_position.pose.position.z = z;
}

void api::set_point(float x, float y){
    setpoint_position.pose.position.x = x;
    setpoint_position.pose.position.y = y;

}
void api::set_home(){
    home = current_position;
}