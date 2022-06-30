#include <chrono>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/PositionTarget.h>
// added libs
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/WaypointPush.h>



#include <string>

#define DEBUG

static void state_cb(const mavros_msgs::State::ConstPtr& msg); //callback function for current state
static void get_pos(const geometry_msgs::PoseStamped::ConstPtr& msg); // callback function for current positon

static mavros_msgs::State current_state;
static geometry_msgs::PoseStamped current_position;



class api
{
private:

    ros::NodeHandle nh; //create node handling

    geometry_msgs::PoseStamped position;
    
    geometry_msgs::PoseStamped setpoint_position;

    geometry_msgs::PoseStamped home; // home position

    mavros_msgs::SetMode offb_set_mode; // setting mode

    /* data */
    
    ros::Rate rate = ros::Rate(30.0);
    
    mavros_msgs::CommandBool arm_cmd; //variable for arming

    // ros topics and services
    ros::Subscriber state_sub;
    ros::Subscriber pos;
    ros::Publisher local_pos_pub;
    ros::ServiceClient arming_client;
    ros::ServiceClient set_mode_client;

    /// experimental features

    geometry_msgs::Twist set_vel;
    geometry_msgs::Vector3Stamped set_accel;
    mavros_msgs::PositionTarget set_point_raw;

    ros::Publisher set_vel_pub;
    ros::Publisher set_accel_pub;
    ros::Publisher set_point_raw_pub;

    ros::Time timer_start;
    ros::Duration dt;
    float dest_threshold;

    ros::Publisher set_attitude_pub;
    /// experimental finish
    mavros_msgs::CommandTOL land_cmd;
    ros::ServiceClient land_client;


public: 
    



    api(int argc, char **argv);
    ~api();
    bool arm();
    bool disarm();
    bool set_mode(std::string mode);

    void take_off(float altitude);
    void take_off_2(float altitude);

    
    void landing(); //updates altitude for landing

    void set_point(float x, float y , float z);
    void set_point(float x, float y); // for horizontal flight

    void set_point_2(float x, float y , float z);
    void set_point_2(float x, float y); // for horizontal flight

    void set_point_NED(float x, float y , float z); //data sent in NED 
    void set_point_NED(float x, float y);

    void set_home(); // sets home position
    void refresh_set_point(); // refreshes set point to current location
    void refresh_set_point_NED();
    void reset(); // reset???
    void march();  /// applying all changes anf flying

    void march_NED();

    bool reached_point(); // function for checking if point reached 
    
    /// experimnetal features
    void set_velocity(float x, float y, float z);

    void set_acceleration(float x, float y, float z);

    void set_timer(double delta);

    bool check_timer();

    void set_attitude(float yaw);
    void land();
    void take_off_NED(float altitude);
    bool reached_point_NED();
    /// experimental finish
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
* @args - input argumets from main
*/
api::api(int argc, char **argv)
{
     //create node
    

    /// set streaming rate
    //rate = ros::Rate(20.0);
    ///
    

    // subcribe to state of drone
    state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    // subcribe to px4 position
    pos = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose",10,get_pos);// subsribe to topi with proper coordinate system

    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);

    arming_client = nh.serviceClient<mavros_msgs::CommandBool> //client to arm
            ("mavros/cmd/arming");

    set_mode_client = nh.serviceClient<mavros_msgs::SetMode> //client to set mode
            ("mavros/set_mode");

    set_vel_pub = nh.advertise<geometry_msgs::Twist>
            ("mavros/setpoint_velocity/cmd_vel_unstamped", 1);
    set_accel_pub = nh.advertise<geometry_msgs::Vector3Stamped>
            ("mavros/setpoint_accel/accel", 1);
    set_point_raw_pub = nh.advertise<mavros_msgs::PositionTarget>
            ("mavros/setpoint_raw/local", 10);
    land_cmd.request.yaw = 0.0;
    land_cmd.request.latitude = 0;
    land_cmd.request.longitude = 0;
    land_cmd.request.altitude = 0;
    land_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");
    /// experimnental finish
    dest_threshold = .1;

    set_point_raw.coordinate_frame = 1;
    set_point_raw.type_mask = 0;
    set_point_raw.yaw = 1.0;
    set_point_raw.yaw_rate = .1;
}

api::~api()
{
}

/*
    * call back function to get state of the vehicle
*/
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
    // ROS_INFO("%f\n",current_state);
}
/*
    * call back function to get position of the drone
*/
void get_pos(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_position = *msg;
}
/*
    * function to arm the drone
    * @return - true/false if arming was sucessful
*/
bool api::arm(){
    arm_cmd.request.value = true;
    if( arming_client.call(arm_cmd) && arm_cmd.response.success){
        ROS_INFO("Vehicle armed");
        return true;
    }
    else return false;
}

/*
    * function to disarm the drone
    * @return - true/false if disarming was sucessful
*/
bool api::disarm(){
    arm_cmd.request.value = false;
    if( arming_client.call(arm_cmd) && arm_cmd.response.success){
        ROS_INFO("Vehicle disarmed");
        return true;
    }
    else return false;
}


/// function to change mode
/*
    * @param - mode to transition to
    * @return - true on success
    * supported:
        "OFFBOARD"
        "MANUAL"
*/
bool api::set_mode(std::string mode){
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
/*
    * function to take off
    * @param - altitude to go to

*/
void api::take_off(float altitude){
    setpoint_position.pose.position.z = current_position.pose.position.z + altitude;
}
/*
    * function to take off
    * @param - altitude to go to

*/
void api::take_off_2(float altitude){
    setpoint_position.pose.position.z += altitude;
}
void api::take_off_NED(float altitude){
    set_point_raw.position.z += altitude;
}

/*
    * function to land
    * not yet works turning off motors

*/
void api::landing(){
    setpoint_position.pose.position.z = home.pose.position.z;
}
/*
    * function to start activity

*/
void api::march(){
    local_pos_pub.publish(setpoint_position);
    ros::spinOnce();
    rate.sleep();
}

void api::march_NED(){
    set_point_raw_pub.publish(set_point_raw);
    ros::spinOnce();
    rate.sleep();
}


void api::refresh_set_point(){
    // setpoint_position = current_position;
    setpoint_position.pose.position.x = current_position.pose.position.x;
    setpoint_position.pose.position.y = current_position.pose.position.y;
    setpoint_position.pose.position.z = current_position.pose.position.z;
    // setpoint_position.pose.orientation.x = 0;
    // setpoint_position.pose.orientation.y = 0;
    // setpoint_position.pose.orientation.z = 0;
    // setpoint_position.pose.orientation.w = -1;
}
void api::refresh_set_point_NED(){
    set_point_raw.position.x = current_position.pose.position.x;
    set_point_raw.position.y = current_position.pose.position.y;
    set_point_raw.position.z = current_position.pose.position.z;
    // setpoint_position = current_position;
    // set_point_raw.pose.position.x = current_position.pose.position.x;
    // set_point_raw.pose.position.y = current_position.pose.position.y;
    // set_point_raw.pose.position.z = current_position.pose.position.z;
    // set_point_raw.pose.orientation.x = 0;
    // set_point_raw.pose.orientation.y = 0;
    // set_point_raw.pose.orientation.z = 0;
    // set_point_raw.pose.orientation.w = -1;
}


void api::set_point(float x, float y , float z){
    setpoint_position.pose.position.x = current_position.pose.position.x + x;
    setpoint_position.pose.position.y = current_position.pose.position.y + y;
    setpoint_position.pose.position.z = current_position.pose.position.z + z;
    #ifdef DEBUG
    ROS_INFO("setpoint_position position x: %lf",setpoint_position.pose.position.x);
    ROS_INFO("setpoint_position position y: %lf",setpoint_position.pose.position.y);
    ROS_INFO("setpoint_position position z: %lf\n",setpoint_position.pose.position.z);
    ROS_INFO("current_position position x: %lf",current_position.pose.position.x);
    ROS_INFO("current_position position y: %lf",current_position.pose.position.y);
    ROS_INFO("current_position position z: %lf\n",current_position.pose.position.z);
    #endif
}
void api::set_point_2(float x, float y , float z){
    setpoint_position.pose.position.x += x;
    setpoint_position.pose.position.y += y;
    setpoint_position.pose.position.z += z;
    #ifdef DEBUG
    ROS_INFO("setpoint_position position x: %lf",setpoint_position.pose.position.x);
    ROS_INFO("setpoint_position position y: %lf",setpoint_position.pose.position.y);
    ROS_INFO("setpoint_position position z: %lf\n",setpoint_position.pose.position.z);
    ROS_INFO("current_position position x: %lf",current_position.pose.position.x);
    ROS_INFO("current_position position y: %lf",current_position.pose.position.y);
    ROS_INFO("current_position position z: %lf\n",current_position.pose.position.z);
    #endif
}
void api::set_point_NED (float x, float y , float z){
    set_point_raw.position.x += x;
    set_point_raw.position.y += y;
    set_point_raw.position.z -= z;
    #ifdef DEBUG

    #endif
}

void api::set_point(float x, float y){
    setpoint_position.pose.position.x = current_position.pose.position.x + x;
    setpoint_position.pose.position.y = current_position.pose.position.y + y;

}
void api::set_point_2(float x, float y){
    setpoint_position.pose.position.x += x;
    setpoint_position.pose.position.y += y;

}

void api::set_point_NED(float x, float y){
    set_point_raw.position.x += x;
    set_point_raw.position.y += y;
}


void api::set_home(){
    home.pose.position.x = current_position.pose.position.x;
    home.pose.position.y = current_position.pose.position.y;
    home.pose.position.z = current_position.pose.position.z;
    home.pose.orientation.x = 0;//current_position.pose.orientation.x;
    home.pose.orientation.y = 0;//current_position.pose.orientation.y;
    home.pose.orientation.z = 0;//current_position.pose.orientation.z;
    home.pose.orientation.w = -1;//current_position.pose.orientation.w;
    // home = current_position;
    #ifdef DEBUG
    ROS_INFO("home position x: %lf",home.pose.position.x);
    ROS_INFO("home position y: %lf",home.pose.position.y);
    ROS_INFO("home position z: %lf\n",home.pose.position.z);
    ROS_INFO("current_position position x: %lf",current_position.pose.position.x);
    ROS_INFO("current_position position y: %lf",current_position.pose.position.y);
    ROS_INFO("current_position position z: %lf\n",current_position.pose.position.z);
    #endif
    
}
/*
    * function to check if destination reached
*/
bool api::reached_point(){
    float dx = setpoint_position.pose.position.x - current_position.pose.position.x ;
    float dy = setpoint_position.pose.position.y - current_position.pose.position.y ;
    float dz = setpoint_position.pose.position.z - current_position.pose.position.z ;
    return  sqrt (dx * dx + dy * dy + dz * dz)  < dest_threshold;
}
bool api::reached_point_NED(){
    float dx = set_point_raw.position.x - current_position.pose.position.x ;
    float dy = set_point_raw.position.y - current_position.pose.position.y ;
    float dz = set_point_raw.position.z - current_position.pose.position.z ;
    return  sqrt (dx * dx + dy * dy + dz * dz)  < dest_threshold;
}

void api::set_velocity(float x, float y, float z){
    set_vel.linear.x = x;
    set_vel.linear.y = y;
    set_vel.linear.z = z;
    set_vel_pub.publish(set_vel);
}

void api::set_acceleration(float x, float y, float z){
    set_accel.vector.x = x;
    set_accel.vector.y = y;
    set_accel.vector.z = z;
    set_accel_pub.publish(set_accel);
}

void api::set_timer(double delta){
    dt = ros::Duration(delta);
    timer_start = ros::Time::now();
}


bool api::check_timer(){
    return  ros::Time::now() - timer_start > dt;
}


void api::land(){
    land_client.call(land_cmd);

}

void api::set_attitude(float yaw){
    set_point_raw.yaw = yaw;
}