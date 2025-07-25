#include <chrono>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/PositionTarget.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <mavros_msgs/AttitudeTarget.h>
// #include <mavros_msgs/WaypointPush.h>

// added libs
#include "geometry_msgs/Quaternion.h"
#include "tf2/transform_datatypes.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "file_write.hpp"

#include <string>

#define DEBUG
#define LOGING

static void state_cb(const mavros_msgs::State::ConstPtr& msg); //callback function for current state
static void get_pos(const geometry_msgs::PoseStamped::ConstPtr& msg); // callback function for current positon

static void get_px4_pos(const geometry_msgs::PoseStamped::ConstPtr& msg); // local position

static mavros_msgs::State current_state;
static geometry_msgs::PoseStamped current_position; // MOCAP position
static geometry_msgs::PoseStamped px4_position; // px4 feedback position

File_write logger;

bool firstDataFlag = false;

class api
{
private:

    ros::NodeHandle nh; //create node handling

    geometry_msgs::PoseStamped position; //current position?
    
    geometry_msgs::PoseStamped setpoint_position;

    geometry_msgs::PoseStamped home; // home position

    mavros_msgs::SetMode offb_set_mode; // setting mode

    mavros_msgs::PositionTarget set_point_raw; // handling positions

    mavros_msgs::CommandTOL land_cmd; // variable for landing

    /* data */
    
    ros::Rate rate = ros::Rate(110.0); // update frequency
    
    mavros_msgs::CommandBool arm_cmd; //variable for arming

    // ros topics and services
    ros::Subscriber state_sub; // subcribing to current state
    ros::Subscriber pos; // subcribing to current position 
    ros::Subscriber px4_pos; //subscribing to px4 feedback position
    ros::Publisher local_pos_pub; // publising local position?
    ros::Publisher set_point_raw_pub; // publising set point
    ros::ServiceClient arming_client; //service for arming
    ros::ServiceClient set_mode_client; // service for setting mode
    ros::ServiceClient land_client; // servicce for landing

    // timer
    ros::Time timer_start;
    ros::Duration dt;

    float dest_threshold; // precision for position based navigation

    /// experimental features
    double roll, pitch, yaw;

    
    /// experimental finish
    
    
public: 
    



    api(int argc, char **argv);
    ~api();
    bool arm(); // function to arm
    bool disarm(); //function to disarm
    bool set_mode(std::string mode); // function to set mode

    // void take_off(float altitude);
    // void take_off_2(float altitude); // take of in ENU

    
    // void landing(); //updates altitude for landing

    // void set_point(float x, float y , float z); // ENU system
    // void set_point(float x, float y); // for horizontal flight

    // void set_point_2(float x, float y , float z);
    // void set_point_2(float x, float y); // for horizontal flight

    void set_point_NED(float x, float y , float z); //data sent in NED 
    void set_point_NED(float x, float y);

    void set_home(); // sets home position
    // void refresh_set_point(); // refreshes set point to current location
    void refresh_set_point_NED();
    // void reset(); // reset???
    // void march();  /// applying all changes anf flying

    void march_NED();  /// applying all changes anf flying

    bool reached_point(); // function for checking if point reached 
    
    void set_timer(double delta);

    bool check_timer();

    
    void land(); // cmd to start landing
    void take_off_NED(float altitude);
    bool reached_point_NED();
    /// experimnetal features
    void get_position(); // function to get current position into variable "position"
    void set_heading_offset(float yaw); // function to set yaw as offset to current
    void set_heading_global(float yaw); // function to set yaw
    void init_heading(); // set point heading as current heading

    void set_point_NED_global(float x, float y);
    void set_point_NED_global(float x, float y, float z);

    void set_global_point(float x, float y);
    void set_global_point(float x, float y, float z);

    void get_position_ret(float &x,float &y,float &z,float &yaw ); // function for sending position and orientation to the user
    /// experimental finish

    void get_px4_yaw(double &yaw );
    bool reached_point_px4();
    void set_home_px4();
};


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
    pos = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose",1,get_pos);// subsribe to topi with proper coordinate system

    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);

    arming_client = nh.serviceClient<mavros_msgs::CommandBool> //client to arm
            ("mavros/cmd/arming");

    set_mode_client = nh.serviceClient<mavros_msgs::SetMode> //client to set mode
            ("mavros/set_mode");

    set_point_raw_pub = nh.advertise<mavros_msgs::PositionTarget> // publisher for set point
            ("mavros/setpoint_raw/local", 10);

    px4_pos = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose",1,get_px4_pos);


    land_cmd.request.yaw = 0.0;
    land_cmd.request.latitude = 0;
    land_cmd.request.longitude = 0;
    land_cmd.request.altitude = 0;
    land_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");
    /// experimnental finish
    dest_threshold = .1;

    set_point_raw.coordinate_frame = 1;
    set_point_raw.type_mask = 2048;
    set_point_raw.yaw = 1.5708; // initial yaw
    set_point_raw.yaw_rate = 0;
    // set_point_raw.header.frame_id = "base_link";//"base_link";
    ros::spinOnce();
    rate.sleep();

    logger.create_table_names(current_position);
    logger.create_table_names(current_state);
    logger.create_table_names(set_point_raw);
}

api::~api()
{
}

/*
    * call back function to get state of the vehicle
*/
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
    #ifdef LOGING
    logger.save_data(current_state);
    #endif
    // ROS_INFO("%f\n",current_state);
}
/*
    * call back function to get position of the drone
*/
void get_pos(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_position = *msg;
    firstDataFlag = true;
    //ROS_INFO("Position updated");
    #ifdef LOGING
    logger.save_data(current_position);
    #endif
    // ROS_INFO_STREAM("current_position: " << current_position);
}

void get_px4_pos(const geometry_msgs::PoseStamped::ConstPtr& msg){
    px4_position = *msg;
}


/*
    * function to arm the drone
    * @return - true/false if arming was sucessful
*/
bool api::arm(){
    arm_cmd.request.value = true;
    if( arming_client.call(arm_cmd) && arm_cmd.response.success){
        #ifdef DEBUG
        ROS_INFO("Vehicle armed");
        #endif

        #ifdef LOGING
        logger.save_data(current_state);
        #endif

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
        #ifdef DEBUG
        ROS_INFO("Vehicle disarmed");
        #endif

        #ifdef LOGING
        logger.save_data(current_state);
        #endif

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
    available for px4
    *
    string MODE_PX4_MANUAL=MANUAL
    string MODE_PX4_ACRO=ACRO
    string MODE_PX4_ALTITUDE=ALTCTL
    string MODE_PX4_POSITION=POSCTL
    string MODE_PX4_OFFBOARD=OFFBOARD
    string MODE_PX4_STABILIZED=STABILIZED
    string MODE_PX4_RATTITUDE=RATTITUDE
    string MODE_PX4_MISSION=AUTO.MISSION
    string MODE_PX4_LOITER=AUTO.LOITER
    string MODE_PX4_RTL=AUTO.RTL
    string MODE_PX4_LAND=AUTO.LAND
    string MODE_PX4_RTGS=AUTO.RTGS
    string MODE_PX4_READY=AUTO.READY
    string MODE_PX4_TAKEOFF=AUTO.TAKEOFF
    *
*/
bool api::set_mode(std::string mode){
    offb_set_mode.request.custom_mode = "OFFBOARD";
    if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){

                #ifdef DEBUG
                ROS_INFO("Offboard enabled");
                #endif

                #ifdef LOGING
                logger.save_data(current_state);
                #endif

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

void api::take_off_NED(float altitude){
    set_point_raw.position.z -= altitude;
    #ifdef DEBUG
    ROS_INFO_STREAM("altitude: " << altitude);
    ROS_INFO_STREAM("target altitude: " << set_point_raw.position.z);
    #endif

    #ifdef LOGING
    logger.save_data(current_state);
    #endif
}



void api::march_NED(){
    set_point_raw.header.stamp = ros::Time::now();
    set_point_raw_pub.publish(set_point_raw);
    #ifdef LOGING
    logger.save_data(set_point_raw);
    #endif
    
    ros::spinOnce();
    rate.sleep();
}



void api::refresh_set_point_NED(){
    set_point_raw.position.x = current_position.pose.position.x;
    set_point_raw.position.y = current_position.pose.position.y;
    set_point_raw.position.z = current_position.pose.position.z;
    #ifdef LOGING
    logger.save_data(set_point_raw);
    logger.save_data(current_position);
    #endif
    #ifdef DEBUG
    ROS_INFO_STREAM("current_position: " << current_position);
    #endif

}



void api::set_point_NED (float x, float y , float z){
    set_point_raw.position.x += x;
    set_point_raw.position.y -= y;
    set_point_raw.position.z -= z;
    #ifdef LOGING
    logger.save_data(set_point_raw);
    #endif
}



void api::set_point_NED(float x, float y){
    set_point_raw.position.x += x;
    set_point_raw.position.y -= y;
    #ifdef LOGING
    logger.save_data(set_point_raw);
    #endif
}

void api::set_global_point(float x, float y){
    set_point_raw.position.x = x;
    set_point_raw.position.y = -y;
    #ifdef LOGING
    logger.save_data(set_point_raw);
    #endif
}
void api::set_global_point(float x, float y, float z){
    set_point_raw.position.x = x;
    set_point_raw.position.y = -y;
    set_point_raw.position.z = -z;
    #ifdef LOGING
    logger.save_data(set_point_raw);
    #endif
}

void api::set_point_NED_global(float x, float y, float z){
    set_point_raw.position.x = home.pose.position.x + x;
    set_point_raw.position.y = home.pose.position.y - y;
    set_point_raw.position.z = home.pose.position.z - z;
    #ifdef LOGING
    logger.save_data(set_point_raw);
    #endif
    
}
void api::set_point_NED_global(float x, float y){
    set_point_raw.position.x = home.pose.position.x + x;
    set_point_raw.position.y = home.pose.position.y - y;
    #ifdef LOGING
    logger.save_data(set_point_raw);
    #endif
}


/*
    * function to set home using MOCAP data
*/
void api::set_home(){
    home.pose.position.x = current_position.pose.position.x;
    home.pose.position.y = current_position.pose.position.y;
    home.pose.position.z = current_position.pose.position.z;
    home.pose.orientation.x = current_position.pose.orientation.x;
    home.pose.orientation.y = current_position.pose.orientation.y;
    home.pose.orientation.z = current_position.pose.orientation.z;
    home.pose.orientation.w = current_position.pose.orientation.w;
    // home = current_position;
    #ifdef LOGING
    logger.save_data(current_position);
    #endif

    #ifdef DEBUG
    ROS_INFO_STREAM("home position: " << home);
    ROS_INFO_STREAM("current position: " << current_position);
    // ROS_INFO("home position x: %lf",home.pose.position.x);
    // ROS_INFO("home position y: %lf",home.pose.position.y);
    // ROS_INFO("home position z: %lf\n",home.pose.position.z);
    // ROS_INFO("current_position position x: %lf",current_position.pose.position.x);
    // ROS_INFO("current_position position y: %lf",current_position.pose.position.y);
    // ROS_INFO("current_position position z: %lf\n",current_position.pose.position.z);
    #endif
    
}

/*
    * function to set home using px4 data
*/
void api::set_home_px4(){
    home.pose.position.x = px4_position.pose.position.x;
    home.pose.position.y = px4_position.pose.position.y;
    home.pose.position.z = px4_position.pose.position.z;
    home.pose.orientation.x = px4_position.pose.orientation.x;
    home.pose.orientation.y = px4_position.pose.orientation.y;
    home.pose.orientation.z = px4_position.pose.orientation.z;
    home.pose.orientation.w = px4_position.pose.orientation.w;
    // home = current_position;
    
    #ifdef DEBUG
    logger.save_data(px4_position);
    ROS_INFO_STREAM("home position: " << home);
    ROS_INFO_STREAM("current position: " << px4_position);
    // ROS_INFO("home position x: %lf",home.pose.position.x);
    // ROS_INFO("home position y: %lf",home.pose.position.y);
    // ROS_INFO("home position z: %lf\n",home.pose.position.z);
    // ROS_INFO("current_position position x: %lf",current_position.pose.position.x);
    // ROS_INFO("current_position position y: %lf",current_position.pose.position.y);
    // ROS_INFO("current_position position z: %lf\n",current_position.pose.position.z);
    #endif
    
}


/*
    * function to check if destination reached
    * uses data from MOCAP
*/
bool api::reached_point_NED(){
    float dx = set_point_raw.position.x - current_position.pose.position.x ;
    float dy = set_point_raw.position.y - current_position.pose.position.y ;
    float dz = set_point_raw.position.z - current_position.pose.position.z ;
    #ifdef DEBUG
    // ROS_INFO_STREAM("set point: " << set_point_raw);
    // ROS_INFO_STREAM("current position: " << current_position);
    #endif
    // logger.save_data(&current_position);
    return  sqrt (dx * dx + dy * dy + dz * dz)  < dest_threshold;
}

/*
    * function to check if destination reached
    * takes into consideration data received from px4
*/
bool api::reached_point_px4(){
    float dx = set_point_raw.position.x - px4_position.pose.position.x ;
    float dy = set_point_raw.position.y - px4_position.pose.position.y ;
    float dz = set_point_raw.position.z - px4_position.pose.position.z ;
    #ifdef DEBUG
    ROS_INFO_STREAM("set point: " << set_point_raw);
    ROS_INFO_STREAM("current position: " << px4_position);
    #endif
    // logger.save_data(&current_position);
    return  sqrt (dx * dx + dy * dy + dz * dz)  < dest_threshold;
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
    logger.save_data(current_state);
    #ifdef DEBUG
    ROS_INFO_STREAM("land cmd invoked: ");
    #endif
}

void api::set_heading_offset(float yaw){
    set_point_raw.yaw += yaw;
}

void api::set_heading_global(float yaw){
    set_point_raw.yaw = yaw;
}

void api::init_heading(){
    set_point_raw.yaw = yaw;
}

/*
* helper function to get current position
*/
void api::get_position(){
    position = current_position;
    // tf2::Matrix3x3().getRPY(roll, pitch, yaw);
    tf2::Quaternion q;
    tf2::fromMsg(current_position.pose.orientation,q);
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw); //extract roll pitch yaw from current position
    #ifdef DEBUG
    ROS_INFO_STREAM("extracted yaw: " << yaw * 180 / 3.14);
    #endif
}

/*
* helper function to extract current position
* receives pointer where to extract data
*/
void api::get_position_ret(float &x,float &y,float &z,float &yaw ){
    position = current_position;
    // tf2::Matrix3x3().getRPY(roll, pitch, yaw);
    static tf2::Quaternion q;
    tf2::fromMsg(current_position.pose.orientation,q);
    tf2::Matrix3x3(q).getRPY(this->roll, this->pitch, this->yaw); //extract roll pitch yaw from current position
    x = position.pose.position.x;
    y = position.pose.position.y;
    z = position.pose.position.z;
    yaw = this->yaw;
    #ifdef DEBUG
    ROS_INFO_STREAM("extracted yaw: " << yaw * 180 / 3.14);
    #endif
}

/*
* helper function to extract px4 orientation
* receives pointer where to extract data
*/
void api::get_px4_yaw(double &my_yaw ){
    static tf2::Quaternion q;
    static double roll, pitch;
    tf2::fromMsg(px4_position.pose.orientation,q);
    tf2::Matrix3x3(q).getRPY(roll, pitch, my_yaw);
    #ifdef DEBUG
    ROS_INFO_STREAM("px4 extracted yaw: " << my_yaw * 180 / 3.14);
    #endif
}


// void api::take_off(float altitude){
//     setpoint_position.pose.position.z = current_position.pose.position.z + altitude;
// }
/*
    * function to take off
    * @param - altitude to go to

*/
// void api::take_off_2(float altitude){
//     setpoint_position.pose.position.z += altitude;
// }

/*
    * function to land
    * not yet works turning off motors

*/
// void api::landing(){
//     setpoint_position.pose.position.z = home.pose.position.z;
// }
/*
    * function to start activity

*/
// void api::march(){
//     local_pos_pub.publish(setpoint_position);
//     ros::spinOnce();
//     rate.sleep();
// }


// void api::refresh_set_point(){
//     // setpoint_position = current_position;
//     setpoint_position.pose.position.x = current_position.pose.position.x;
//     setpoint_position.pose.position.y = current_position.pose.position.y;
//     setpoint_position.pose.position.z = current_position.pose.position.z;
// }


// void api::set_point(float x, float y , float z){
//     setpoint_position.pose.position.x = current_position.pose.position.x + x;
//     setpoint_position.pose.position.y = current_position.pose.position.y + y;
//     setpoint_position.pose.position.z = current_position.pose.position.z + z;
//     #ifdef DEBUG

//     #endif
// }
// void api::set_point_2(float x, float y , float z){
//     setpoint_position.pose.position.x += x;
//     setpoint_position.pose.position.y += y;
//     setpoint_position.pose.position.z += z;
//     #ifdef DEBUG

//     #endif
// }

// void api::set_point(float x, float y){
//     setpoint_position.pose.position.x = current_position.pose.position.x + x;
//     setpoint_position.pose.position.y = current_position.pose.position.y + y;

// }
// void api::set_point_2(float x, float y){
//     setpoint_position.pose.position.x += x;
//     setpoint_position.pose.position.y += y;

// }

// bool api::reached_point(){
//     float dx = setpoint_position.pose.position.x - current_position.pose.position.x ;
//     float dy = setpoint_position.pose.position.y - current_position.pose.position.y ;
//     float dz = setpoint_position.pose.position.z - current_position.pose.position.z ;
//     return  sqrt (dx * dx + dy * dy + dz * dz)  < dest_threshold;
// }
