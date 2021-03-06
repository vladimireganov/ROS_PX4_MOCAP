#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

//#include <mavros_msgs/CommandTOL>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
    ROS_INFO("%f\n",current_state);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    
    ros::Publisher global_pos_pub = nh.advertise<sensor_msgs::NavSatFix>
            ("mavros/global_position/global", 10);
    ros::Publisher local_pos = nh.advertise<geometry_msgs::PoseStamped>
            ("/mavros/local_position/pose", 10);
    //ros::ServiceClient takeoff_cmd = nh.serviceClient<mavros_msgs::CommandTOL>
    //        ("mavros/cmd/takeoff");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    sensor_msgs::NavSatFix pos;
    pos.latitude = 0;
    pos.longitude = 0;
    pos.altitude = 0;
    global_pos_pub.publish(pos);
    geometry_msgs::PoseStamped lpose;
    lpose.pose.position.x = 0;
    lpose.pose.position.y = 0;
    lpose.pose.position.z = 1;

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 1;

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    // set_mode_client.call(offb_set_mode);
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    // while(ros::ok() && current_state.connected){
    //     // ROS_INFO("Connecting");
    //     global_pos_pub.publish(pos);
    //     local_pos.publish(lpose);
    //     local_pos_pub.publish(pose);
    //     // arming_client.call(arm_cmd); 
    //     ros::spinOnce();
    //     rate.sleep();
    // }
    ROS_INFO("Connected");
    arming_client.call(arm_cmd); 
    ROS_INFO("Armed");
    // set_mode_client.call(offb_set_mode);
    // ROS_INFO("Offboard");
    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    // mavros_msgs::SetMode offb_set_mode;
    // offb_set_mode.request.custom_mode = "OFFBOARD";

   
    mavros_msgs::CommandBool arm_cmd2;
    arm_cmd2.request.value = false;
    ros::Time last_request = ros::Time::now();
    // set_mode_client.call(offb_set_mode);
    // ROS_INFO("Offboard");
    
    while(ros::ok()){
    //    if( current_state.mode != "OFFBOARD" &&
    //        (ros::Time::now() - last_request > ros::Duration(5.0))){
    //        if( set_mode_client.call(offb_set_mode) &&
    //            offb_set_mode.response.mode_sent){
    //            ROS_INFO("Offboard enabled");
    //        }
    //        last_request = ros::Time::now();
    //    } else {
    //        if( !current_state.armed &&
    //            (ros::Time::now() - last_request > ros::Duration(5.0))){
    //            if( arming_client.call(arm_cmd) &&
    //                arm_cmd.response.success){
    //                ROS_INFO("Vehicle armed");
    //            }
    //            last_request = ros::Time::now();
    //        }
    //    }
    //arming_client.call(arm_cmd);
        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }
    // arming_client.call(arm_cmd2);
    return 0;
}