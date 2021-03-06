#include <chrono>

// #include <ros/ros.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <mavros_msgs/CommandBool.h>
// #include <mavros_msgs/SetMode.h>
// #include <mavros_msgs/State.h>
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

geometry_msgs::PoseStamped position;

void get_pos(const geometry_msgs::PoseStamped::ConstPtr& msg){
    position = *msg;
}

int main(int argc, char **argv)
{
    ros::Time start;
    bool flag = true;
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
    
    ros::Subscriber pos = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose",10,get_pos);

    //ros::ServiceClient takeoff_cmd = nh.serviceClient<mavros_msgs::CommandTOL>
    //        ("mavros/cmd/takeoff");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ROS_INFO("Connecting");
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Connected");
    

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = position.pose.position.x;
    pose.pose.position.y = position.pose.position.y;
    pose.pose.position.z = position.pose.position.z + 1;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    mavros_msgs::CommandBool arm_cmd2;
    arm_cmd2.request.value = false;
    ros::Time last_request = ros::Time::now();
    
    // arming_client.call(arm_cmd); 
    // ROS_INFO("ARMED");

    // set_mode_client.call(offb_set_mode);
    // ROS_INFO("OFFBOARD");
    // ///
    // start = clock();
    // ///
    // ROS_INFO("TAKEOFF");
    arming_client.call(arm_cmd);
    while(ros::ok()){
       if( current_state.mode != "OFFBOARD" &&
           (ros::Time::now() - last_request > ros::Duration(5.0))){
           if( set_mode_client.call(offb_set_mode) &&
               offb_set_mode.response.mode_sent){
               ROS_INFO("Offboard enabled");
    //             pose.pose.position.x = position.pose.position.x;
    //             pose.pose.position.y = position.pose.position.y;
    //             pose.pose.position.z = 1;

    // //send a few setpoints before starting
    //             for(int i = 100; ros::ok() && i > 0; --i){
    //                 local_pos_pub.publish(pose);
    //                 ros::spinOnce();
    //                 rate.sleep();
    //             }
           }
           last_request = ros::Time::now();
           continue;
       } else {
           if( !current_state.armed &&
               (ros::Time::now() - last_request > ros::Duration(5.0))){
               if( arming_client.call(arm_cmd) &&
                   arm_cmd.response.success){
                   ROS_INFO("Vehicle armed");
    //                pose.pose.position.x = position.pose.position.x;
    //                 pose.pose.position.y = position.pose.position.y;
    //                 pose.pose.position.z = 1;

    // //send a few setpoints before starting
    //                 for(int i = 100; ros::ok() && i > 0; --i){
    //                     local_pos_pub.publish(pose);
    //                     ros::spinOnce();
    //                     rate.sleep();
    //                 }
               }
               last_request = ros::Time::now();
               continue;
           }
           if( current_state.armed){
                ROS_INFO("Starting Flight test");
                break;
           }
       }
       ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("TAKEOFF");
    start = ros::Time::now();
    flag = true;
    while(ros::ok()){
    //arming_client.call(arm_cmd);
        local_pos_pub.publish(pose);

        
        if ((ros::Time::now() - start) > ros::Duration(10.0)){
            // pose.pose.position.x = position.pose.position.x;
            // pose.pose.position.y = position.pose.position.y;
            pose.pose.position.z = pose.pose.position.z - 0.5;
            flag = false;
            ROS_INFO("Descending");
        }
        ros::spinOnce();
        rate.sleep();
        if (!flag){
            break;
        }
    }
    flag = true;
    start = ros::Time::now();
    while(ros::ok()){
    //arming_client.call(arm_cmd);
        local_pos_pub.publish(pose);

        
        if (ros::Time::now() - start > ros::Duration(5.0)){
            // pose.pose.position.x = position.pose.position.x;
            // pose.pose.position.y = position.pose.position.y;
            pose.pose.position.z = pose.pose.position.z - 1.5;
            // flag = false;
            flag = false;
            ROS_INFO("LAND");
        }
        ros::spinOnce();
        rate.sleep();
        if (!flag){
            break;
        }
    }

    flag = true;
    start = ros::Time::now();
    while(ros::ok()){
    //arming_client.call(arm_cmd);
        local_pos_pub.publish(pose);

        
        if (ros::Time::now() - start > ros::Duration(5.0)){
            // pose.pose.position.x = position.pose.position.x;
            // pose.pose.position.y = position.pose.position.y;
            arming_client.call(arm_cmd2);
        }
        ros::spinOnce();
        rate.sleep();
        if (!flag){
            break;
        }
    }


    arming_client.call(arm_cmd2);
    
    return 0;
}