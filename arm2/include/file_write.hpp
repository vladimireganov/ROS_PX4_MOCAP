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

#include <string>
#include <iostream>
#include <fstream>

using namespace std;

#include  "data.hpp"



class File_write
{
private:
    string name_of_the_file;
    string log_file_name;

    ofstream PoseStamped_log;
    string PoseStamped_file_name;
    ofstream PositionTarget_log;
    string PositionTarget_file_name;
    ofstream State_log;
    string State_file_name;

public:
    File_write(/* args */){
        PoseStamped_file_name = "position.csv";
        PositionTarget_log = "target_position.csv";
        PoseStamped_file_name = "state.csv";
    }

    // File_write(string name,Data& data );

    ~File_write();
    void connect_data(Data& data);

    void write_to_file();
    void create_table_names();

    void create_table_names(geometry_msgs::PoseStamped &Data);
    void create_table_names(mavros_msgs::PositionTarget &Data);
    void create_table_names(mavros_msgs::State &Data);

    void save_data();

    void save_data(geometry_msgs::PoseStamped &Data); 
    void save_data(mavros_msgs::PositionTarget &Data);
    void save_data(mavros_msgs::State &Data);

    void close_files();

    Data* my_data;
};



File_write::~File_write()
{
    //add making sure all files closed and will not be lost
}

void File_write::save_data(geometry_msgs::PoseStamped &Data){ // prints data into the file
    // data_file.open(name_of_the_file);

    PoseStamped_log << Data->header->seq << ",";
    PoseStamped_log << Data->header->frame_id << ",";
    PoseStamped_log << Data->header->stamp << ",";

    PoseStamped_log << Data->pose->position->x << ",";
    PoseStamped_log << Data->pose->position->y << ",";
    PoseStamped_log << Data->pose->position->z << ",";

    PoseStamped_log << Data->pose->orientation->x << ",";
    PoseStamped_log << Data->pose->orientation->y << ",";
    PoseStamped_log << Data->pose->orientation->z << ",";
    PoseStamped_log << Data->pose->orientation->w ;

    PoseStamped_log << "\n";

    data_file.flush();
    // std::cout << "saving";
    // data_file.close();
}

void File_write::save_data(mavros_msgs::PositionTarget &Data){ // prints data into the file
    // data_file.open(name_of_the_file);
    /*
    std_msgs/Header header
    uint8 coordinate_frame
    uint16 type_mask
    geometry_msgs/Point position
    geometry_msgs/Vector3 velocity
    geometry_msgs/Vector3 acceleration_or_force
    float32 yaw
    float32 yaw_rate
    */
    PositionTarget_log << Data->header->seq << ",";
    PositionTarget_log << Data->header->frame_id << ",";
    PositionTarget_log << Data->header->stamp << ",";

    PositionTarget_log << Data->position->x << ","; //position
    PositionTarget_log << Data->position->y << ",";
    PositionTarget_log << Data->position->z << ",";

    PositionTarget_log << Data->yaw << ",";
    PositionTarget_log << Data->yaw_rate << ",";

    PositionTarget_log << Data->velocity->x << ","; //velocity
    PositionTarget_log << Data->velocity->y << ",";
    PositionTarget_log << Data->velocity->z << ",";

    PositionTarget_log << Data->acceleration_or_force->x << ","; //acceleration
    PositionTarget_log << Data->acceleration_or_force->y << ",";
    PositionTarget_log << Data->acceleration_or_force->z << ",";

    PositionTarget_log << Data->type_mask ;

    PositionTarget_log << "\n";

    data_file.flush();
    // std::cout << "saving";
    // data_file.close();
}
void File_write::save_data(mavros_msgs::State &Data){ // prints data into the file
    // data_file.open(name_of_the_file);
    /*
    std_msgs/Header header
    bool connected
    bool armed
    bool guided
    bool manual_input
    string mode
    uint8 system_status
    */

    State_log << Data->header->seq << ",";
    State_log << Data->header->frame_id << ",";
    State_log << Data->header->stamp << ",";

    State_log << Data->connected << ",";
    State_log << Data->armed << ",";
    State_log << Data->guided << ",";
    State_log << Data->manual_input << ",";
    State_log << Data->mode << ",";
    State_log << Data->system_status;

    State_log << "\n";

    data_file.flush();
    // std::cout << "saving";
    // data_file.close();
}


void File_write::close_files(){
    data_file.flush();
    data_file.close();
}

void File_write::create_table_names(geometry_msgs::PoseStamped &Data){ //prints name of the columns in the csv file
    PoseStamped_log.open(PoseStamped_file_name);

    PoseStamped_log << "seq,"; //Header
    PoseStamped_log << "frame_id,"; //Header
    PoseStamped_log << "stamp,"; //Header

    PoseStamped_log << "position_x,";
    PoseStamped_log << "position_y,";
    PoseStamped_log << "position_z,";

    PoseStamped_log << "quaternion_x,";
    PoseStamped_log << "quaternion_y,";
    PoseStamped_log << "quaternion_z,";
    PoseStamped_log << "quaternion_w\n";

    PoseStamped_log.flush();

    // data_file.close();
}
void File_write::create_table_names(mavros_msgs::PositionTarget &Data){ //prints name of the columns in the csv file
    PositionTarget_log.open(PositionTarget_file_name);

    PositionTarget_log << "seq,"; //Header
    PositionTarget_log << "frame_id,"; //Header
    PositionTarget_log << "stamp,"; //Header

    PositionTarget_log << "position_x,";
    PositionTarget_log << "position_y,";
    PositionTarget_log << "position_z,";

    PositionTarget_log << "yaw,";
    PositionTarget_log << "yaw_rate,";

    PositionTarget_log << "velocity_x,";
    PositionTarget_log << "velocity_y,";
    PositionTarget_log << "velocity_z,";

    PositionTarget_log << "acceleration_x,";
    PositionTarget_log << "acceleration_y,";
    PositionTarget_log << "acceleration_z";

    PositionTarget_log << "type_mask\n";

    PositionTarget_log.flush();

    // data_file.close();
}
void File_write::create_table_names(mavros_msgs::State &Data){ //prints name of the columns in the csv file
    State_log.open(State_file_name);
    /*
    std_msgs/Header header
    bool connected
    bool armed
    bool guided
    bool manual_input
    string mode
    uint8 system_status
    */

    State_log << "seq,"; //Header 
    State_log << "frame_id,"; //Header 
    State_log << "stamp,"; //Header 

    State_log << "connected,";
    State_log << "armed,";
    State_log << "guided,";
    State_log << "manual_input,";
    State_log << "mode,";
    State_log << "system_status\n";


    State_log.flush();

    // data_file.close();
}