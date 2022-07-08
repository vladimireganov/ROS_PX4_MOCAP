#include <Eigen/Eigen>

#include <string>
#include <iostream>
#include <fstream>
using namespace std;
using namespace Eigen;

#include  "data.hpp"



class File_write
{
private:
    string name_of_the_file;
    string log_file_name;
    ofstream data_file;
    ofstream log_file;

public:
    File_write(/* args */){
        name_of_the_file = "data.csv";
        log_file_name = "log.txt";
    }
    File_write(string name){
        name_of_the_file = "/home/ros/test/src/large_scale_traj_optimizer-main/example1/src/"+ name + ".csv";
        log_file_name = "/home/ros/test/src/large_scale_traj_optimizer-main/example1/src/" + name + "_log.txt";
    }
    // File_write(string name,Data& data );

    ~File_write();
    void connect_data(Data& data);
    void write_to_file();
    void create_table_names();
    void save_data();
    void close_files();
    Data* my_data;
};

// File_write::File_write(/* args */)
// {   
//     name_of_the_file = "data.csv";
//     log_file_name = "log.txt";

// }
// File_write::File_write(string name)
// {   
//     name_of_the_file = name + ".csv";
//     log_file_name = name + "_log.txt";
// }
// File_write::File_write(string name,Data& data )
// {   
//     name_of_the_file = name + ".csv";
//     log_file_name = name + "_log.txt";
//     my_data = &data;

// }

File_write::~File_write()
{
    //add making sure all files closed and will not be lost
}

void File_write::connect_data(Data& data){
    my_data = &data;
}

void File_write::create_table_names(){ //prints name of the columns in the csv file
    data_file.open(name_of_the_file);

    data_file << "iteration,";
    data_file << "time,";
    data_file << "position_x,";
    data_file << "position_y,";
    data_file << "position_z,";
    data_file << "velocity_x,";
    data_file << "velocity_y,";
    data_file << "velocity_z,";
    data_file << "acceleration_x,";
    data_file << "acceleration_y,";
    data_file << "acceleration_z\n";


    data_file.flush();

    // data_file.close();
}


void File_write::save_data(){ // prints data into the file
    // data_file.open(name_of_the_file);

    data_file << my_data->iteration << ",";
    data_file << my_data->time << ",";

    data_file << my_data->position[0] << ",";
    data_file << my_data->position[1] << ",";
    data_file << my_data->position[2] << ",";

    data_file << my_data->velocity[0] << ",";
    data_file << my_data->velocity[1] << ",";
    data_file << my_data->velocity[2] << ",";

    data_file << my_data->acceleration[0] << ",";
    data_file << my_data->acceleration[1] << ",";
    data_file << my_data->acceleration[2];

    data_file << "\n";

    data_file.flush();
    // std::cout << "saving";
    // data_file.close();
}

void File_write::close_files(){
    data_file.flush();
    data_file.close();
}