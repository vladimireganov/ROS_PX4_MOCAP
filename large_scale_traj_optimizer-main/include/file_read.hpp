#include <Eigen/Eigen>


#include <string>
#include <iostream>
#include <fstream>
using namespace std;
using namespace Eigen;

// #include "data.hpp"
#include <vector>
#include <sstream>

vector<vector<string>> content;
vector<string> row;

class File_read
{
private:
    string name_of_the_file;
    fstream data_file;

public:
    File_read(string name);

    MatrixXd read_all_data();
    // Data_read* my_data;
};

File_read::File_read(string name){
    name_of_the_file = name;
    // my_data = &data;
}

/// highly inefficient 
/// will improve later
MatrixXd File_read::read_all_data(){
    data_file.open(name_of_the_file);
    string line, split,item;
    vector<Eigen::Array3d> wayPoints;
    Eigen::Array3d point;
    if(data_file.is_open())
    {   
        std::getline(data_file,line);
        while (std::getline(data_file,line))                                                                                                                                          
        {   
            std::istringstream line_s(line);
            // std::cout << "start of conversion";
            std::getline(line_s, item, ',');
            std::cout << item;
            point[0]= std::stod(item.c_str(),NULL);
            

            std::getline(line_s, item, ',');
            std::cout << item;
            point[1]= std::stod(item.c_str(),NULL);
            

            std::getline(line_s, item, ',');
            std::cout << item;
            point[2]= std::stod(item.c_str(),NULL); 
            

            wayPoints.push_back(point);
        }
    }
    int N = wayPoints.size();
    MatrixXd route(3, N+1);
    route.col(0).setZero();
    for (int i = 0; i < N; i++){
        route.col(i+1) << wayPoints[i];
    }
    return route;
}