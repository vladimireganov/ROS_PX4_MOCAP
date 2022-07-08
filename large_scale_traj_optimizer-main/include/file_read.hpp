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
    vector<Eigen::Vector3d> wayPoints;
    Eigen::Vector3d point;
    if(data_file.is_open())
    {   
        while (data_file.getline(line, split))                                                                                                                                          
        {   
            std::istringstream line_s(split);

            std::getline(line_s, item, ',');
            point[0]= stod(item);
            std::cout << item;

            std::getline(line_s, item, ',');
            point[1]= stod(item);
            std::cout << item;

            std::getline(line_s, item, ',');
            point[2]= stod(item); 
            std::cout << item;

            wayPoints.push_back(point);
        }
    }
    int N = wayPoints.size();
    MatrixXd route(3, N);
    for (int i = 0; i < N; i++){
        route.col(i) << wayPoints[i];
    }
    return route;
}