#include <Eigen/Eigen>

#include <string>
#include <iostream>
#include <fstream>
using namespace std;
using namespace Eigen;

#include "data.hpp"
#include <vector>
#include <sstream>

vector<vector<string>> content;
vector<string> row;

class File_read
{
private:
    string name_of_the_file;
    ifstream data_file;

public:
    File_read(string name,Data_read& data );
    ~File_read();

    MatrixXd read_all_data();
    Data_read* my_data;
};

File_read::File_read(string name,Data_read& data ){
    name_of_the_file = name;
    my_data = &data;
}

/// highly inefficient 
/// will improve later
MatrixXd File_read::read_all_data(){
    data_file.open(name_of_the_file);
    string line, word;
    vector<Eigen::Vector3d> wayPoints;
    Eigen::Vector3d point;
    if(data_file.is_open())
    {   
        while (std::getline(ss, split, delimiter))                                                                                                                                          
        {                                                                                                                                                                                          
            splits.push_back(split);
            point[0]= double(split[0]);
            point[1]= double(split[1]); 
            point[2]= double(split[2]); 
            wayPoints.push_back(point);
        }
    }
    int N = wayPoints.size();
    MatrixXd route(3, N);
    for (int i = 0; i < N; i++){
        route.col(i) << wayPoints(i);
    }
    return route;
}