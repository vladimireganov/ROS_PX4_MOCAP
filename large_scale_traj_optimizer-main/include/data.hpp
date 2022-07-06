#include <Eigen/Eigen>

#include <string>
using namespace std;
using namespace Eigen;

class Data
{
private:
    /* data */

public:
    int iteration; // number of iteration
    double time; // time
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Vector3d acceleration;

    Data(/* args */);
    ~Data();
};

class Data_read
{
private:
    /* data */

public:
    int iteration; // number of iteration
    double time; // time
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Vector3d acceleration;

    Data(/* args */);
    ~Data();
};