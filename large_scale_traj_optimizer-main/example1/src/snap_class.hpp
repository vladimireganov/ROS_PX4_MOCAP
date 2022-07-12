#include "traj_min_jerk.hpp"
#include "traj_min_snap.hpp"
#include <Eigen/Dense>

#include <chrono>

#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <random>

#include <ros/ros.h>

#include "file_write.hpp"

#include "file_read.hpp"

using namespace std;
using namespace ros;
using namespace Eigen;

//helper function for simple time allocation
VectorXd allocateTime(const MatrixXd &wayPs,
                      double vel,
                      double acc)
{
    int N = (int)(wayPs.cols()) - 1;
    VectorXd durations(N);
    if (N > 0)
    {

        Eigen::Vector3d p0, p1;
        double dtxyz, D, acct, accd, dcct, dccd, t1, t2, t3;
        for (int k = 0; k < N; k++)
        {
            p0 = wayPs.col(k);
            p1 = wayPs.col(k + 1);
            D = (p1 - p0).norm();

            acct = vel / acc;
            accd = (acc * acct * acct / 2);
            dcct = vel / acc;
            dccd = acc * dcct * dcct / 2;

            if (D < accd + dccd)
            {
                t1 = sqrt(acc * D) / acc;
                t2 = (acc * t1) / acc;
                dtxyz = t1 + t2;
            }
            else
            {
                t1 = acct;
                t2 = (D - accd - dccd) / vel;
                t3 = dcct;
                dtxyz = t1 + t2 + t3;
            }

            durations(k) = dtxyz;
        }
    }

    return durations;
}

class snap_class
{
private:
    min_jerk::JerkOpt jerkOpt;
    min_jerk::Trajectory minJerkTraj;

    min_snap::SnapOpt snapOpt;
    min_snap::Trajectory minSnapTraj;

    MatrixXd route;
    VectorXd ts;
    Matrix3d iS, fS;
    Eigen::Matrix<double, 3, 4> iSS, fSS;
    
    Vector3d zeroVec(0.0, 0.0, 0.0);
    Rate lp(1000);
    int groupSize = 1;
    File_read test("/home/ros/test/src/large_scale_traj_optimizer-main/example1/src/trajectory.csv");

    int N;
    Data min_snap_data;
    File_write min_snap_file ("snap");

public:
    snap_class(/* args */);
    ~snap_class();
};

snap_class::snap_class(/* args */)
{
    iS.setZero();
    fS.setZero();
    route = test.read_all_data();

    N = (int) (route.cols()) -1;

    min_snap_file.connect_data(min_snap_data);


}

snap_class::~snap_class()
{
}

snap_class::reset(){

    iS.col(0) << route.leftCols<1>();
    fS.col(0) << route.rightCols<1>();
    ts = allocateTime(route, 3.0, 3.0);

    iSS << iS, Eigen::MatrixXd::Zero(3, 1);
    fSS << fS, Eigen::MatrixXd::Zero(3, 1);

    // tc0 = std::chrono::high_resolution_clock::now();
    jerkOpt.reset(iS, fS, route.cols() - 1);
    snapOpt.reset(iSS, fSS, route.cols() - 1);
}

snap_class::gen_snap(){
    snapOpt.generate(route.block(0, 1, 3, N - 1), ts);
    snapOpt.getTraj(minSnapTraj);
}

snap_class::gen_jerk(){
    jerkOpt.generate(route.block(0, 1, 3, N - 1), ts);
    jerkOpt.getTraj(minJerkTraj);
}

snap_class::save_data(double discretization){
    min_snap_file.create_table_names();
    for (double i = 0; i < minSnapTraj.getTotalDuration(); i+= discretization){ //discretization time
        min_snap_data.time = i;
        min_snap_data.iteration++;
        min_snap_data.position = minSnapTraj.getPos(i);
        min_snap_data.velocity = minSnapTraj.getVel(i);
        min_snap_data.acceleration = minSnapTraj.getAcc(i);
        min_snap_file.save_data();
    }
    min_snap_file.close_files();
}