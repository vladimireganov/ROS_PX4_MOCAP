#include "traj_min_jerk.hpp"
#include "traj_min_snap.hpp"

#include <chrono>

#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <random>

#include <ros/ros.h>

#include "file_write.hpp"

using namespace std;
using namespace ros;
using namespace Eigen;

class RandomRouteGenerator
{
public:
    RandomRouteGenerator(Array3d l, Array3d u)
        : lBound(l), uBound(u), uniformReal(0.0, 1.0) {}

    inline MatrixXd generate(int N)
    {
        MatrixXd route(3, N + 1);
        Array3d temp;
        route.col(0).setZero();
        for (int i = 0; i < N; i++)
        {
            temp << uniformReal(gen), uniformReal(gen), uniformReal(gen);
            temp = (uBound - lBound) * temp + lBound;
            route.col(i + 1) << temp;
        }
        return route;
    }

private:
    Array3d lBound;
    Array3d uBound;
    std::mt19937_64 gen;
    std::uniform_real_distribution<double> uniformReal;
};

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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "example1_node");
    ros::NodeHandle nh_;

    RandomRouteGenerator routeGen(Array3d(-16, -16, -16), Array3d(16, 16, 16));

    min_jerk::JerkOpt jerkOpt;
    min_jerk::Trajectory minJerkTraj;

    min_snap::SnapOpt snapOpt;
    min_snap::Trajectory minSnapTraj;

    MatrixXd route;
    VectorXd ts;
    Matrix3d iS, fS;
    Eigen::Matrix<double, 3, 4> iSS, fSS;
    iS.setZero();
    fS.setZero();
    Vector3d zeroVec(0.0, 0.0, 0.0);
    Rate lp(1000);
    int groupSize = 1;

    std::chrono::high_resolution_clock::time_point tc0, tc1, tc2;
    double d0, d1;
    int N = 10;
    // for (int i = 2; i <= 128 && ok(); i++)
    // {
        d0 = d1 = 0.0;
        // for (int j = 0; j < groupSize && ok(); j++)
        // {
            route = routeGen.generate(N);
            iS.col(0) << route.leftCols<1>();
            fS.col(0) << route.rightCols<1>();
            ts = allocateTime(route, 3.0, 3.0);

            iSS << iS, Eigen::MatrixXd::Zero(3, 1);
            fSS << fS, Eigen::MatrixXd::Zero(3, 1);

            tc0 = std::chrono::high_resolution_clock::now();
            jerkOpt.reset(iS, fS, route.cols() - 1);
            jerkOpt.generate(route.block(0, 1, 3, N - 1), ts);
            jerkOpt.getTraj(minJerkTraj);
            tc1 = std::chrono::high_resolution_clock::now();

            d0 += std::chrono::duration_cast<std::chrono::duration<double>>(tc1 - tc0).count();

            tc1 = std::chrono::high_resolution_clock::now();
            snapOpt.reset(iSS, fSS, route.cols() - 1);
            snapOpt.generate(route.block(0, 1, 3, N - 1), ts);
            snapOpt.getTraj(minSnapTraj);
            tc2 = std::chrono::high_resolution_clock::now();

            d1 += std::chrono::duration_cast<std::chrono::duration<double>>(tc2 - tc1).count();
        // }

        std::cout << "Route : \n" << route << "\n";

        // std::cout << "Route : " << route << "\n";
        std::cout << "len = " << route.size() << "\n";
        std::cout << "len in dim= " << route.size() /3 << "\n";

        std::cout << "Trajectory: \n"<<minSnapTraj.getPositions() << "\n";

        // std::cout << "Velocity: \n"<<minSnapTraj.getPositions() << "\n";

        // std::cout << "Acceleration: \n"<<minSnapTraj.getPositions() << "\n";

        std::cout << "Objective: \n"<<snapOpt.getObjective() << "\n";

        std::cout << "Durations: \n"<<minSnapTraj.getDurations() << "\n";

        std::cout << "Total Duration :" << minSnapTraj.getTotalDuration() << "\n";

        Eigen::VectorXd dur = minSnapTraj.getDurations();

        std::cout << "Piece Number: " << N
                  << " MinJerk Comp. Time: " << d0 / groupSize << " s"
                  << " MinSnap Comp. Time: " << d1 / groupSize << " s" << std::endl;

        std::cout << "Number of pieces: " << minSnapTraj.getPieceNum() <<"\n";
        std::cout << "Positions\tVelocoty\tAcceleration\n";

        double time_p = 0.0;
        for (int i = 0 ; i < N; i++ ){
            std::cout <<minSnapTraj.getPos(time_p)[0] <<"\t"<<minSnapTraj.getVel(time_p)[0] <<"\t"<<minSnapTraj.getAcc(time_p)[0] << "\n";
            std::cout <<minSnapTraj.getPos(time_p)[1] <<"\t"<<minSnapTraj.getVel(time_p)[1] <<"\t"<<minSnapTraj.getAcc(time_p)[1] << "\n";
            std::cout <<minSnapTraj.getPos(time_p)[2] <<"\t"<<minSnapTraj.getVel(time_p)[2] <<"\t"<<minSnapTraj.getAcc(time_p)[2] << "\n";
            time_p += dur[i];
            // std::cout ;
            // std::cout ;

        }


        std::cout << "getPos :\n";
        for (double i = 0; i < minSnapTraj.getTotalDuration(); i+= 0.1){
            // std::cout <<minSnapTraj.getPos(dur[i])[0] <<"\t"<<minSnapTraj.getVel(dur[i])[0] <<"\t"<<minSnapTraj.getAcc(dur[i])[0] << "\n";
            // std::cout <<minSnapTraj.getPos(dur[i])[1] <<"\t"<<minSnapTraj.getVel(dur[i])[1] <<"\t"<<minSnapTraj.getAcc(dur[i])[1] << "\n";
            std::cout << "at time: " << i << "\t" <<minSnapTraj.getPos(i) << "\n";
            // std::cout ;
            // std::cout ;
        }
        Data min_snap_data;
        Data min_jerk_data;

        File_write min_snap_file = File_write("snap",min_snap_data);
        File_write min_jerk_file = File_write("jerk",min_jerk_data);

        min_snap_file.create_table_names();
        for (double i = 0; i < minSnapTraj.getTotalDuration(); i+= 0.1){ //discretization time
            min_snap_data.time = i;
            min_snap_data.iteration++;
            min_snap_data.position = minSnapTraj.getPos(i);
            min_snap_data.velocity = minSnapTraj.getVel(i);
            min_snap_data.acceleration = minSnapTraj.getAcc(i);
            min_snap_file.save_data();
        }
        min_snap_file.close_files();



        ros::spinOnce();
        lp.sleep();
    // }

    return 0;
}
