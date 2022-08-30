#include <chrono>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>


#include "api.hpp"
#include <string>

#include "traj_min_snap.hpp"
#include <Eigen/Dense>
#include "file_read.hpp"

using namespace std;
using namespace ros;
using namespace Eigen;
///
/*
*/
///

// function for time allocation
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
//


int main(int argc, char **argv){

    ros::init(argc, argv, "offb_node"); // needed to init ros node

    /*
    part for minimum snap
    */
    min_snap::SnapOpt snapOpt;
    min_snap::Trajectory minSnapTraj;

    MatrixXd route;
    VectorXd ts;
    Matrix3d iS, fS;
    Eigen::Matrix<double, 3, 4> iSS, fSS;
    iS.setZero();
    fS.setZero();

    File_read test("/home/ros/test/src/large_scale_traj_optimizer-main/example1/src/trajectory.csv");
    route = test.read_all_data();
    int N = (int) (route.cols()) -1;

    iS.col(0) << route.leftCols<1>();
    fS.col(0) << route.rightCols<1>();
    ts = allocateTime(route, 3.0, 2.0); // current time allocation is 3 m/s and 3 m/s^2

    iSS << iS, Eigen::MatrixXd::Zero(3, 1);
    fSS << fS, Eigen::MatrixXd::Zero(3, 1);


    snapOpt.reset(iSS, fSS, route.cols() - 1);
    snapOpt.generate(route.block(0, 1, 3, N - 1), ts);
    snapOpt.getTraj(minSnapTraj);
    /*
    part for minimum snap
    */


    api my_drone = api(argc, argv); //initialize class to work with drone
    float x,y,z,yaw;
    Eigen::Vector3d position;

    while (!firstDataFlag){
        ros::spinOnce();
    }

    
    
    my_drone.refresh_set_point_NED(); // init set_point to current state
    
    my_drone.get_position();
    
    my_drone.get_position_ret(x,y,z,yaw);
    my_drone.set_heading_global(yaw);
    // my_drone.take_off_2(1);
    for (int i = 0; i < 100; i++) // send a few initial set points before continueing
    {
        my_drone.march_NED(); ///send cmds
    }
    my_drone.arm(); // try to arm
    my_drone.set_mode(std::string("OFFBOARD")); //try to transition into offboard mode
    
    ROS_INFO("Take off\n");
    my_drone.take_off_NED(-1); // go .5 m up
    my_drone.set_heading_offset(0);
    // my_drone.set_timer(5.0);
    // while(ros::ok() && ! my_drone.check_timer()){ //while loop for main program
    //     my_drone.march_NED();//spin code (publish set points)
    // }
    while(ros::ok() && !my_drone.reached_point_NED()){ //while loop for main program
        my_drone.march_NED();//spin code (publish set points)
    }

    ROS_INFO("Take off completed\n");
    my_drone.set_home();

    // trjectory 1
    for (double i = 0; i < minSnapTraj.getTotalDuration(); i+= 0.08){
        position =  minSnapTraj.getPos(i);
        my_drone.set_point_NED_global(position[0],position[1],position[2]);
        my_drone.set_heading_offset(0);
        my_drone.set_timer(0.05);
        while(ros::ok() && ! my_drone.check_timer()){ //while loop for main program
            my_drone.march_NED();//spin code (publish set points)
        }
    }
    my_drone.set_timer(7.0);
    while ( ! my_drone.check_timer() && ros::ok()){my_drone.march_NED();} // finishing first trajectory


    /*
    trajectory 2
    begin
    */
    iS.setZero();
    fS.setZero();

    File_read test("/home/ros/test/src/large_scale_traj_optimizer-main/example1/src/trajectory.csv");
    route = test.read_all_data();
    N = (int) (route.cols()) -1;
    iS.col(0) << route.leftCols<1>();
    fS.col(0) << route.rightCols<1>();
    ts = allocateTime(route, 3.0, 2.0); // current time allocation is 3 m/s and 3 m/s^2
    iSS << iS, Eigen::MatrixXd::Zero(3, 1);
    fSS << fS, Eigen::MatrixXd::Zero(3, 1);
    snapOpt.reset(iSS, fSS, route.cols() - 1);
    snapOpt.generate(route.block(0, 1, 3, N - 1), ts);
    snapOpt.getTraj(minSnapTraj);
    my_drone.set_home(); // flying
    // trjectory 1
    for (double i = 0; i < minSnapTraj.getTotalDuration(); i+= 0.08){
        position =  minSnapTraj.getPos(i);
        my_drone.set_point_NED_global(position[0],position[1],position[2]);
        my_drone.set_heading_offset(0);
        my_drone.set_timer(0.05);
        while(ros::ok() && ! my_drone.check_timer()){ //while loop for main program
            my_drone.march_NED();//spin code (publish set points)
        }
    }
    my_drone.set_timer(7.0);
    while ( ! my_drone.check_timer() && ros::ok()){my_drone.march_NED();} // finishing first trajectory
    /*
    trajectory 2 
    end
    */



    /*
    trajectory 3
    begin
    */
    iS.setZero();
    fS.setZero();

    File_read test("/home/ros/test/src/large_scale_traj_optimizer-main/example1/src/trajectory.csv");
    route = test.read_all_data();
    N = (int) (route.cols()) -1;
    iS.col(0) << route.leftCols<1>();
    fS.col(0) << route.rightCols<1>();
    ts = allocateTime(route, 3.0, 2.0); // current time allocation is 3 m/s and 3 m/s^2
    iSS << iS, Eigen::MatrixXd::Zero(3, 1);
    fSS << fS, Eigen::MatrixXd::Zero(3, 1);
    snapOpt.reset(iSS, fSS, route.cols() - 1);
    snapOpt.generate(route.block(0, 1, 3, N - 1), ts);
    snapOpt.getTraj(minSnapTraj);
    my_drone.set_home(); // flying
    // trjectory 1
    for (double i = 0; i < minSnapTraj.getTotalDuration(); i+= 0.08){
        position =  minSnapTraj.getPos(i);
        my_drone.set_point_NED_global(position[0],position[1],position[2]);
        my_drone.set_heading_offset(0);
        my_drone.set_timer(0.05);
        while(ros::ok() && ! my_drone.check_timer()){ //while loop for main program
            my_drone.march_NED();//spin code (publish set points)
        }
    }
    my_drone.set_timer(7.0);
    while ( ! my_drone.check_timer() && ros::ok()){my_drone.march_NED();} // finishing first trajectory
    /*
    trajectory 3
    end
    */


    // Lets drone to hold and finish trajectory before landing
    my_drone.set_timer(7.0);
    while ( ! my_drone.check_timer() && ros::ok()){my_drone.march_NED();}
    my_drone.land();

    my_drone.set_timer(7.0);
    while ( ! my_drone.check_timer() && ros::ok()){}
    ROS_INFO("Landed:\n");
    my_drone.disarm();

    logger.close_files();




    return 0;
}