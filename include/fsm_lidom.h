#ifndef FSMLIDOM_H
#define FSMLIDOM_H

#include <memory>
#include <chrono>
#include <signal.h>
#include <cmath>
#include <numeric>
#include <iostream>
#include <fstream>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <math.h>
#include <time.h>
#include <tuple>
#include <functional>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <std_srvs/Empty.h>
#include "fsm_core.h"


class FSMLO
{
  public:

    FSMLO(ros::NodeHandle nh, ros::NodeHandle nh_private);
    ~FSMLO();

  private:

    // Nodehandles
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;



    // Scan topic subscriber
    ros::Subscriber scan_sub_;

    // Pose estimate publisher
    ros::Publisher pose_estimate_pub_;

    // Path estimate publisher
    ros::Publisher path_estimate_pub_;

    // Initial pose setting service
    ros::ServiceServer set_initial_pose_service_;

    // Starts lidar odometry
    ros::ServiceServer start_service_;

    // Stops lidar odometry
    ros::ServiceServer stop_service_;


    // The topic where scans crash
    std::string scan_topic_;

    // The topic where the initial pose may be provided (optional)
    std::string initial_pose_topic_;

    // The topic where fsm's pose estimate is published
    std::string pose_estimate_topic_;

    // The topic where fsm's path estimate is published
    std::string path_estimate_topic_;


    // Params
    size_t SIZE_SCAN;
    FSM::input_params ip_;

    // Forward plan (DFT)
    fftw_plan r2rp_;

    // Backward plan (IDFT)
    fftw_plan c2rp_;

    // Locks callback execution
    bool lock_;

    // Previous range scan
    std::vector<double> sv_;

    // Current range scan
    std::vector<double> sr_;

    // Scan counter
    unsigned int sc_;

    // Current pose estimate
    std::tuple<double,double,double> current_pose_;

    // The initial pose (optionally provided)
    std::tuple<double,double,double> initial_pose_;


    // The path estimate
    std::vector< std::tuple<double,double,double> > path_estimate_;


    // **** methods

    void cacheFFTW3Plans(
      const unsigned int& sz);

    double extractYawFromPose(
      const geometry_msgs::Pose& pose);

    bool initialPoseService(
      std_srvs::Empty::Request &req,
      std_srvs::Empty::Response &res);

    void initParams();

    std::vector<double> retypeScan(
      const sensor_msgs::LaserScan::Ptr& scan_msg);

    geometry_msgs::PoseStamped retypePose(
      const std::tuple<double,double,double>& pose);

    bool serviceStart(
      std_srvs::Empty::Request &req,
      std_srvs::Empty::Response &res);
    bool serviceStop(
      std_srvs::Empty::Request &req,
      std_srvs::Empty::Response &res);

    void scanCallback(
      const sensor_msgs::LaserScan::Ptr& scan_msg);

};

#endif // FSMLIDOM_H
