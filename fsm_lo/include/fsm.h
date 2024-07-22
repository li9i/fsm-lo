#ifndef FSMLO_H
#define FSMLO_H

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
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
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

    // Name of this package
    std::string PKG_NAME;


    // Scan topic subscriber
    ros::Subscriber scan_sub_;

    // LO publisher
    ros::Publisher lo_pub_;

    // Pose estimate publisher
    ros::Publisher pose_estimate_pub_;

    // Path estimate publisher
    ros::Publisher path_estimate_pub_;


    // Clearing the estimated trajectory service
    ros::ServiceServer clear_trajectory_service_;

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

    // The topic where fsm's odometry is published
    std::string lo_topic_;


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

    // Previous scan receipt time
    ros::Time tv_;

    // Current scan receipt time
    ros::Time tr_;

    // Scan counter
    unsigned int sc_;

    // The origin (0,0,0)
    std::tuple<double,double,double> origin;

    // initial pose
    std::tuple<double,double,double> initial_pose_;

    // The transform
    Eigen::Matrix3d M;

    // The path estimate
    std::vector< std::tuple<double,double,double> > path_estimate_;

    // The path estimate
    nav_msgs::Path path_estimate_msg_;

    // Map frame name
    std::string global_frame_id_;

    // Base frame name
    std::string base_frame_id_;

    // Lidar odometry frame name
    std::string lo_frame_id_;

    // Transform /map -> /lo
    tf2_ros::TransformBroadcaster lo_tf_;

    // **** methods

    void cacheFFTW3Plans(
      const unsigned int& sz);

    double extractYawFromPose(
      const geometry_msgs::Pose& pose);


    void initParams();
    void initPSS();

    void publishLO(const std::tuple<double,double,double>& diff);
    void publishLOMessage(const std::tuple<double,double,double>& diff);
    void publishLOPathMessage();
    void publishLOPoseMessage();
    void publishLOTransform(const std::tuple<double,double,double>& diff);

    std::vector<double> retypeScan(
      const sensor_msgs::LaserScan::Ptr& scan_msg);

    geometry_msgs::Pose retypePose(
      const std::tuple<double,double,double>& pose);

    geometry_msgs::PoseStamped retypePoseStamped(
      const std::tuple<double,double,double>& pose,
      const std::string& frame_id);

    bool serviceClearTrajectory(
      std_srvs::Empty::Request &req,
      std_srvs::Empty::Response &res);
    bool serviceInitialPose(
      std_srvs::Empty::Request &req,
      std_srvs::Empty::Response &res);
    bool serviceStart(
      std_srvs::Empty::Request &req,
      std_srvs::Empty::Response &res);
    bool serviceStop(
      std_srvs::Empty::Request &req,
      std_srvs::Empty::Response &res);

    void scanCallback(
      const sensor_msgs::LaserScan::Ptr& scan_msg);

};

#endif // FSMLO_H
