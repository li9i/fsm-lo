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
#include <sensor_msgs/LaserScan.h>
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

    // Params
    size_t SIZE_SCAN;
    FSM::input_params ip_;

    // Forward plan (DFT)
    fftw_plan r2rp_;

    // Backward plan (IDFT)
    fftw_plan c2rp_;

    // Locks callback execution
    bool lock_;

    // Scan topic subscriber
    ros::Subscriber scan_sub_;

    // Previous range scan
    std::vector<double> sv_;

    // Current range scan
    std::vector<double> sr_;

    // Scan counter
    unsigned int sc_;

    // The topic where scans crash
    std::string scan_topic_;




    // **** methods

    void cacheFFTW3Plans(const unsigned int& sz);

    void initParams();

    std::vector<double> retypeScan(const sensor_msgs::LaserScan::Ptr& scan_msg);

    void scanCallback(const sensor_msgs::LaserScan::Ptr& scan_msg);

};

#endif // FSMLIDOM_H
