#include <fsm_lidom.h>

/*******************************************************************************
*/
FSMLO::FSMLO(ros::NodeHandle nh, ros::NodeHandle nh_private) :
  nh_(nh),
  nh_private_(nh_private),
  lock_(true),
  sc_(0),
  initial_pose_{0.0,0.0,0.0},
  current_pose_{0.0,0.0,0.0}
{
  ROS_INFO("[FSM_LIDOM] Starting up");

  // init params
  initParams();

  // cache fftw plans for efficiency of execution
  cacheFFTW3Plans(SIZE_SCAN);

  // The subscriber to the input scans topic
  scan_sub_ =
    nh_.subscribe(scan_topic_, 1, &FSMLO::scanCallback, this);

  // Initial pose setting service
  set_initial_pose_service_ = nh_.advertiseService(
    "fsm_lidom/set_initial_pose_service", &FSMLO::initialPoseService, this);

  // Start service
  start_service_= nh_.advertiseService(
    "fsm_lidom/start", &FSMLO::serviceStart, this);

  // Stop service
  stop_service_= nh_.advertiseService(
    "fsm_lidom/stop", &FSMLO::serviceStop, this);

  // fsm's pose estimate
  pose_estimate_pub_ =
    nh_.advertise<geometry_msgs::PoseStamped>(pose_estimate_topic_, 1);

  // fsm's path estimate
  path_estimate_pub_ =
    nh_.advertise<nav_msgs::Path>(path_estimate_topic_, 1);
}


/*******************************************************************************
*/
FSMLO::~FSMLO()
{
  printf("[FSMLO] Destroying FSMLO\n");
}


/*******************************************************************************
*/
void
FSMLO::cacheFFTW3Plans(const unsigned int& sz)
{
  // Create forward  plan
  double* r2r_in;
  double* r2r_out;

  r2r_in = (double*) fftw_malloc(sz * sizeof(double));
  r2r_out = (double*) fftw_malloc(sz * sizeof(double));

  r2rp_ = fftw_plan_r2r_1d(sz, r2r_in, r2r_out, FFTW_R2HC, FFTW_MEASURE);

  // Create backward plan
  fftw_complex* c2r_in;
  double* c2r_out;

  c2r_in = (fftw_complex*) fftw_malloc(sz * sizeof(fftw_complex));
  c2r_out = (double*) fftw_malloc(sz * sizeof(double));

  c2rp_ = fftw_plan_dft_c2r_1d(sz, c2r_in, c2r_out, FFTW_MEASURE);
}


/*******************************************************************************
*/
double
FSMLO::extractYawFromPose(const geometry_msgs::Pose& pose)
{
  tf::Quaternion q(
    pose.orientation.x,
    pose.orientation.y,
    pose.orientation.z,
    pose.orientation.w);

  tf::Matrix3x3 mat(q);
  double roll, pitch, yaw;
  mat.getRPY(roll, pitch, yaw);

  FSM::Utils::wrapAngle(&yaw);

  return yaw;
}


/*******************************************************************************
 * If there is an initial pose then set it
*/
bool FSMLO::initialPoseService(
  std_srvs::Empty::Request &req,
  std_srvs::Empty::Response &res)
{
  boost::shared_ptr<geometry_msgs::PoseWithCovarianceStamped const> pose_msg_shared;
  geometry_msgs::PoseWithCovarianceStamped pose_msg;

  // Take only one message from `initial_pose_topic_`
  pose_msg_shared =
    ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>(
      initial_pose_topic_);

  if (pose_msg_shared != NULL)
  {
    pose_msg = *pose_msg_shared;

    std::get<0>(initial_pose_) = pose_msg.pose.pose.position.x;
    std::get<1>(initial_pose_) = pose_msg.pose.pose.position.y;
    std::get<2>(initial_pose_) = extractYawFromPose(pose_msg.pose.pose);

    ROS_INFO("[FSM_LIDOM] Setting initial pose");
    return true;
  }
  else
  {
    ROS_ERROR("[FSM_LIDOM] Failed to set initial pose");
    return false;
  }
}

/*******************************************************************************
*/
  void
FSMLO::initParams()
{
  // getParam does not recognise unsigned int
  int int_param;

  // ---------------------------------------------------------------------------
  if (!nh_private_.getParam ("scan_topic", scan_topic_))
  {
    ROS_WARN("[FSM_LIDOM] no scan_topic param found; resorting to defaults");
    scan_topic_ = "/base_scan";
  }

  // ---------------------------------------------------------------------------
  if (!nh_private_.getParam ("initial_pose_topic", initial_pose_topic_))
  {
    ROS_WARN("[FSM_LIDOM] no initial_pose_topic param found; resorting to defaults");
    initial_pose_topic_ = "/fsm_lidom/initial_pose";
  }

  // ---------------------------------------------------------------------------
  if (!nh_private_.getParam ("pose_estimate_topic", pose_estimate_topic_))
  {
    ROS_WARN("[FSM_LIDOM] no pose_estimate_topic param found; resorting to defaults");
    initial_pose_topic_ = "/fsm_lidom/pose_estimate";
  }

  // ---------------------------------------------------------------------------
  if (!nh_private_.getParam ("path_estimate_topic", path_estimate_topic_))
  {
    ROS_WARN("[FSM_LIDOM] no path_estimate_topic param found; resorting to defaults");
    path_estimate_topic_ = "/fsm_lidom/path_estimate";
  }

  // ---------------------------------------------------------------------------
  if (!nh_private_.getParam ("size_scan", int_param))
  {
    ROS_WARN("[FSM_LIDOM] no size_scan param found; resorting to defaults");
    SIZE_SCAN = 360;
  }
  else
    SIZE_SCAN = static_cast<unsigned int>(int_param);

  // ---------------------------------------------------------------------------
  if (!nh_private_.getParam ("num_iterations", int_param))
  {
    ROS_WARN("[FSM_LIDOM] no num_iterations param found; resorting to defaults");
    ip_.num_iterations = 5;
  }
  else
    ip_.num_iterations = static_cast<unsigned int>(int_param);

  // ---------------------------------------------------------------------------
  if (!nh_private_.getParam ("xy_bound", ip_.xy_bound))
  {
    ROS_WARN("[FSM_LIDOM] no xy_bound param found; resorting to defaults");
    ip_.xy_bound = 0.2;
  }

  // ---------------------------------------------------------------------------
  if (!nh_private_.getParam ("t_bound", ip_.t_bound))
  {
    ROS_WARN("[FSM_LIDOM] no t_bound param found; resorting to defaults");
    ip_.t_bound = M_PI/2;
  }

  // ---------------------------------------------------------------------------
  if (!nh_private_.getParam ("max_counter", int_param))
  {
    ROS_WARN("[FSM_LIDOM] no max_counter param found; resorting to defaults");
    ip_.max_counter = 200;
  }
  else
    ip_.max_counter = static_cast<unsigned int>(int_param);

  // ---------------------------------------------------------------------------
  if (!nh_private_.getParam ("min_magnification_size", int_param))
  {
    ROS_WARN("[FSM_LIDOM] no min_magnification_size param found; resorting to defaults");
    ip_.min_magnification_size = 0;
  }
  else
    ip_.min_magnification_size = static_cast<unsigned int>(int_param);

  // ---------------------------------------------------------------------------
  if (!nh_private_.getParam ("max_magnification_size", int_param))
  {
    ROS_WARN("[FSM_LIDOM] no max_magnification_size param found; resorting to defaults");
    ip_.max_magnification_size = 3;
  }
  else
    ip_.max_magnification_size = static_cast<unsigned int>(int_param);

  // ---------------------------------------------------------------------------
  if (!nh_private_.getParam ("max_recoveries", int_param))
  {
    ROS_WARN("[FSM_LIDOM] no max_recoveries param found; resorting to defaults");
    ip_.max_recoveries = 10;
  }
  else
    ip_.max_recoveries = static_cast<unsigned int>(int_param);

  assert(SIZE_SCAN > 0);
  assert(ip_.num_iterations > 0);
  assert(ip_.xy_bound >= 0.0);
  assert(ip_.t_bound >= 0.0);
  assert(ip_.max_counter > 0);
  assert(ip_.min_magnification_size >= 0);
  assert(ip_.max_magnification_size >= ip_.min_magnification_size);
  assert(ip_.max_recoveries >= 0);
}


/*******************************************************************************
*/
  std::vector<double>
FSMLO::retypeScan(const sensor_msgs::LaserScan::Ptr& scan_msg)
{
  std::vector<double> ret_vector;

  for (auto i(0); i < scan_msg->ranges.size(); i++)
    ret_vector.push_back(scan_msg->ranges[i]);

  return ret_vector;
}


/*******************************************************************************
*/
geometry_msgs::PoseStamped
FSMLO::retypePose(const std::tuple<double,double,double>& pose)
{
  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.stamp = ros::Time::now();
  pose_msg.header.frame_id = "/map";

  // Set position
  pose_msg.pose.position.x = std::get<0>(pose);
  pose_msg.pose.position.y = std::get<1>(pose);
  pose_msg.pose.position.z = 0.0;

  // Set orientation
  tf::Quaternion q;
  q.setRPY(0.0, 0.0, std::get<2>(pose));
  q.normalize();
  tf::quaternionTFToMsg(q, pose_msg.pose.orientation);

  return pose_msg;
}


/*******************************************************************************
 *
 */
bool FSMLO::serviceStart(
  std_srvs::Empty::Request &req,
  std_srvs::Empty::Response &res)
{
  ROS_INFO("[FSM_LIDOM] Starting up...");
  lock_ = false;
}


/*******************************************************************************
 *
 */
bool FSMLO::serviceStop(
  std_srvs::Empty::Request &req,
  std_srvs::Empty::Response &res)
{
  ROS_INFO("[FSM_LIDOM] Shutting down...");
  lock_ = true;
}


/*******************************************************************************
*/
  void
FSMLO::scanCallback(const sensor_msgs::LaserScan::Ptr& scan_msg)
{
  if (lock_)
    return;

  // New scan received
  sc_++;

  // This is the first scan received; one is nothing
  if (sc_ <= 1)
  {
    // Make scan_msg into a vector<double>, remove zero ranges, and subsample
    // to SIZE_SCAN elements
    sv_ = retypeScan(scan_msg);
    sv_ = FSM::DatasetUtils::interpolateRanges(sv_);
    sv_ = FSM::Utils::subsampleScan(sv_, SIZE_SCAN);

    // Set current pose estimate
    current_pose_ = initial_pose_;
    path_estimate_.push_back(current_pose_);

    // Publish the current pose estimate
    pose_estimate_pub_.publish(retypePose(current_pose_));

    return;
  }
  else
  {
    // Lock
    lock_ = true;

    // Make scan_msg into a vector<double>, remove zero ranges, and subsample
    // to SIZE_SCAN elements
    sr_ = retypeScan(scan_msg);
    sr_ = FSM::DatasetUtils::interpolateRanges(sr_);
    sr_ = FSM::Utils::subsampleScan(sr_, SIZE_SCAN);
  }


  // The reference scan in 2d points
  std::vector< std::pair<double,double> > vp;
  FSM::Utils::scan2points(sv_, current_pose_, &vp);

  FSM::output_params op;
  std::tuple<double,double,double> result_pose;

  // ---------------------------------------------------------------------------
  // Do your magic thing
  FSM::Match::fmtdbh(sr_, current_pose_, vp, r2rp_, c2rp_, ip_, &op,
    &result_pose);
  ROS_INFO("[FSM_LIDOM] FSM executed in %f sec\n", op.exec_time);
  // ---------------------------------------------------------------------------

  // The new scan (at time t) becomes the old scan (at time t+1)
  sv_ = sr_;

  // The pose estimate at time t becomes the old estimate at  time t+1
  current_pose_ = result_pose;
  path_estimate_.push_back(current_pose_);

  // Publish the current pose estimate
  pose_estimate_pub_.publish(retypePose(current_pose_));

  // Publish the whole path estimate
  nav_msgs::Path path;
  path.header.stamp = ros::Time::now();
  path.header.frame_id = "/map";
  for (auto p(0); p < path_estimate_.size(); p++)
    path.poses.push_back(retypePose(path_estimate_[p]));

  path_estimate_pub_.publish(path);

  // Unlock
  lock_ = false;
}
