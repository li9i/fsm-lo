#include <fsm_lidom.h>

/*******************************************************************************
*/
FSMLO::FSMLO(ros::NodeHandle nh, ros::NodeHandle nh_private) :
  nh_(nh),
  nh_private_(nh_private),
  lock_(true),
  sc_(0),
  initial_pose_{0.0,0.0,0.0},
  origin{0.0,0.0,0.0},
  M(Eigen::Matrix3d::Identity()),
  path_estimate_msg_(nav_msgs::Path())
{
  ROS_INFO("[FSM_LIDOM] Init-ing...");

  // init params
  initParams();

  // cache fftw plans for efficiency of execution
  cacheFFTW3Plans(SIZE_SCAN);

  // The subscriber to the input scans topic
  scan_sub_ =
    nh_.subscribe(scan_topic_, 1, &FSMLO::scanCallback, this);

  // Clearing the estimated trajectory service
  clear_trajectory_service_ = nh_.advertiseService(
    "fsm_lidom/clear_estimated_trajectory", &FSMLO::serviceClearTrajectory, this);

  // Initial pose setting service
  set_initial_pose_service_ = nh_.advertiseService(
    "fsm_lidom/set_initial_pose", &FSMLO::serviceInitialPose, this);

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

  ROS_INFO("[%s] Init-ed.", PKG_NAME.c_str());
}


/*******************************************************************************
*/
FSMLO::~FSMLO()
{
  printf("[%s] Destroying FSMLO\n", PKG_NAME.c_str());
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
*/
  void
FSMLO::initParams()
{
  // getParam does not recognise unsigned int
  int int_param;

  // ---------------------------------------------------------------------------
  nh_private_.param<std::string>("pkg_name", PKG_NAME, "FSM_LIDOM");

  // ---------------------------------------------------------------------------
  if (!nh_private_.getParam ("scan_topic", scan_topic_))
  {
    ROS_WARN("[%s] no scan_topic param found; resorting to defaults",
      PKG_NAME.c_str());
    scan_topic_ = "/base_scan";
  }

  // ---------------------------------------------------------------------------
  if (!nh_private_.getParam ("initial_pose_topic", initial_pose_topic_))
  {
    ROS_WARN("[%s] no initial_pose_topic param found; resorting to defaults",
      PKG_NAME.c_str());
    initial_pose_topic_ = "/fsm_lidom/initial_pose";
  }

  // ---------------------------------------------------------------------------
  if (!nh_private_.getParam ("pose_estimate_topic", pose_estimate_topic_))
  {
    ROS_WARN("[%s] no pose_estimate_topic param found; resorting to defaults",
      PKG_NAME.c_str());
    initial_pose_topic_ = "/fsm_lidom/pose_estimate";
  }

  // ---------------------------------------------------------------------------
  if (!nh_private_.getParam ("path_estimate_topic", path_estimate_topic_))
  {
    ROS_WARN("[%s] no path_estimate_topic param found; resorting to defaults",
      PKG_NAME.c_str());
    path_estimate_topic_ = "/fsm_lidom/path_estimate";
  }

  // ---------------------------------------------------------------------------
  if (!nh_private_.getParam ("size_scan", int_param))
  {
    ROS_WARN("[%s] no size_scan param found; resorting to defaults",
      PKG_NAME.c_str());
    SIZE_SCAN = 360;
  }
  else
    SIZE_SCAN = static_cast<unsigned int>(int_param);

  // ---------------------------------------------------------------------------
  if (!nh_private_.getParam ("num_iterations", int_param))
  {
    ROS_WARN("[%s] no num_iterations param found; resorting to defaults",
      PKG_NAME.c_str());
    ip_.num_iterations = 2;
  }
  else
    ip_.num_iterations = static_cast<unsigned int>(int_param);

  // ---------------------------------------------------------------------------
  if (!nh_private_.getParam ("xy_bound", ip_.xy_bound))
  {
    ROS_WARN("[%s] no xy_bound param found; resorting to defaults",
      PKG_NAME.c_str());
    ip_.xy_bound = 0.2;
  }

  // ---------------------------------------------------------------------------
  if (!nh_private_.getParam ("t_bound", ip_.t_bound))
  {
    ROS_WARN("[%s] no t_bound param found; resorting to defaults",
      PKG_NAME.c_str());
    ip_.t_bound = M_PI/4;
  }

  // ---------------------------------------------------------------------------
  if (!nh_private_.getParam ("max_counter", int_param))
  {
    ROS_WARN("[%s] no max_counter param found; resorting to defaults",
      PKG_NAME.c_str());
    ip_.max_counter = 200;
  }
  else
    ip_.max_counter = static_cast<unsigned int>(int_param);

  // ---------------------------------------------------------------------------
  if (!nh_private_.getParam ("min_magnification_size", int_param))
  {
    ROS_WARN("[%s] no min_magnification_size param found; resorting to defaults",
      PKG_NAME.c_str());
    ip_.min_magnification_size = 0;
  }
  else
    ip_.min_magnification_size = static_cast<unsigned int>(int_param);

  // ---------------------------------------------------------------------------
  if (!nh_private_.getParam ("max_magnification_size", int_param))
  {
    ROS_WARN("[%s] no max_magnification_size param found; resorting to defaults",
      PKG_NAME.c_str());
    ip_.max_magnification_size = 3;
  }
  else
    ip_.max_magnification_size = static_cast<unsigned int>(int_param);

  // ---------------------------------------------------------------------------
  if (!nh_private_.getParam ("max_recoveries", int_param))
  {
    ROS_WARN("[%s] no max_recoveries param found; resorting to defaults",
      PKG_NAME.c_str());
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
void
FSMLO::publishResults(const std::tuple<double,double,double>& pose)
{
  // Construct pose estimate message and publish it
  geometry_msgs::PoseStamped pose_msg = retypePose(pose);
  pose_estimate_pub_.publish(pose_msg);

  // Constrict trajectory estimate message and publish it
  path_estimate_msg_.header.stamp = ros::Time::now();
  path_estimate_msg_.header.frame_id = "/map";
  path_estimate_msg_.poses.push_back(pose_msg);
  path_estimate_pub_.publish(path_estimate_msg_);
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
bool FSMLO::serviceClearTrajectory(
  std_srvs::Empty::Request &req,
  std_srvs::Empty::Response &res)
{
  ROS_INFO("[%s] Clearing trajectory vector...", PKG_NAME.c_str());
  lock_ = true;
  path_estimate_.clear();
  path_estimate_msg_.poses.clear();
  lock_ = false;

  return true;
}


/*******************************************************************************
 * If there is an initial pose then set it
*/
bool FSMLO::serviceInitialPose(
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

    ROS_INFO("[%s] Setting initial pose to (%.2f,%.2f,%.2f)",
      PKG_NAME.c_str(),
      std::get<0>(initial_pose_),
      std::get<1>(initial_pose_),
      std::get<2>(initial_pose_));
    return true;
  }
  else
  {
    ROS_ERROR("[%s] Failed to set initial pose", PKG_NAME.c_str());
    return false;
  }
}


/*******************************************************************************
 *
 */
bool FSMLO::serviceStart(
  std_srvs::Empty::Request &req,
  std_srvs::Empty::Response &res)
{
  ROS_INFO("[%s] Starting up...", PKG_NAME.c_str());
  lock_ = false;

  return true;
}


/*******************************************************************************
 *
 */
bool FSMLO::serviceStop(
  std_srvs::Empty::Request &req,
  std_srvs::Empty::Response &res)
{
  ROS_INFO("[%s] Shutting down...", PKG_NAME.c_str());
  lock_ = true;

  return true;
}


/*******************************************************************************
*/
  void
FSMLO::scanCallback(const sensor_msgs::LaserScan::Ptr& scan_msg)
{
  if (lock_)
  {
    ROS_WARN("[%s] Will not process this scan", PKG_NAME.c_str());
    return;
  }

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
  FSM::Utils::scan2points(sv_, origin, &vp);

  FSM::output_params op;
  std::tuple<double,double,double> diffs;

  // ---------------------------------------------------------------------------
  // Do your magic thing
  FSM::Match::fmtdbh(sr_, origin, vp, r2rp_, c2rp_, ip_, &op,
    &diffs);
  ROS_INFO("[%s] FSM executed in %.1f ms", PKG_NAME.c_str(), 1000*op.exec_time);
  // ---------------------------------------------------------------------------

  // Compute transform
  M = FSM::Utils::computeTransform(diffs, M);

  std::tuple<double,double,double> result_pose;
  std::get<0>(result_pose) = M(0,2);
  std::get<1>(result_pose) = M(1,2);
  std::get<2>(result_pose) = atan2(M(1,0), M(0,0));

  printf("%f,%f,%f\n", std::get<0>(result_pose),
    std::get<1>(result_pose),
    std::get<2>(result_pose));


  // Append resulting pose to the estimated trajectory
  path_estimate_.push_back(result_pose);

  // Publish `result_pose` and `path_estimate_`
  publishResults(result_pose);

  // The new scan (at time t) becomes the old scan (at time t+1)
  sv_ = sr_;

  // Unlock
  lock_ = false;
}
