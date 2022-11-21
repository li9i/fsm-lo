#include <fsm_lidom.h>

/*******************************************************************************
*/
FSMLO::FSMLO(ros::NodeHandle nh, ros::NodeHandle nh_private) :
  nh_(nh),
  nh_private_(nh_private),
  lock_(false),
  sc_(0),
  initial_pose_{0,0,0}
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
    "fsm_lidom/set_initial_service", &FSMLO::initialPoseService, this);

  // fsm's pose estimate
  pose_estimate_pub_ =
    nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(pose_estimate_topic_, 1);
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
  if (!nh_private_.getParam ("sigma_noise_real", ip_.sigma_noise_real))
  {
    ROS_WARN("[FSM_LIDOM] no sigma_noise_real param found; resorting to defaults");
    ip_.sigma_noise_real = 0.2;
  }

  // ---------------------------------------------------------------------------
  if (!nh_private_.getParam ("sigma_noise_map", ip_.sigma_noise_map))
  {
    ROS_WARN("[FSM_LIDOM] no sigma_noise_map param found; resorting to defaults");
    ip_.sigma_noise_map = 0.2;
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

  // ---------------------------------------------------------------------------
  if (!nh_private_.getParam ("enforce_terminal_constraint", ip_.enforce_terminal_constraint))
  {
    ROS_WARN("[FSM_LIDOM] no enforce_terminal_constraint param found; resorting to defaults");
    ip_.enforce_terminal_constraint = false;
  }

  // ---------------------------------------------------------------------------
  if (!nh_private_.getParam ("enforce_early_gearup", ip_.enforce_early_gearup))
  {
    ROS_WARN("[FSM_LIDOM] no enforce_early_gearup param found; resorting to defaults");
    ip_.enforce_early_gearup = false;
  }

  assert(SIZE_SCAN > 0);
  assert(ip_.num_iterations > 0);
  assert(ip_.xy_bound >= 0.0);
  assert(ip_.t_bound >= 0.0);
  assert(ip_.sigma_noise_real >= 0.0);
  assert(ip_.sigma_noise_map >= 0.0);
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

  std::tuple<double,double,double> zero_pose;
  std::get<0>(zero_pose) = 0.0;
  std::get<1>(zero_pose) = 0.0;
  std::get<2>(zero_pose) = 0.0;

  std::tuple<double,double,double> virtual_pose = zero_pose;
  std::tuple<double,double,double> result_pose = zero_pose;

  // The reference scan in 2d points
  std::vector< std::pair<double,double> > vp;
  FSM::Utils::scan2points(sv_, virtual_pose, &vp);

  FSM::output_params op;

  // Do your magic thing
  FSM::Match::fmtdbh(sr_, virtual_pose, vp, "FMT", r2rp_, c2rp_, ip_, &op,
    &result_pose);

  // The new scan (at time t) becomes the old scan (at time t+1)
  sv_ = sr_;


  // Unlock
  lock_ = false;
}
