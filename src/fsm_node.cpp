#include <fsm_lidom.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fsm_lidom");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  FSMLO fsmlo(nh, nh_private);

  ros::spin();
  return 0;
}
