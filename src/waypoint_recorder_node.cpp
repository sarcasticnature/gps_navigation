#include "ros/ros.h"
#include "gps_navigation/waypoint_recorder.h"


int main(int argc, char** argv)
{
  ros::init(argc, argv, "waypoint_recorder");
  ros::NodeHandle nh("~");

  gps_navigation::WaypointRecorder recorder(nh);

  ros::spin();

  return 0;
}

