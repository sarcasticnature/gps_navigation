#include "ros/ros.h"
#include "gps_navigation/waypoint_printer.h"


int main(int argc, char** argv)
{
  ros::init(argc, argv, "waypoint_printer");
  ros::NodeHandle nh("~");

  gps_navigation::WaypointPrinter printer(nh);

  ros::spin();

  return 0;
}

