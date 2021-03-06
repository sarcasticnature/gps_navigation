#include "gps_navigation/waypoint_printer.h"

namespace gps_navigation
{

WaypointPrinter::WaypointPrinter(ros::NodeHandle nh) : nh_(nh)
{
  if(!nh_.getParam("filename", filename_)){
    ROS_ERROR("Could not get parameter \"filename\", shutting down");
    ros::shutdown();
  }
  waypoint_file_.open(ros::package::getPath("gps_navigation")
                      + "/waypoint_files/"
                      + filename_
                      + ".csv");
  if(!waypoint_file_.good()){
    ROS_ERROR("Could not open waypoint file, shutting down");
    ros::shutdown();
  }
  ROS_INFO_STREAM("Reading file: " << ros::package::getPath("gps_navigation")
                  + "/waypoint_files/"
                  + filename_
                  + ".csv");
  readFile();
  printFile();
  ros::shutdown();
}

void WaypointPrinter::readFile()
{
  Waypoint point;
  std::string datum;
  while (std::getline(waypoint_file_, datum, ',')) {
    point.lat = stof(datum);
    std::getline(waypoint_file_, datum, '\n');
    point.lon = stof(datum);

    waypoints_.push_back(point);
  }
}

void WaypointPrinter::printFile()
{
  ROS_INFO_STREAM("Read waypoints:");
  for (auto point : waypoints_) {
    ROS_INFO_STREAM(std::fixed << std::setprecision(8) << "Lat: " << point.lat << " Long: " << point.lon);
  }
}

}

