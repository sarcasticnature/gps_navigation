#include <string>
#include <fstream>
#include <sstream>

#include "ros/ros.h"
#include "ros/package.h"

namespace gps_navigation
{

class WaypointPrinter
{
public:
  explicit WaypointPrinter(ros::NodeHandle nh);


private:
  void printFile();

  ros::NodeHandle nh_;
  std::string filename_;
  std::ifstream waypoint_file_;

};

WaypointPrinter::WaypointPrinter(ros::NodeHandle nh) : nh_(nh)
{
  ROS_INFO_STREAM("Package path is: " << ros::package::getPath("gps_navigation"));
  nh_.getParam("filename", filename_);
  waypoint_file_.open(ros::package::getPath("gps_navigation")
                      + "/waypoint_files/"
                      + filename_);
  ROS_INFO_STREAM("Full path is: " << ros::package::getPath("gps_navigation")
                      + "/waypoint_files/"
                      + filename_);
  printFile();
  ros::shutdown();
}

void WaypointPrinter::printFile()
{
  std::string lat;
  std::string lon;
  while (std::getline(waypoint_file_, lat, ',')) {
    std::getline(waypoint_file_, lon, '\n');
    ROS_INFO_STREAM("Read waypoint: latitude: " << lat
                    << " longitude: " << lon);

  }

}

} // namespace gps_navigation

