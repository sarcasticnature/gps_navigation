#include <string>
#include <fstream>
#include <vector>

#include "ros/ros.h"
#include "ros/package.h"

namespace gps_navigation
{

class WaypointPrinter
{
public:
  struct Waypoint
  {
    double lat;
    double lon;
  };

  explicit WaypointPrinter(ros::NodeHandle nh);


private:
  void readFile();
  void printFile();

  ros::NodeHandle nh_;
  std::string filename_;
  std::ifstream waypoint_file_;
  std::vector<Waypoint> waypoints_;

};


} // namespace gps_navigation

