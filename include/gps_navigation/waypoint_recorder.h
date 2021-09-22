#include <fstream>
#include <sstream>
#include <string>

#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "sensor_msgs/NavSatFix.h"

namespace gps_navigation
{

class WaypointRecorder
{
public:
  explicit WaypointRecorder(ros::NodeHandle nh); 


private:
  void triggerCallback(const std_msgs::Empty::ConstPtr&);
  void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
  void recordWaypoint();

  ros::NodeHandle nh_;
  ros::Subscriber trigger_sub_;
  ros::Subscriber gps_sub_;
  double latitude_;
  double longitude_;
  std::string file_name_;
  std::string gps_topic_;
  std::ofstream waypoint_file_;

};

} // namespace gps_navigation

