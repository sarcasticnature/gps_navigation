#include <string>
#include <fstream>

#include "ros/ros.h"
#include "ros/package.h"
#include "geometry_msgs/PoseStamped.h"
#include "geographic_msgs/GeoPoint.h"
#include "robot_localization/FromLL.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "actionlib/client/simple_action_client.h"


namespace gps_navigation
{

class WaypointNavigator
{
public:
  struct Waypoint
  {
    double lat;
    double lon;
  };

  using Client = actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;
  explicit WaypointNavigator(const ros::NodeHandle &nh);
  void navigate();

private:
  void readWaypoints();
  void waypointToGoal(const Waypoint& waypoint);

  ros::NodeHandle nh_;
  ros::ServiceClient from_ll_client_;
  Client ac_;
  robot_localization::FromLL from_ll_srv_;

  std::string world_frame_;
  double timeout_;
  std::string filename_;
  std::fstream waypoint_file_;
  std::vector<Waypoint> waypoints_;
  move_base_msgs::MoveBaseGoal goal_;
};

} // namespace gps_navigation
