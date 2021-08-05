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


WaypointNavigator::WaypointNavigator(const ros::NodeHandle &nh)
    : nh_(nh),
      ac_("move_base", true)
{
  if(!nh_.getParam("world_frame", world_frame_)){
    ROS_ERROR("Could not get parameter \"world_frame\", shutting down");
    ros::shutdown();
  }
  if(!nh_.getParam("timeout", timeout_)){
    ROS_ERROR("Could not get parameter \"timeout\", shutting down");
    ros::shutdown();
  }
  if(!nh_.getParam("filename", filename_)){
    ROS_ERROR("Could not get parameter \"filename\", shutting down");
    ros::shutdown();
  }

  from_ll_client_ = nh_.serviceClient<robot_localization::FromLL>("/fromLL");

  waypoint_file_.open(ros::package::getPath("gps_navigation")
                      + "/waypoint_files/"
                      + filename_);

  ROS_INFO("Waiting for action server");
  ac_.waitForServer();
  readWaypoints();
}


void WaypointNavigator::navigate()
{
  for (auto waypoint : waypoints_) {
    waypointToGoal(waypoint);

    ac_.sendGoal(goal_);
    bool got_result = ac_.waitForResult(ros::Duration(timeout_));
  
    if (got_result) {
      auto status = ac_.getState();
      if (status != actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_ERROR_STREAM("move_base goal failed with status: "
                        << status.toString());
        ros::shutdown();
      }
      else
        ROS_INFO("Goal reached, moving on to next waypoint");
    }
    else {
      ROS_ERROR("waitForResult timed out!");
      ac_.cancelGoal();
      ros::shutdown();
    }
  }
  ROS_INFO("Reached last waypoint, shutting down");
  ros::shutdown();
}


void WaypointNavigator::readWaypoints()
{
  Waypoint point;
  std::string datum;
  ROS_INFO("Reading waypoint file");
  while (std::getline(waypoint_file_, datum, ',')) {
    point.lat = stod(datum);
    std::getline(waypoint_file_, datum, '\n');
    point.lon = stod(datum);

    waypoints_.push_back(point);
  }
}

void WaypointNavigator::waypointToGoal(const Waypoint& waypoint)
{
  from_ll_srv_.request.ll_point.latitude = waypoint.lat;
  from_ll_srv_.request.ll_point.longitude = waypoint.lon;
  if (!from_ll_client_.call(from_ll_srv_)) {
    ROS_ERROR("Could not call service /fromLL !");
    return;
  }
  ROS_INFO_STREAM("Transformed waypoint lat: " << waypoint.lat
                  << " lon: " << waypoint.lon
                  << " into map point x: " << from_ll_srv_.response.map_point.x
                  << " y: " << from_ll_srv_.response.map_point.y);

  goal_.target_pose.header.stamp = ros::Time::now();
  goal_.target_pose.header.frame_id = world_frame_;
  goal_.target_pose.pose.position.x = from_ll_srv_.response.map_point.x;
  goal_.target_pose.pose.position.y = from_ll_srv_.response.map_point.y;
  goal_.target_pose.pose.orientation.w = 1.0;

}

} // namespace gps_navigation
