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

private:
  void readWaypoints();
  void waypointToGoal(const Waypoint& waypoint);
  void doneCallback(const actionlib::SimpleClientGoalState& state,
                    const move_base_msgs::MoveBaseResultConstPtr& result);
  void timeoutCallback(const ros::TimerEvent&);
  void feedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr& fb);

  ros::NodeHandle nh_;
  ros::Subscriber setpoint_sub_;
  ros::Timer countdown_;
  ros::ServiceClient from_ll_client_;
  Client ac_;
  robot_localization::FromLL from_ll_srv_;

  std::string world_frame_;
  float timeout_;
  std::string filename_;
  std::fstream waypoint_file_;
  std::vector<Waypoint> waypoints_;
  move_base_msgs::MoveBaseGoal goal_;
};


WaypointNavigator::WaypointNavigator(const ros::NodeHandle &nh)
    : nh_(nh),
      ac_("move_base", true)
{
  nh_.getParam("world_frame", world_frame_);
  nh_.getParam("timeout", timeout_);
  nh_.getParam("filename", filename_);

  countdown_ = nh.createTimer(ros::Duration(timeout_),
                            boost::bind(&WaypointNavigator::timeoutCallback,
                                        this,
                                        _1),
                            true,   // oneshot
                            false); // do not auto-start

  from_ll_client_ = nh_.serviceClient<robot_localization::FromLL>("/fromLL");

  waypoint_file_.open(ros::package::getPath("gps_navigation")
                      + "/waypoint_files/"
                      + filename_);

  readWaypoints();
  waypointToGoal(waypoints_[0]);
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


void WaypointNavigator::doneCallback(const actionlib::SimpleClientGoalState& state,
                                      const move_base_msgs::MoveBaseResultConstPtr& result)
{
  ROS_INFO_STREAM("Action completed with state: " << state.toString());
  countdown_.stop();
}

void WaypointNavigator::timeoutCallback(const ros::TimerEvent&)
{
  ROS_INFO("Timeout hit, canceling move_base action");
  ac_.cancelGoal();
  ROS_INFO("Action canceled, shutting down");
  countdown_.stop();
  ros::shutdown();
}

void WaypointNavigator::feedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr& fb)
{
  ROS_INFO_STREAM("Feedback: x=" << fb->base_position.pose.position.x
                  << " y=" << fb->base_position.pose.position.y);

}

}
