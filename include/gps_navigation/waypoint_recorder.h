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


WaypointRecorder::WaypointRecorder(ros::NodeHandle nh) : nh_(nh)
{
  trigger_sub_ = nh_.subscribe("record_waypoint",
                               10,
                               &WaypointRecorder::triggerCallback,
                               this);
  if(!nh_.getParam("gps_topic", gps_topic_)){
    ROS_ERROR("Could not get parameter \"gps_topic\", shutting down");
    ros::shutdown();
  }
  ROS_INFO_STREAM("GPS topic is: " << gps_topic_);
  gps_sub_ = nh_.subscribe(gps_topic_,
                               10,
                               &WaypointRecorder::gpsCallback,
                               this);
  if(!nh_.getParam("waypoint_file", file_name_)){
    ROS_ERROR("Could not get parameter \"waypoint_file\", shutting down");
    ros::shutdown();
  }
  waypoint_file_.open(file_name_);
}

void WaypointRecorder::triggerCallback(const std_msgs::Empty::ConstPtr&)
{
  ROS_INFO("Recording waypoint");
  recordWaypoint();
}

void WaypointRecorder::gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  latitude_ = msg->latitude;
  longitude_ = msg->longitude;
}

void WaypointRecorder::recordWaypoint()
{
  ROS_INFO_STREAM("Recording data to file: " << std::fixed << latitude_ << ',' << longitude_);
  waypoint_file_ << std::fixed << std::setprecision(8) << latitude_ << ',' << longitude_ << std::endl;
}



} // namespace gps_navigation

