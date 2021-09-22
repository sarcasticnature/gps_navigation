#include "gps_navigation/waypoint_recorder.h"

namespace gps_navigation
{

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

}

