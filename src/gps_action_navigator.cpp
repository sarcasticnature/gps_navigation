#include "gps_navigation/gps_action_navigator.h"

namespace gps_navigation
{

GPSActionNavigator::GPSActionNavigator(const ros::NodeHandle &nh)
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

  setpoint_sub_ = nh_.subscribe("/gps_nav_setpoint",
                                10,
                                &GPSActionNavigator::setpointCallback,
                                this);

  countdown_ = nh.createTimer(ros::Duration(timeout_),
                            boost::bind(&GPSActionNavigator::timeoutCallback,
                                        this,
                                        _1),
                            true,   // oneshot
                            false); // do not auto-start

  from_ll_client_ = nh_.serviceClient<robot_localization::FromLL>("/fromLL");


}

void GPSActionNavigator::setpointCallback(const geographic_msgs::GeoPoint::ConstPtr& msg)
{
  ROS_INFO("Recieved setpoint, lat: %f long: %f alt: %f",
           msg->latitude, msg->longitude, msg->altitude);
  from_ll_srv_.request.ll_point = *msg;
  if (from_ll_client_.call(from_ll_srv_)) {
    ROS_INFO("Transformed to map point, x: %f y: %f z: %f",
             from_ll_srv_.response.map_point.x,
             from_ll_srv_.response.map_point.y,
             from_ll_srv_.response.map_point.z);

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.header.frame_id = world_frame_;
    goal.target_pose.pose.position.x = from_ll_srv_.response.map_point.x;
    goal.target_pose.pose.position.y = from_ll_srv_.response.map_point.y;
    goal.target_pose.pose.orientation.w = 1.0;
  
    countdown_.start();
    ROS_INFO("Sending goal to move_base");
    ac_.sendGoal(goal,
                boost::bind(&GPSActionNavigator::doneCallback, this, _1, _2),
                Client::SimpleActiveCallback(),
                boost::bind(&GPSActionNavigator::feedbackCallback, this, _1));
  }
  else {
    ROS_ERROR("Failed to call service fromLL");
  }
}

void GPSActionNavigator::doneCallback(const actionlib::SimpleClientGoalState& state,
                                      const move_base_msgs::MoveBaseResultConstPtr& result)
{
  ROS_INFO_STREAM("Action completed with state: " << state.toString());
  countdown_.stop();
}

void GPSActionNavigator::timeoutCallback(const ros::TimerEvent&)
{
  ROS_INFO("Timeout hit, canceling move_base action");
  ac_.cancelGoal();
  ROS_INFO("Action canceled");

  countdown_.stop();
}

void GPSActionNavigator::feedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr& fb)
{
  ROS_INFO_STREAM("Feedback: x=" << fb->base_position.pose.position.x
                  << " y=" << fb->base_position.pose.position.y);
}

}

