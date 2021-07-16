#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geographic_msgs/GeoPoint.h"
#include "robot_localization/FromLL.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "actionlib/client/simple_action_client.h"


namespace gps_navigation
{

class GPSTransformAction
{
public:
  using Client = actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;
  explicit GPSTransformAction(const ros::NodeHandle &nh);

private:
  void setpointCallback(const geographic_msgs::GeoPoint::ConstPtr& msg);
  void timeoutCallback(const ros::TimerEvent&);
  void feedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr& fb);

  ros::NodeHandle nh_;
  ros::Subscriber setpoint_sub_;
  ros::Timer timeout_;
  ros::ServiceClient from_ll_client_;
  Client ac_;
  robot_localization::FromLL from_ll_srv_;

};


GPSTransformAction::GPSTransformAction(const ros::NodeHandle &nh)
    : nh_(nh),
      ac_("move_base", true)
{
  setpoint_sub_ = nh_.subscribe("gps_nav_setpoint",
                                10,
                                &GPSTransformAction::setpointCallback,
                                this);

  timeout_ = nh.createTimer(ros::Duration(3.0),
                            boost::bind(&GPSTransformAction::timeoutCallback, this, _1),
                            true,   // oneshot
                            false); // do not auto-start

  from_ll_client_ = nh_.serviceClient<robot_localization::FromLL>("fromLL");

}

void GPSTransformAction::setpointCallback(const geographic_msgs::GeoPoint::ConstPtr& msg)
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
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.pose.position.x = from_ll_srv_.response.map_point.x;
    goal.target_pose.pose.position.y = from_ll_srv_.response.map_point.y;
    goal.target_pose.pose.orientation.w = 1.0;
  
    timeout_.start();
    ROS_INFO("Sending goal to move_base");
    ac_.sendGoal(goal,
                Client::SimpleDoneCallback(),
                Client::SimpleActiveCallback(),
                boost::bind(&GPSTransformAction::feedbackCallback, this, _1));
  }
  else {
    ROS_ERROR("Failed to call service fromLL");
  }
}

void GPSTransformAction::timeoutCallback(const ros::TimerEvent&)
{
  ROS_INFO("Timeout hit, canceling move_base action");
  ac_.cancelGoal();
  ROS_INFO("Action canceled");
}

void GPSTransformAction::feedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr& fb)
{
  ROS_INFO_STREAM("Feedback: x=" << fb->base_position.pose.position.x
                  << " y=" << fb->base_position.pose.position.y);

}

}
