#include "gps_navigation/gps_simple_navigator.h"

namespace gps_navigation
{

GPSSimpleNavigator::GPSSimpleNavigator(const ros::NodeHandle &nh) : nh_(nh)
{
    setpoint_sub_ = nh_.subscribe("/gps_nav_setpoint",
                                  10,
                                  &GPSSimpleNavigator::setpointCallback,
                                  this);

    goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);

    from_ll_client_ = nh_.serviceClient<robot_localization::FromLL>("/fromLL");

    if(!nh_.getParam("world_frame", world_frame_)){
      ROS_ERROR("Could not get parameter \"world_frame\", shutting down");
      ros::shutdown();
    }

    ROS_INFO_STREAM("world_frame is: " << world_frame_);

}

void GPSSimpleNavigator::setpointCallback(const geographic_msgs::GeoPoint::ConstPtr& msg)
{
    ROS_INFO("Recieved setpoint, lat: %f long: %f alt: %f",
             msg->latitude, msg->longitude, msg->altitude);
    from_ll_srv_.request.ll_point = *msg;
    if (from_ll_client_.call(from_ll_srv_)) {
        ROS_INFO("Transformed to map point, x: %f y: %f z: %f",
                 from_ll_srv_.response.map_point.x,
                 from_ll_srv_.response.map_point.y,
                 from_ll_srv_.response.map_point.z);
    }
    else {
        ROS_ERROR("Failed to call service fromLL");
        return;
    }

    goal_msg_.header.stamp = ros::Time::now();
    goal_msg_.header.frame_id = world_frame_;
    goal_msg_.pose.position.x = from_ll_srv_.response.map_point.x;
    goal_msg_.pose.position.y = from_ll_srv_.response.map_point.y;
    goal_msg_.pose.position.z = from_ll_srv_.response.map_point.z;
    goal_msg_.pose.orientation.w = 1;

    goal_pub_.publish(goal_msg_);

}

}

