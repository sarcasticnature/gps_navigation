#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "geographic_msgs/GeoPoint.h"
#include "robot_localization/FromLL.h"


class GPSTransform
{
public:
    GPSTransform(ros::NodeHandle &nh);

private:
    void setpointCallback(const geographic_msgs::GeoPoint::ConstPtr& msg);
    ros::NodeHandle nh_;
    ros::Subscriber setpoint_sub_;
    ros::ServiceClient from_ll_client_;
    robot_localization::FromLL from_ll_srv_;
    
};


GPSTransform::GPSTransform(ros::NodeHandle &nh) : nh_(nh)
{
    setpoint_sub_ = nh_.subscribe("husky_gps_setpoint",
                                  10,
                                  &GPSTransform::setpointCallback,
                                  this);
    from_ll_client_ = nh_.serviceClient<robot_localization::FromLL>("fromLL");

}

void GPSTransform::setpointCallback(const geographic_msgs::GeoPoint::ConstPtr& msg)
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
    }
}

