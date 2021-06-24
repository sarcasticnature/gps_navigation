#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "geographic_msgs/GeoPoint.h"


class GPSTransform
{
public:
    GPSTransform(ros::NodeHandle &nh);

private:
    void setpointCallback(const geographic_msgs::GeoPoint::ConstPtr& msg);
    ros::NodeHandle nh_;
    ros::Subscriber setpoint_sub_;
};


GPSTransform::GPSTransform(ros::NodeHandle &nh) : nh_(nh)
{
    setpoint_sub_ = nh_.subscribe("husky_gps_setpoint",
                                  10,
                                  &GPSTransform::setpointCallback,
                                  this);

}

void GPSTransform::setpointCallback(const geographic_msgs::GeoPoint::ConstPtr& msg)
{
    ROS_INFO("Recieved setpoint, lat: %f long: %f alt: %f",
             msg->latitude, msg->longitude, msg->altitude);
}

