#include <string>
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geographic_msgs/GeoPoint.h"
#include "robot_localization/FromLL.h"


namespace gps_navigation
{

class GPSSimpleNavigator
{
public:
    explicit GPSSimpleNavigator(const ros::NodeHandle &nh);

private:
    void setpointCallback(const geographic_msgs::GeoPoint::ConstPtr& msg);

    ros::NodeHandle nh_;
    ros::Subscriber setpoint_sub_;
    ros::Publisher goal_pub_;
    ros::ServiceClient from_ll_client_;
    geometry_msgs::PoseStamped goal_msg_;
    robot_localization::FromLL from_ll_srv_;
    std::string world_frame_;
};

}
