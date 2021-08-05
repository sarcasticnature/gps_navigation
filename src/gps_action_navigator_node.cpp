#include "ros/ros.h"
#include "gps_navigation/gps_action_navigator.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "gps_action_navigator");
    ros::NodeHandle nh("~");

    gps_navigation::GPSActionNavigator gps_action_navigator(nh);

    ros::spin();
    return 0;
}

