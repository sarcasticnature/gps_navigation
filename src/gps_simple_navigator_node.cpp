#include "ros/ros.h"
#include "gps_navigation/gps_simple_navigator.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "gps_simple_navigator");
    ros::NodeHandle nh("~");

    gps_navigation::GPSSimpleNavigator gps_simple_navigator(nh);

    ros::spin();
    return 0;
}

