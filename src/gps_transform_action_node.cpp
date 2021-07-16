#include "ros/ros.h"
#include "gps_navigation/gps_transform_action.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "gps_transform_action");
    ros::NodeHandle nh;

    gps_navigation::GPSTransformAction gps_transform_action(nh);

    ros::spin();
    return 0;
}

