#include "ros/ros.h"
#include "gps_navigation/gps_transform_simple.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "gps_transform_simple");
    ros::NodeHandle nh("~");

    gps_navigation::GPSTransformSimple gps_transform_simple(nh);

    ros::spin();
    return 0;
}

