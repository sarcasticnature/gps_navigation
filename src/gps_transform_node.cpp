#include "ros/ros.h"
#include "gps_navigation/gps_transform.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "gps_transform");
    ros::NodeHandle nh;

    GPSTransform gps_transform(nh);

    ros::spin();
    return 0;
}

