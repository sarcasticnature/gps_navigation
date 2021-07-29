#include "ros/ros.h"
#include "gps_navigation/waypoint_navigator.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "waypoint_navigator");
    ros::NodeHandle nh("~");

    gps_navigation::WaypointNavigator navigator(nh);
    navigator.navigate();

    ros::spin();
    return 0;
}

