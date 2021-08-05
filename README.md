# gps_navigation
A ros package for GPS navigation on the Clearpath Husky and Heron

### How to use:
##### Recording Waypoints
The `waypoint_navigator` node can be used to collect waypoints at the robot's current location.  Waypoints are stored as latitude and longitude coordinates in .csv format.  Waypoints are stored in and read from the `waypoint_files` directory.

Steps to use the node:
1. After sourcing the ros overlay, launch the node using `roslaunch gps_navigation waypoint_recorder.launch gps_topic:=$GPS_TOPIC_NAME`, where `$GPS_TOPIC_NAME` is the topic that NavSatFix message is published to.  This is `/gps/fix` for the Husky and `/navsat/fix` for the Heron.  Optionally, you can provide an additional argument to the launch file to specify the name of the output file.  If no filename is specified, it defaults to "waypoint_default.csv"
2. Whenever the robot reaches a location you want to record, you will need to publish a `std_msgs/Empty` message on the topic `/waypoint_recorder/record_waypoint`.  The full command is: `rostopic pub --once /waypoint_recorder/record_waypoint std_msgs/Empty "{}"` but once you type in `rostopic pub --once /waypoint_recorder` you can hit tab a couple of times and the rest will auto complete.
3. Continue driving the robot to new locations and recording waypoints until you are done, then kill the node using Ctrl-c.  Once finished, the .csv file has been created.

