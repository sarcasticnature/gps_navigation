# gps_navigation
A ros package for GPS navigation on the Clearpath Husky and Heron

### How to use:
##### Navigating to GPS coordinates
There are two ways to manually navigate to a set of GPS coordinates.  The first, `gps_simple_navigator`, simply publishes to the topic `/move_base_simple/goal` to tell `move_base` where to go.  The second, `gps_action_navigator`, utilizes ROS actions to allow the code to cancel the goal and stop the robot if needed.

Steps to use `gps_simple_navigator`:
1. Launch `gps_simple.launch` or `heron_gps_simple.launch`.  `gps_simple.launch` takes in the world frame as an argument, whereas `heron_gps_simple.launch` defaults to the `odom` frame.
2. Wait for the goal to be reached.  Note that the node will run for an 'unlimited' amout of time, until either the goal is reached or `move_base` is stopped.



##### Recording Waypoints
The `waypoint_recorder` node can be used to collect waypoints at the robot's current location.  Waypoints are stored as latitude and longitude coordinates in .csv format.  Waypoints are stored in and read from the `waypoint_files` directory.

Steps to use:
1. After sourcing the ros overlay, launch the node using `roslaunch gps_navigation waypoint_recorder.launch gps_topic:=$GPS_TOPIC_NAME`, where `$GPS_TOPIC_NAME` is the topic that NavSatFix message is published to.  This is `/gps/fix` for the Husky and `/navsat/fix` for the Heron.  Optionally, you can provide an additional argument to the launch file to specify the name of the output file.  If no filename is specified, it defaults to "waypoint_default.csv"
2. Whenever the robot reaches a location you want to record, you will need to publish a `std_msgs/Empty` message on the topic `/waypoint_recorder/record_waypoint`.  The full command is: `rostopic pub --once /waypoint_recorder/record_waypoint std_msgs/Empty "{}"` but once you type in `rostopic pub --once /waypoint_recorder` you can hit tab a couple of times and the rest will auto complete.
3. Continue driving the robot to new locations and recording waypoints until you are done, then kill the node using Ctrl-c.

#### Navigating to waypoints (autonomously)
The `waypoint_navigator` node can be used to navigate to wayponts stored in a .csv file with the same format as the `waypoint_recorder` node creates.  A .csv generated manually can also be used, so long as it is located in the `waypoint_files` directory and matches the same format.  

Steps to use:
1. Generate a .csv file and place it in the `waypoint_files` directory.
2. Launch the waypoint navigator.
    * `waypoint_navigator` should work for an arbitrary robot (with the correct arguments), whereas `heron_waypoint_nav` provides defaults for all arguments.
    * Parameters:
      * `world_frame`: the world frame for the target robot.  For robots that navigate based on a map or SLAM, this will be the `map` frame.  For robots that navigate without a map using only odometry (such as the Heron), this will be the `odom` frame.
      * `timeout`: length of time a move_base goal will be allowed to run before being cancelled.
      * `filename`: the name of the .csv file to be read.  **Note:** the file extension (.csv) should not be included in this argument.  Defaults to `waypoint_default`.
3. The node will run until either the last waypoint in the file has been reached or the timeout expires when moving to the next waypoint.



