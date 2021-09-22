#include <string>
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geographic_msgs/GeoPoint.h"
#include "robot_localization/FromLL.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "actionlib/client/simple_action_client.h"


namespace gps_navigation
{

class GPSActionNavigator
{
public:
  using Client = actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;
  explicit GPSActionNavigator(const ros::NodeHandle &nh);

private:
  void setpointCallback(const geographic_msgs::GeoPoint::ConstPtr& msg);
  void doneCallback(const actionlib::SimpleClientGoalState& state,
                    const move_base_msgs::MoveBaseResultConstPtr& result);
  void timeoutCallback(const ros::TimerEvent&);
  void feedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr& fb);

  ros::NodeHandle nh_;
  ros::Subscriber setpoint_sub_;
  ros::Timer countdown_;
  ros::ServiceClient from_ll_client_;
  Client ac_;
  robot_localization::FromLL from_ll_srv_;
  std::string world_frame_;
  float timeout_;

};



}
