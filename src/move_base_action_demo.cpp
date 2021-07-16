#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

using Client = actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;

class DriveActionClient
{
public:
  explicit DriveActionClient(ros::NodeHandle nh) : ac_("move_base", true),
                                                   nh_(nh)
  {
    ROS_INFO("Waiting for server");
    ac_.waitForServer();
    ROS_INFO("Server started");
  }

  void doneCallback(const actionlib::SimpleClientGoalState& state,
                    const move_base_msgs::MoveBaseResultConstPtr& result)
  {


  }

  void feedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr& fb)
  {
    ROS_INFO_STREAM("Feedback: x=" << fb->base_position.pose.position.x
                    << " y=" << fb->base_position.pose.position.y);
  }

  void driveForward(int distance)
  {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "base_link";
    goal.target_pose.header.stamp = ros::Time::now();
  
    goal.target_pose.pose.position.x = distance;
    goal.target_pose.pose.orientation.w = 1.0;

    ac_.sendGoal(goal,
                 Client::SimpleDoneCallback(),
                 Client::SimpleActiveCallback(),
                 boost::bind(&DriveActionClient::feedbackCallback, this, _1));
  }

private:
  ros::NodeHandle nh_;
  Client ac_;
};


int main(int argc, char** argv)
{

  ros::init(argc, argv, "move_base_action_demo");
  ros::NodeHandle nh;

  DriveActionClient drive_ac(nh);

  drive_ac.driveForward(1.5);
  ros::spin();
  return 0;
}


