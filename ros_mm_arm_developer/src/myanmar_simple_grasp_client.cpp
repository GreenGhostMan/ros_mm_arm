#include <ros/ros.h>
#include <ros_mm_arm_developer/PickAndPlaceAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose.h>

const std::string arm_link = "base_link";
const double gripper_open = 0.03;
const double gripper_closed = 0.014;

const double z_up = 0.08;
const double z_down = -0.05;

const double block_size = 0.02;


class RosMyanmar
{
private:
    
  ros::NodeHandle nh_;
  
  // Actions
  actionlib::SimpleActionClient<ros_mm_arm_developer::PickAndPlaceAction> pick_and_place_action_;
  ros_mm_arm_developer::PickAndPlaceGoal pick_and_place_goal_;
  ros::Subscriber sub_;

public:

  RosMyanmar() :
    pick_and_place_action_("pick_and_place", true)
  {
    // Initialize goals
    pick_and_place_goal_.frame = arm_link;
    pick_and_place_goal_.z_up = z_up;
    pick_and_place_goal_.gripper_open = gripper_open;
    pick_and_place_goal_.gripper_closed = gripper_closed;
    
    ROS_INFO("Finished initializing, waiting for servers...");
    
    pick_and_place_action_.waitForServer();
    
    ROS_INFO("Found servers.");

    
    sub_ = nh_.subscribe("position_publisher_tf",10, &RosMyanmar::moveBlock, this);
       
   
  }
  
  
  void moveBlock(const geometry_msgs::PoseConstPtr& subscribedPose)
  {
    geometry_msgs::Pose start_pose;
    start_pose.position= subscribedPose->position;
    start_pose.orientation = subscribedPose->orientation;
    pick_and_place_goal_.pickup_pose = start_pose;
    ROS_INFO_STREAM("x:"<< start_pose.position.x<<std::endl<<
                    "y:"<< start_pose.position.y<<std::endl<<
                    "z:"<< start_pose.position.z<<std::endl<<
                    "orientation:"<<std::endl<<
                    "x:"<< start_pose.orientation.x<<std::endl<<
                    "y:"<< start_pose.orientation.y<<std::endl<<
                    "z:"<< start_pose.orientation.z<<std::endl<<
                    "w:"<< start_pose.orientation.w<<std::endl);
    pick_and_place_action_.sendGoalAndWait(pick_and_place_goal_, ros::Duration(30.0), ros::Duration(30.0));
    //pick_and_place_action_.sendGoal(pick_and_place_goal_);
    //pick_and_place_action_.waitForResult(ros::Duration(30.0)); 
    //sub_.shutdown();
    
  } 
};

int main(int argc, char** argv)
{
  // initialize node
  ros::init(argc, argv, "grasp_client");

  RosMyanmar rm;

  // everything is done in cloud callback, just spin
  ros::spin();
}

