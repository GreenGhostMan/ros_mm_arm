#include <ros/ros.h>
#include <tf/tf.h>

#include <actionlib/server/simple_action_server.h>
#include <ros_mm_arm_developer/PickAndPlaceAction.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/PoseArray.h>

// for gripper command in RViz
#include <sensor_msgs/JointState.h>
// for gripper command in Real Robot
#include <std_msgs/Float64.h>


namespace RosMyanmar
{

class PickAndPlaceServer
{
private:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<ros_mm_arm_developer::PickAndPlaceAction> as_;
  std::string action_name_;

  ros_mm_arm_developer::PickAndPlaceFeedback     feedback_;
  ros_mm_arm_developer::PickAndPlaceResult       result_;
  ros_mm_arm_developer::PickAndPlaceGoalConstPtr goal_;

  //ros::Publisher target_pose_pub_;
  //ros::Subscriber pick_and_place_sub_;

  // Move groups to control arm and gripper with MoveIt!
  moveit::planning_interface::MoveGroupInterface arm_;
  //moveit::planning_interface::MoveGroupInterface gripper_;

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  //moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool status;

  // Pick and place parameters
  std::string arm_link;
  double gripper_open;
  double gripper_closed;
  double attach_time;
  double detach_time;
  double z_up;

  //static const std::string arm_e;
  //std::string arm_g="hello";
  //const std::string arm_z="bink";
  //arm_e = "hacker";
  //static const std::string GRIPPER = "gripper_group";


public:
  PickAndPlaceServer(const std::string name) :
    nh_("~"), as_(name, false), action_name_(name), arm_("arm_group")//, gripper_("gripper_group")
  {
    // Read specific pick and place parameters
    nh_.param("grasp_attach_time", attach_time, 1.5);
    nh_.param("grasp_detach_time", detach_time, 1.0);

    // Getting Basic Information
    // We can print the name of the reference frame for this robot.
    //ROS_INFO("Reference frame: %s", arm_.getPlanningFrame().c_str());
    //ROS_INFO("End effector link: %s", arm_.getEndEffectorLink().c_str());

    // Register the goal and feedback callbacks
    as_.registerGoalCallback(boost::bind(&PickAndPlaceServer::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&PickAndPlaceServer::preemptCB, this));

    as_.start();
  }



  void goalCB()
  {    
    ROS_INFO("[pick and place] Received goal!");
    goal_ = as_.acceptNewGoal();
    arm_link = goal_->frame;
    gripper_open = goal_->gripper_open;
    gripper_closed = goal_->gripper_closed;
    z_up = goal_->z_up;

    //arm_.setPoseReferenceFrame(arm_link);

    // Allow some leeway in position (meters) and orientation (radians)
    //arm_.setGoalPositionTolerance(0.001);
    //arm_.setGoalOrientationTolerance(0.1);

    // Allow replanning to increase the odds of a solution
    //arm_.allowReplanning(true);

    //pickAndPlace(goal_->pickup_pose, goal_->place_pose);
    pickAndPlace(goal_->pickup_pose);
  }

  
  void preemptCB()
  {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    as_.setPreempted();
  }

  void pickAndPlace(const geometry_msgs::Pose& start_pose)
  {
    ROS_INFO("pickAndPlace Function start!");

    // // open gripper
    // gripper_.setNamedTarget("gripper_open");
    // status = (gripper_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    // if(status) {
    //   gripper_.move();
    //   ros::Duration(detach_time).sleep();
    // }
    
    
    
    // ROS_INFO("Setting Pose target pick_pose");    
    
    // geometry_msgs::Pose up_pose = start_pose;    
    // arm_.setPoseTarget(start_pose);
    
    // status = (arm_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    // ROS_INFO("Visualizing plan start pose goal %s", status ? "OK" : "FAILED");
    
    // if(status)
    // {
    //   arm_.move();
    //   ros::Duration(attach_time).sleep();
      
      
    // }
    // else
    // {
    //   return;
    // } 
    
    as_.setSucceeded(result_);
  }
  bool openGripper()
  {
    
  }
  bool closeGripper()
  {

  }
}; // class
}; // namespace



int main(int argc, char** argv)
{
  ros::init(argc, argv, "pick_and_place_action_server");

  RosMyanmar::PickAndPlaceServer server("pick_and_place");

  // Setup an multi-threaded spinner as the move groups operations need continuous spinning
  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();

  return 0;
}
