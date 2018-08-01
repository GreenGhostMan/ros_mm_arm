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
  moveit::planning_interface::MoveGroupInterface gripper_;

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool status;

  // Pick and place parameters
  std::string arm_link;
  double gripper_open;
  double gripper_closed;
  double attach_time;
  double detach_time;
  double z_up;

  //const std::string arm_group ="arm_group";
  //const std::string gripper="gripper_group";

public:
  PickAndPlaceServer(const std::string name) :
    nh_("~"), as_(name, false), action_name_(name), arm_("arm_group"), gripper_("gripper_group")
  {
    // Read specific pick and place parameters
    nh_.param("grasp_attach_time", attach_time, 1.5);
    nh_.param("grasp_detach_time", detach_time, 1.0);

    // Getting Basic Information
    // We can print the name of the reference frame for this robot.
    ROS_INFO("Reference frame: %s", arm_.getPlanningFrame().c_str());
    ROS_INFO("End effector link: %s", arm_.getEndEffectorLink().c_str());

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

    arm_.setPoseReferenceFrame(arm_link);

    // Allow some leeway in position (meters) and orientation (radians)
    arm_.setGoalPositionTolerance(0.001);
    arm_.setGoalOrientationTolerance(0.1);

    // Allow replanning to increase the odds of a solution
    arm_.allowReplanning(true);


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

    bool arm_state = getReady();  
    
    if(arm_state) {
      bool move_arm = moveArmTo(start_pose);
    }
    else {
      return;
    }

    as_.setSucceeded(result_);
  }
  bool moveArmTo(const std::string& target)
  {
    ROS_DEBUG("[pick and place] Move arm to '%s' position", target.c_str());

    arm_.setNamedTarget(target);
    status = (arm_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Going object location is ................... %s", status ? "OK" : "FAILED");    
    if(status)
    {
      arm_.move();
      ros::Duration(attach_time).sleep();      
      return true;
    }
    else
    {
      return false;
      ROS_ERROR("NameTarget %s FAILED!",target.c_str() );
    } 
  }
  /**
   * Move arm to a target pose. Only position coordinates are taken into account; the
   * orientation is calculated according to the direction and distance to the target.
   * @param target Pose target to achieve
   * @return True of success, false otherwise
   */
  bool moveArmTo(const geometry_msgs::Pose& target)
  {
    int attempts = 0;
    while (attempts < 5)
    {
      geometry_msgs::PoseStamped modiff_target;
      modiff_target.header.frame_id = arm_link;
      modiff_target.pose = target;

      double x = modiff_target.pose.position.x;
      double y = modiff_target.pose.position.y;
      double z = modiff_target.pose.position.z;
      double d = sqrt(x*x + y*y);
      if (d > 0.23)
      {
        // Maximum reachable distance by the turtlebot arm is 23 cm
        ROS_ERROR("Target pose out of reach [%f > %f]", d, 0.23);
        as_.setAborted(result_);
        return false;
      }
      /*---------------------------------------------------------------------------------------
      Pitch is 90 (vertical) at 10(13 in ros_mm_arm) cm from the arm base; the farther the target is, 
      the closer to horizontal
      we point the gripper (0.205 = arm's max reach - vertical pitch distance + ε). 
      Yaw is the direction to the target.
      We also try some random variations of both to increase the chances of successful planning.
      Roll is simply ignored, as our arm lacks the proper dof.
      ----------------------------------------------------------------------------------------*/
      //double rp = 0.0;
      double rp = M_PI_2 - std::asin((d - 0.1)/0.105) + attempts*fRand(-0.05, +0.05);
      double ry = std::atan2(y, x) + attempts*fRand(-0.05, +0.05);
      double rr = 0.0;

      tf::Quaternion q = tf::createQuaternionFromRPY(rr, rp, ry);
      tf::quaternionTFToMsg(q, modiff_target.pose.orientation);

      // Slightly increase z proportionally to pitch to avoid hitting the table with the lower gripper corner
      ROS_DEBUG("z increase:  %f  +  %f", modiff_target.pose.position.z, std::abs(std::cos(rp))/50.0);
      modiff_target.pose.position.z += std::abs(std::cos(rp))/50.0;

      ROS_DEBUG("Set pose target [%.2f, %.2f, %.2f] [d: %.2f, r: %.2f, p: %.2f, y: %.2f]", x, y, z, d, rr, rp, ry);
      //target_pose_pub_.publish(modiff_target);

      if (arm_.setPoseTarget(modiff_target) == false)
      {
        ROS_ERROR("Set pose target [%.2f, %.2f, %.2f, %.2f] failed",
                  modiff_target.pose.position.x, modiff_target.pose.position.y, modiff_target.pose.position.z,
                  tf::getYaw(modiff_target.pose.orientation));
        as_.setAborted(result_);
        return false;
      }

      moveit::planning_interface::MoveItErrorCode result = arm_.move();
      if (bool(result) == true)
      {
        return true;
      }
      else
      {
        ROS_ERROR("[pick and place] Move to target failed (error %d) at attempt %d",
                  result.val, attempts + 1);
      }
      attempts++;
    }

    ROS_ERROR("[pick and place] Move to target failed after %d attempts", attempts);
    as_.setAborted(result_);
    return false;
  }
  // bool setMyPickPose(geometry_msgs::Pose start_pose) {

  //   ROS_INFO("Setting Pose target pick_pose");    
  //   geometry_msgs::Pose up_pose = start_pose;
        
  //   arm_.setPoseTarget(start_pose);
  //   //
  //   return true;
  // }
  bool getReady(){
    ROS_INFO("Getting Home and ready ....");
    bool go_status = goHome();
    if(go_status) {
      
      closeGripper();
      openGripper();
      ros::Duration(0.2).sleep();
      return true;
    }
    else {
      return false;
    }
    
  }
  bool goHome() 
  {
    arm_.setNamedTarget("home");
    status = (arm_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Going Home ................................. %s", status ? "OK" : "FAILED");
    if(status)
    {
      arm_.move();
      ros::Duration(attach_time).sleep();
      return true;
    }
    else{
      return false;
    }
  }
  bool openGripper()
  {
    // open gripper
    gripper_.setNamedTarget("gripper_open");
    gripper_.move();
    //ros::Duration(2.0).sleep();
    return true;
  }
  bool closeGripper()
  {
    // close gripper
    gripper_.setNamedTarget("gripper_close");
    gripper_.move();
    //ros::Duration(2.0).sleep();
    return true;
  }
  float fRand(float min, float max)
  {
    return ((float(rand()) / float(RAND_MAX)) * (max - min)) + min;
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
