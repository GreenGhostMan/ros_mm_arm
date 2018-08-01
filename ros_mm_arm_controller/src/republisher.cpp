#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <dynamixel_msgs/JointState.h>
#include <typeinfo>

class Republisher 
{
public:
	Republisher(ros::NodeHandle* nodehandle):nh_(*nodehandle) {
		init_all_subscribers();
		init_all_pubishers();
	}
private:
	ros::NodeHandle nh_;
	ros::Subscriber pan_sub,lift_sub,flex_sub,grip_sub;
	ros::Publisher pan_pub,lift_pub,flex_pub,grip_pub;

	std_msgs::Float64 val_from_pan;
	std_msgs::Float64 val_from_lift;
	std_msgs::Float64 val_from_flex;
	std_msgs::Float64 val_from_grip;

	void init_all_subscribers() {
		pan_sub  = nh_.subscribe("/arm_shoulder_pan_joint2/state",1,&Republisher::pan_cb,this);
		lift_sub = nh_.subscribe("/arm_shoulder_lift_joint2/state",1,&Republisher::lift_cb,this);
		flex_sub = nh_.subscribe("/arm_wrist_flex_joint2/state",1,&Republisher::flex_cb,this);
		grip_sub = nh_.subscribe("/gripper_joint2/state",1,&Republisher::grip_cb,this);
	}
	void init_all_pubishers() {
		pan_pub  = nh_.advertise<std_msgs::Float64>("/arm_shoulder_pan_joint/command",1,true);
		lift_pub = nh_.advertise<std_msgs::Float64>("/arm_shoulder_lift_joint/command",1,true);
		flex_pub = nh_.advertise<std_msgs::Float64>("/arm_wrist_flex_joint/command",1,true);
		grip_pub = nh_.advertise<std_msgs::Float64>("/gripper_joint/command",1,true);
	}

	void pan_cb(const dynamixel_msgs::JointState js) {
		val_from_pan.data = js.current_pos;
		//ROS_INFO_STREAM(typeid(js.current_pos).name());
		pan_pub.publish(val_from_pan);
	}
	void lift_cb(const dynamixel_msgs::JointState js) {
		val_from_lift.data = js.current_pos;
		lift_pub.publish(val_from_lift);
	}
	void flex_cb(const dynamixel_msgs::JointState js) {
		val_from_flex.data = js.current_pos;
		flex_pub.publish(val_from_flex);
	}
	void grip_cb(const dynamixel_msgs::JointState js) {
		val_from_grip.data = js.current_pos;
		grip_pub.publish(val_from_grip);
	}
};

int main(int argc,char** argv) {

	ros::init(argc,argv,"republisher_node");
	ros::NodeHandle nh;
	Republisher repub(&nh);

	ros::spin();
	return 0;
}