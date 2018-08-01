#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <dynamixel_controllers/TorqueEnable.h>

void blabla() {

	/*
	// set arm 1 joints are 0.0 radian
	ros::Publisher j1_pan_pub  = nh_.advertise<std_msgs::Float64>("/arm_shoulder_pan_joint/command",1,true);
	ros::Publisher j1_lift_pub = nh_.advertise<std_msgs::Float64>("/arm_shoulder_lift_joint/command",1,true);
	ros::Publisher j1_flex_pub = nh_.advertise<std_msgs::Float64>("/arm_wrist_flex_joint/command",1,true);
	ros::Publisher j1_grip_pub = nh_.advertise<std_msgs::Float64>("/gripper_joint/command",1,true);
	
	// set arm 2 joints are 0.0 radian
	ros::Publisher j2_pan_pub  = nh_.advertise<std_msgs::Float64>("/arm_shoulder_pan_joint2/command",1,true);
	ros::Publisher j2_lift_pub = nh_.advertise<std_msgs::Float64>("/arm_shoulder_lift_joint2/command",1,true);
	ros::Publisher j2_flex_pub = nh_.advertise<std_msgs::Float64>("/arm_wrist_flex_joint2/command",1,true);
	ros::Publisher j2_grip_pub = nh_.advertise<std_msgs::Float64>("/gripper_joint2/command",1,true);
	
	std_msgs::Float64 msg;
	msg.data = 0.0;
	int i = 0;
	while(i < 100) {
	j1_pan_pub.publish(msg);
	j1_lift_pub.publish(msg);
	j1_flex_pub.publish(msg);
	j1_grip_pub.publish(msg);

	j2_pan_pub.publish(msg);
	j2_lift_pub.publish(msg);
	j2_flex_pub.publish(msg);
	j2_grip_pub.publish(msg);
	i += 1;
	}
	*/
}

int main(int argc,char** argv) {
	ros::init(argc,argv,"set_zero_node");
	ros::NodeHandle nh_;
	// torque of arm 2
	dynamixel_controllers::TorqueEnable te;
	te.request.torque_enable = false;
	ros::ServiceClient client1 = nh_.serviceClient<dynamixel_controllers::TorqueEnable>("/arm_shoulder_lift_joint2/torque_enable");
	ros::ServiceClient client2 = nh_.serviceClient<dynamixel_controllers::TorqueEnable>("/arm_shoulder_pan_joint2/torque_enable");
	ros::ServiceClient client3 = nh_.serviceClient<dynamixel_controllers::TorqueEnable>("/arm_wrist_flex_joint2/torque_enable");
	ros::ServiceClient client4 = nh_.serviceClient<dynamixel_controllers::TorqueEnable>("/gripper_joint2/torque_enable");

	if(client1.call(te)) { ROS_INFO("PAN MOTOR TORQUE OFF!"); }
	if(client2.call(te)) { ROS_INFO("LIFT MOTOR TORQUE OFF!"); }
	if(client3.call(te)) { ROS_INFO("FLEX MOTOR TORQUE OFF!"); }
	if(client4.call(te)) { ROS_INFO("GRIPPER MOTOR TORQUE OFF!"); }

	return 0;
}

