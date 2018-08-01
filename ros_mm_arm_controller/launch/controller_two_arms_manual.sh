#!/bin/sh
# Zeroing
gnome-terminal -x bash -c 'rostopic pub /arm_shoulder_pan_joint/command std_msgs/Float64 0.0'
gnome-terminal -x bash -c 'rostopic pub /arm_shoulder_lift_joint/command std_msgs/Float64 0.0'
gnome-terminal -x bash -c 'rostopic pub /arm_wrist_flex_joint/command std_msgs/Float64 0.0'
gnome-terminal -x bash -c 'rostopic pub /gripper_joint/command std_msgs/Float64 0.0'

gnome-terminal -x bash -c 'rostopic pub /arm_shoulder_pan_joint2/command std_msgs/Float64 0.0'
gnome-terminal -x bash -c 'rostopic pub /arm_shoulder_lift_joint2/command std_msgs/Float64 0.0'
gnome-terminal -x bash -c 'rostopic pub /arm_wrist_flex_joint2/command std_msgs/Float64 0.0'
gnome-terminal -x bash -c 'rostopic pub /gripper_joint2/command std_msgs/Float64 0.0'


# torque off
rosservice call /arm_shoulder_pan_joint2/torque_enable false
rosservice call /arm_shoulder_lift_joint2/torque_enable false
rosservice call /arm_wrist_flex_joint2/torque_enable false
rosservice call /gripper_joint2/torque_enable false
