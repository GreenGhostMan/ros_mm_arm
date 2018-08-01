#!/usr/bin/env python
#---------------------------------------------------------------
# THIS PROGRAM IS SUBSCRIBING "/camera/rgb/image_raw" TOPIC AND |
# PUBLISHING "position_publisher" TOPIC.                        |
# PLEASE ADJUST formula from excel and helper variables         |
#---------------------------------------------------------------

import sys
import rospy
import math
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Pose as obj_pos

#_______For Rotation Matrix_______#
#first_rot  = np.pi # +180 degree
second_rot = np.pi # +180 degree
first_rot = (185.0/180.0)*np.pi #(for degree to Radian conversions )

#________For Displament___________#
dis_x = 28.8 #cm
dis_y = -0.2 #cm
dis_x *= 0.01 #m
dis_y *= 0.01 #m

# (ROTATION MATRIX)
R_X = [ [1,0,0],[0,np.cos(first_rot),-np.sin(first_rot)],[0,np.sin(first_rot),np.cos(first_rot)]]
R_Z = [ [np.cos(second_rot),-np.sin(second_rot),0],[np.sin(second_rot),np.cos(second_rot),0],[0,0,1]]
R_0_C = np.dot(R_X,R_Z)

# (DISPLACEMENT VECTOR)
d_0_C = [ [dis_x],[dis_y],[0] ]

# (HOMOGENEOUS TRANSFORMATION MATRIX)
H_0_C = np.concatenate( (R_0_C,d_0_C),1 )
H_0_C = np.concatenate( (H_0_C, [ [0,0,0,1] ] ), 0)


class tf_calculator:
	def __init__(self):
		self.position_pub = rospy.Publisher("position_publisher_tf",obj_pos,queue_size=10)
		self.position_sub = rospy.Subscriber("position_publisher",obj_pos, self.callback);	

	def callback(self,data):	
		
		x_loc = data.position.x
		y_loc = data.position.y 
		PC = [ [x_loc],[y_loc],[0],[1] ]
		P0 = np.dot(H_0_C,PC)
		X0 = P0[0]
		Y0 = P0[1]
		print(X0,Y0, "meter from base_link")		
		
		obj = obj_pos()
		obj.position.x = X0
		obj.position.y = Y0
		obj.position.z = data.position.z
		obj.orientation.x = 0.0
		obj.orientation.y = 0.0
		obj.orientation.z = 0.0
		obj.orientation.w = 1.0
		self.position_pub.publish(obj)
		
def main(args):
	ic = tf_calculator()
	rospy.init_node('calculate_tf_manual',anonymous=True)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting Down")


if __name__ == '__main__':
	main(sys.argv)
