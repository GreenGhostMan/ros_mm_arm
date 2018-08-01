#!/usr/bin/env python
#---------------------------------------------------------------
# THIS PROGRAM IS SUBSCRIBING "/camera/rgb/image_raw" TOPIC AND |
# PUBLISHING "position_publisher" TOPIC.                        |
# PLEASE ADJUST formula from excel and helper variables         |
#---------------------------------------------------------------

import sys
import rospy
import cv2
import math
import numpy as np

from std_msgs.msg import String
from geometry_msgs.msg import Pose as obj_pos
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# Object height as cm
object_height = 2
object_height *= 0.01 # to meter

#_____FORMULA FROM EXCEL____#
# helper variable
y_pixels = 0.0
x_pixels = 0.0
center_pixel = 0.0
x_pixels = center_pixel - x_pixels
pixel_per_cm = 1.0

# formula
#y_cm = use formula with y_pixels
#pixel_per_cm = use formula with y_pixels
x_cm = (x_pixels - center_pixel)/pixel_per_cm

# finally we get pixels to cm conversions ratio
x_ratio = 0.0
y_ratio = 0.0

class pose_finder:
	def __init__(self):
		self.position_pub = rospy.Publisher("position_publisher",obj_pos,queue_size=10)
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.callback)
		## helper variables
		self.capture = False
		self.buffer = CvBridge()		

	def callback(self,data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
		except CvBridgeError as e:
			print(e)
		if not self.capture:
			self.take_first_img(cv_image)

		else:			
			self.run_algorithm(cv_image)

			
	
	def take_first_img(self,cv_image):
		
		(rows,cols,channels) = cv_image.shape
		print (cols,rows)

		gray_1 = cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY)
		cv2.putText(cv_image,'col='+str(cols),(500,30),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,0,255),2,cv2.LINE_AA)
		cv2.putText(cv_image,'row='+str(rows),(500,50),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,0,255),2,cv2.LINE_AA)
			
		cv2.imshow("Press Escape", cv_image)
		cv2.imshow("Debug_1",gray_1)
		#print(gray_1)

		self.buffer = gray_1

		k = cv2.waitKey(3)
		if k == 27:
			cv2.destroyAllWindows()
			self.capture = True

	def run_algorithm(self,cv_image):
		gray_2 = cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY)

		gray_1 = self.buffer
		diff =  np.absolute( np.matrix(np.int16(gray_1)) - np.matrix(np.int16(gray_2)) ) 
		
		diff[diff>255] = 255		
		diff = np.uint8(diff)

		cv2.imshow("Difference",diff)
		cv2.waitKey(3)

		thresh = 100
		bw = diff
		bw[bw>150] = 1
		bw[bw<150] = 0

		# algorithm
		col_sum = np.matrix(np.sum(bw,0))
		col_num = np.matrix(np.arange(640))
		col_mul = np.multiply(col_sum,col_num)
		total = np.sum(col_mul)
		total_total = np.sum(np.sum(bw))
		col_loc = total/total_total
		#print(col_loc)

		x_loc = col_loc * x_ratio
		#x_loc = round(x_loc,2)

		# algorithm
		row_sum = np.matrix(np.sum(bw,1))
		row_sum = row_sum.transpose()
		row_num = np.matrix(np.arange(480))
		row_mul = np.multiply(row_sum,row_num)
		total = np.sum(row_mul)
		total_total = np.sum(np.sum(bw))
		row_loc = total/total_total
		#print(row_loc)

		y_loc = row_loc * y_ratio
		#y_loc = round(y_loc,2)
		if math.isnan(x_loc) == True :
			x_loc = 0.0
			y_loc = 0.0
		print(round(x_loc,2),round(y_loc,2), " cm from camera link")
		
		# cm to meter
		x_loc *= 0.01
		y_loc *= 0.01
		

		obj = obj_pos()
		obj.position.x = x_loc
		obj.position.y = y_loc
		obj.position.z = object_height
		obj.orientation.x = 0.0
		obj.orientation.y = 0.0
		obj.orientation.z = 0.0
		obj.orientation.w = 1.0
		self.position_pub.publish(obj)	
		

def main(args):
	ic = pose_finder()
	rospy.init_node('position_finder',anonymous=True)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting Down")
	cv2.destroyAllWindows()


if __name__ == '__main__':
	main(sys.argv)
