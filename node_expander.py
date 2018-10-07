import rospy
import cv2
import numpy as np

from std_msgs.msg import String

pub=None

def callback(msg_in):
	# Resize 1D array into 2D
	array= np.resize(msg_in.data.info)
	# Dilate obstacle until reasonable
	array_dilated=cv2.dilate(array,1)
	# overwrite input with dilated obstacle map
	msg_in=np.flatten(array_dilated)
	# Publish the msg to dedicated publisher
	pub.publish(msg_in)

def listener():
	rospy.init('grid_expanded')
	pub=rospy.publisher('/grid_expanded',String, queue_size=10)
	rospy.Subscriber('/grid',OC,callback)
	rospy.spin()

