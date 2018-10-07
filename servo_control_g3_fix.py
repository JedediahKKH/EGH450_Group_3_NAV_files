#!/usr/bin/env python
import rospy
from mavros_msgs.msg import ActuatorControl
from std_msgs.msg import String

# Global boolin variables
blue_open = False
orange_open = False

# Use these to identify the servos (-1 later to work with the vector correctly)
blue_servo = 5
orange_servo = 6

# Talker function to publish pwm signal
def main():
	rospy.init_node('actuator_controller', anonymous=True)

	rospy.Subscriber("/release_signal", String, callback)
	pub = rospy.Publisher('/mavros/actuator_control', ActuatorControl, queue_size=10)

	rate = rospy.Duration(1/5.0)

	msg_out = ActuatorControl()
	msg_out.group_mix = 2 # Use group 2 (auxilary controls)
	msg_out.controls = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

	while not rospy.is_shutdown():
		if blue_open:
			act_index = blue_servo - 1
			msg_out.controls[act_index] = 1.0
		else:
			act_index = blue_servo - 1
			msg_out.controls[act_index] = -1.0

		if orange_open:
			act_index = orange_servo - 1
			msg_out.controls[act_index] = 1.0
		else:
			act_index = orange_servo - 1
			msg_out.controls[act_index] = -1.0

		msg_out.header.stamp = rospy.Time.now()
		pub.publish(msg_out)
		rospy.sleep(rate)

def callback(msg_in):
	global blue_open    # Needed to modify global copy of variable
	global orange_open

	string = msg_in.data

	if string == "blue open":
		blue_open = True
		rospy.loginfo(string)
	elif string == "blue close":
		blue_open = False
		rospy.loginfo(string)
	elif string == "orange open":
		orange_open = True
		rospy.loginfo(string)
	elif string == "orange close":
		orange_open = False
		rospy.loginfo(string)

if __name__ == '__main__':
	try:
		main()

	except rospy.ROSInterruptException:
		pass
