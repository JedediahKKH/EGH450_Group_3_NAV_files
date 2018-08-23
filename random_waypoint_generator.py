import numpy as np
import random
import math
import rospy
from contrail_msgs.msg import Waypoint, WaypointList
 
# Define constant values of arena co-ordinates (5mx5mx2.5m)
ARENA_MIN_X=-2.5
ARENA_MAX_X=2.5
ARENA_MIN_Y=-2.5
ARENA_MAX_Y=2.5
ARENA_MIN_Z=0
ARENA_MAX_Z=2.5
SAFETY_MARGIN=0.5

# Define obstacle properties (1mx1mxinf m)
OBS_WIDTH=1.0
OBS_HEIGHT=2.5
# Input obstacle centroid position
obs_x=1.0
obs_y=1.0
# Inaccessible space occupied by obstacle 
obs_x_min=obs_x-OBS_WIDTH/2
obs_x_max=obs_x+OBS_WIDTH/2
obs_y_min=obs_y-OBS_WIDTH/2
obs_y_max=obs_y+OBS_WIDTH/2
obs_z_min=ARENA_MIN_Z
obs_z_max=ARENA_MAX_Z

def check_values(x_pos,y_pos,z_pos,yaw):
	x_pos=float(x_pos)

	valid_x=False
	valid_y=False
	valid_z=False
	valid_yaw=False
	if x_pos<ARENA_MAX_X-SAFETY_MARGIN or x_pos>ARENA_MIN_X+SAFETY_MARGIN:
		# x_pos is within the arena, check if valid with obstacle
		if x_pos<obs_x_min or x_pos>obs_x_max:
			valid_x=True
	if y_pos<ARENA_MAX_Y-SAFETY_MARGIN or y_pos>ARENA_MIN_Y+SAFETY_MARGIN:
		# y_pos is within the arena, check if valid with obstacle
		if y_pos<obs_y_min or y_pos>obs_y_max:
			valid_y=True
	if z_pos<ARENA_MAX_Z-SAFETY_MARGIN or z_pos>ARENA_MIN_Z:
		# z_pos is within the arena, check if valid with obstacle
		if valid_x and valid_y:
			valid_z=True
	if yaw<=2*math.pi and x_pos>=0:
		# valid yaw detected
		valid_yaw=True
	if valid_x and valid_y and valid_z and valid_yaw:
		return True
	else:
		return False 

def generate_pose():
	# Define 4D target pose
	generated_pose=np.zeros([4,1])
	valid_pose= False
	# Generate random values for a possible pose
	g_pose_x=5*random.random()-2.5
	g_pose_y=5*random.random()-2.5
	g_pose_z=random.uniform(0,2.5)
	g_pose_yaw=random.uniform(0,2*math.pi)
	# Check if generated values are admissible
	valid_pose=check_values(g_pose_x,g_pose_y,g_pose_z,g_pose_yaw)

	if valid_pose:
		generated_pose=np.array([[g_pose_x],[g_pose_y],[g_pose_z],[g_pose_yaw]])
	else:
		generated_pose=np.array([[np.inf],[np.inf],[np.inf],[np.inf]])
	print("Generated Waypoint:")
	print(generated_pose)
	return generated_pose

if __name__== "__main__":
	# TO-DO: Setup ROS and Waypoint Publisher (waypointList)
	# Send to uavemulator/waypoints

	# Input number of desired poses
	numPoses=5
	# Initialise the array to store legal poses
	pose_array=np.zeros((4,1))
	# While there are less than numPoses of waypoints, generate more random waypoints
		# generate waypoint
		# Check if the waypoint generated has infinites-> if no, append-> else continue with while loop
		# pose_array should be an array of shape 4 by numPoses <- check the shape
		# Print to screen (for unit test submission) 	
	while(pose_array.shape[1]<numPoses):
		# Generate waypoint
		generated_waypoint=generate_pose()
		if not np.isinf(generated_waypoint).any():
			print("Valid Waypoint Generated")
			# Check if pose_array contains the initialised values
			if (pose_array[:,0]== False).all():
				# First column is all zeros, replace with a valid waypoint
				pose_array=generated_waypoint
				print("Initialised values replaced")
			else:
				# Concatenate with a valid waypoint
				pose_array=np.concatenate((pose_array,generated_waypoint),axis=1)
		else:
			# Invalid waypoint generated
			print("Invalid Waypoint Generated-- Regenerating new waypoint")

	print(pose_array)
	# Compile WaypointList
	# Go thru list of waypoints, put them into msg structure
#	msg_out= WaypointList()
	# Compile Header
	# Edit waypoints in
#	for wp in waypoint_list:
#		waypoint= Waypoint()
		# Compile waypoint in struct
#		msg_out.waypoints.append(waypoint)
		
	# Publish WaypointList
#	pub_wp.publish(msg_out)
