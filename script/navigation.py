#!/usr/bin/env python

'''
	This file is the whole patrol process navigation file
	------------------------------------------------------------------
	To run this patrol process file only, you need to run the lines below in command prompt:
	- roslaunch jupiterobot_bringup jupiterobot_bringup.launch
	- roslaunch jupiterobot_navigation rplidar_amcl_demo.launch map_file:=/home/mustar/catkin_ws/src/lab_demo/maps/lab_new_map.yaml
	- roslaunch turtlebot_rviz_launchers view_navigation.launch
	- rosrun lab_demo navigation.py
'''

import rospy
from gtts import gTTS
import os
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult

from std_msgs.msg import String
from math import radians


original = 0

class officeTour:
	def __init__(self):
		rospy.on_shutdown(self.cleanup)
		
		self.task_exec = True
		self.description = ""

		# ================== INITIALIZATION ================== 
		# Subscribe to the move_base action server
		self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

		rospy.loginfo("Waiting for move_base action server...")

		# Wait for the action server to become available
		self.move_base.wait_for_server(rospy.Duration(120))
		rospy.loginfo("Connected to move base server")

		# A variable to hold the initial pose of the robot to be set by the user in RViz
		initial_pose = PoseWithCovarianceStamped()
		rospy.Subscriber('initialpose', PoseWithCovarianceStamped, self.update_initial_pose)
		location_pub1 = rospy.Publisher('location_table', String, queue_size=10)
		location_pub2 = rospy.Publisher('location_rack',String,queue_size=10)
		# Subscribe for the tast_status for all task
		rospy.Subscriber('task_status', String, self.task_status_callback)
		
		# Get the initial pose from the user
		rospy.loginfo("*** Click the 2D Pose Estimate button in RViz to set the robot's initial pose...")
		rospy.wait_for_message('initialpose', PoseWithCovarianceStamped)

		# Make sure we have the initial pose
		while initial_pose.header.stamp == "":
			rospy.sleep(1)
		# =====================================================
			
		rospy.loginfo("Ready to go")
		rospy.sleep(1)

		# Coordinates
		# run amcl, rostopic echo /amcl_pose 
		locations = dict()
		
		coordinate1 = [2.87095, 0.28711, 0.08880, 0.99604]
		coordinate2 = [2.47440, -0.7087, -0.54361, 0.83934]

		# Start navigate to destination flow 
		self.goal = MoveBaseGoal()
		rospy.loginfo("Start Navigation")
		self.goal.target_pose.header.frame_id = 'map'
		self.goal.target_pose.header.stamp = rospy.Time.now()
		rospy.sleep(2)
		
		self.navigate(coordinate1)

		# location_table
		location_pub1.publish("True")
		
		rospy.loginfo("Doing task ---- Waiting for people")
		while self.task_exec and not rospy.is_shutdown():
			rospy.loginfo("Waiting for task to complete...")
			rospy.sleep(1)
		rospy.loginfo("Task completed")
		rospy.sleep(2)
		
		self.navigate(coordinate2)
		
		# location_rack
		location_pub2.publish("True")

		self.task_exec = True	
		rospy.loginfo("Doing task ---- Waiting for ai and arm put objects into rack")
		while self.task_exec and not rospy.is_shutdown():
			rospy.loginfo("Waiting for task to complete...")
			rospy.sleep(1)
		rospy.loginfo("Task completed")
		rospy.sleep(2)

        # After visiting each corner, robot will go back to starting point
		rospy.loginfo("Going back initial point")
		rospy.sleep(2)
		self.goal.target_pose.pose = self.origin
		self.move_base.send_goal(self.goal)
		end_point = self.move_base.wait_for_result(rospy.Duration(300))
		if end_point == 1:
			rospy.loginfo("Reached initial point")
			rospy.sleep(2)
		rospy.Rate(5).sleep()

	def update_initial_pose(self, initial_pose):
		global original
		self.initial_pose = initial_pose
		if original == 0:
			self.origin = self.initial_pose.pose.pose
			original = 1

	def cleanup(self):
		rospy.loginfo("Shutting down navigation	....")
		self.move_base.cancel_goal()
		
	def task_status_callback(self,data):
		status = data.data
		if status == 'True':
			self.task_exec = False
			rospy.loginfo('Done performing task')
    
	def navigate(self,coordinate):
		destination = Pose(Point(coordinate[0], coordinate[1], 0.000), Quaternion(0.0, 0.0, coordinate[2], coordinate[3]))
		self.goal.target_pose.pose = destination
		self.move_base.send_goal(self.goal)
		waiting = self.move_base.wait_for_result(rospy.Duration(300))
		if waiting == 1:
			rospy.loginfo("Reached destination")
			rospy.sleep(2)
			return True
		else:
			rospy.loginfo("Failed to reach destination")
			return False
		
if __name__=="__main__":
	rospy.init_node('navi_point')
	try:
		officeTour()
	except:
		pass