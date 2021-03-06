#!/usr/bin/env python3
import sys
import math
import rospy
import numpy as np

from geometry_msgs.msg import Twist
from nav_msgs.msg      import Odometry
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan

class LocalPlanner:
	# Initialize the Local Planner
	def __init__(self):
		# Goal coordinates and theta
		self.goal_x = 0
		self.goal_y = 0
		self.goal_th = 0
		self.stage = 0
		self.last_vel = 0
		self.front_dist = 1
		self.blocked = False
		
		self.vmsg = Twist()
		
		# Create a publisher to send velocity commands.
		self.pub_vl_cmd = rospy.Publisher('/vel_cmd', Twist, queue_size = 10)
		
		# Create a subscriber to listen to the odometry.
		# rospy.Subscriber('/odom', Odometry, self.cb_odom)
		
		# Create a subscriber to listen to the pose.
		rospy.Subscriber('/pose', PoseStamped, self.cb_pose)
		
		# Create a subscriber to listen to the LaserScanner
		rospy.Subscriber('/scan', LaserScan, self.cb_laserscan)
		
		# Create a subscriber to listen to the 2D Navigation goal.
		rospy.Subscriber('/move_base_simple/goal', PoseStamped,
							self.cb_pose_stamp)
	
	# Calculate the different between angles			
	def AngleDiff(self, t1, t2):
		return (t1-t2) - math.pi * round((t1-t2)/math.pi)
		
	def Filter(self, slope, cutoff, var):
		if var < 0:
			return max(-cutoff, var * slope)
		else:
			return min(cutoff, var * slope)
			
	# Run this function when a LaserScanner message is received.
	def cb_laserscan(self, msg):
		ranges = msg.ranges
		mid_idx = int(len(ranges)/2)
		new_ranges = ranges[mid_idx-10:mid_idx+10]
		new_ranges = [x for x in ranges if x != 0 and x != 5]
		if len(new_ranges) != 0:
			self.front_dist = min(new_ranges)
		else:
			self.front_dist = min([x for x in ranges if x != 0 and x != 5])
		
	# Run this function when a 2D Nav goal message is received.					
	def cb_pose_stamp(self, msg):
		assert (msg.header.frame_id == 'map'), "Message not in map frame"
		
		# Update goal position.
		self.goal_x = msg.pose.position.x
		self.goal_y = msg.pose.position.y
		self.stage = 0
		# Update goal theta (conversion from quaternion to Euler needed).
		qz = msg.pose.orientation.z
		qw = msg.pose.orientation.w
		
		self.goal_th = math.atan2(2*qz*qw, 1 - 2*qz*qz)
		
	def cb_odom(self, msg):
		self.vmsg.linear.x = 0.0
		self.vmsg.linear.y = 0.0
		self.vmsg.linear.z = 0.0
		self.vmsg.angular.x = 0.0
		self.vmsg.angular.y = 0.0
		self.vmsg.angular.z = 0.0
		
		# Update actual position
		act_x = msg.pose.pose.position.x
		act_y = msg.pose.pose.position.y
		
		# Update actual heading
		qz = msg.pose.pose.orientation.z
		qw = msg.pose.pose.orientation.w
		act_th = math.atan2(2*qz*qw, 1 - 2*qz*qz)
		
		# Compute difference in position and heading to target position
		del_x = self.goal_x - act_x
		del_y = self.goal_y - act_y
		theta_target = math.atan2(del_y, del_x)
		del_th_tar = ((theta_target - act_th) - math.pi) % (2*math.pi) - math.pi
		
		# Compute difference between goal and current heading, and distance
		del_th = self.AngleDiff(self.goal_th, act_th)
		del_th = ((self.goal_th - act_th) - math.pi) % (2*math.pi) - math.pi
		dist = math.sqrt(del_x*del_x + del_y*del_y)
		
		# Compute necessary velocity commands
		if (self.front_dist <= 0.37 and self.stage == 1):
			temp = 0.9 * self.last_vel
			self.vmsg.linear.x = temp
			self.last_vel = temp
			if temp < 0.1:
				self.stage = 3
				# self.blocked = True
		# elif (self.front_dist > 0.2 and self.blocked):
		# 	self.stage == 0
		# 	self.blocked = False
		elif (self.stage == 0):
			# bot is far away and doesnt face the goal
			self.vmsg.angular.z = self.Filter(.8, 2.2, del_th_tar)
			if (abs(del_th_tar) < 0.1 and self.vmsg.angular.z < .001):
			    self.stage = 1
		elif (self.stage == 1):
			# bot is far away and faces the goal
			temp = self.Filter(0.8, 0.5, dist)
			self.vmsg.linear.x = temp
			self.last_vel = temp
			self.vmsg.angular.z = self.Filter(.8, 2, del_th_tar)
			if dist < 0.05:
				self.stage = 2
		elif (self.stage == 2):
			# bot is close to goal, but faces the wrong way
			self.vmsg.angular.z = self.Filter(.8, 2.2, del_th)
			if (abs(del_th) < 0.02 and self.vmsg.angular.z < .001):
			    self.stage = 3
		else:
			# bot is close to goal and faces the right way
			self.vmsg.angular.z = 0
			self.last_vel = 0
	
		self.pub_vl_cmd.publish(self.vmsg)
		
	def cb_pose(self, msg):
		assert (msg.header.frame_id == 'map'), "Message not in map frame"
		
		self.vmsg.linear.x = 0.0
		self.vmsg.linear.y = 0.0
		self.vmsg.linear.z = 0.0
		self.vmsg.angular.x = 0.0
		self.vmsg.angular.y = 0.0
		self.vmsg.angular.z = 0.0
		
		# Update actual position
		act_x = msg.pose.position.x
		act_y = msg.pose.position.y
		
		# Update actual heading
		qz = msg.pose.orientation.z
		qw = msg.pose.orientation.w
		act_th = math.atan2(2*qz*qw, 1 - 2*qz*qz)
		
		# Compute difference in position and heading to target position
		del_x = self.goal_x - act_x
		del_y = self.goal_y - act_y
		theta_target = math.atan2(del_y, del_x)
		del_th_tar = ((theta_target - act_th) - math.pi) % (2*math.pi) - math.pi
		
		# Compute difference between goal and current heading, and distance
		del_th = self.AngleDiff(self.goal_th, act_th)
		del_th = ((self.goal_th - act_th) - math.pi) % (2*math.pi) - math.pi
		dist = math.sqrt(del_x*del_x + del_y*del_y)
		
		# Compute necessary velocity commands
		if (self.front_dist <= 0.37 and self.stage == 1):
			temp = 0.9 * self.last_vel
			self.vmsg.linear.x = temp
			self.last_vel = temp
			if temp < 0.1:
				self.stage = 3
				self.blocked = True
		elif (self.front_dist > 0.2 and self.blocked):
			self.stage == 0
			self.blocked = False
		elif (self.stage == 0):
			# bot is far away and doesnt face the goal
			self.vmsg.angular.z = self.Filter(.8, 2.2, del_th_tar)
			if (abs(del_th_tar) < 0.1 and self.vmsg.angular.z < .001):
			    self.stage = 1
		elif (self.stage == 1):
			# bot is far away and faces the goal
			temp = self.Filter(0.8, 0.5, dist)
			self.vmsg.linear.x = temp
			self.last_vel = temp
			self.vmsg.angular.z = self.Filter(.8, 2, del_th_tar)
			if dist < 0.05:
				self.stage = 2
		elif (self.stage == 2):
			# bot is close to goal, but faces the wrong way
			self.vmsg.angular.z = self.Filter(.8, 2.2, del_th)
			if (abs(del_th) < 0.02 and self.vmsg.angular.z < .001):
			    self.stage = 3
		else:
			# bot is close to goal and faces the right way
			self.vmsg.angular.z = 0
			self.last_vel = 0
	
		self.pub_vl_cmd.publish(self.vmsg)
			
if __name__ == "__main__":
    # Initialize the ROS node.
    rospy.init_node('localplanner')

    # Instantiate the LocalPlanner object
    odometry = LocalPlanner()

    # Report and spin (waiting while callbacks do their work).
    rospy.loginfo("Planner spinning...")
    rospy.spin()
    rospy.loginfo("Planner stopped.")
	
