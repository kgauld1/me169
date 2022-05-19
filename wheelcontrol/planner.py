#!/usr/bin/env python3
import sys
import math
import rospy
import numpy as np
import matplotlib.pyplot as plt

from geometry_msgs.msg import Twist
from nav_msgs.msg      import Odometry
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid

import pickle

class Planner:
	# Initialize the Local Planner
	def __init__(self):
		# Goal coordinates and theta
		self.act_x = 0
		self.act_y = 0
		self.act_th = 0
		self.goal_x = 0
		self.goal_y = 0
		self.goal_th = 0
		self.stage = 0
		self.last_vel = 0
		self.front_dist = 1
		self.blocked = False
		self.moves = [] # List of points to get to goal
		self.moveind = 0 # Current move
		self.res = 300/7.625
		
		self.vmsg = Twist()
		
		self.mapmsg = rospy.wait_for_message('/map', OccupancyGrid, 30.0)
		self.imdat = np.array(self.mapmsg.data).reshape(300,300)
		
		self.closest = pickle.load(open('/home/tropic/brushfire_out.pickle', 'rb'))
		
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
			
	def dist(self, x1, y1, x2, y2):
		return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)
			
	def NoWallHit(self, x, y):
		u = int((x+3.81)* self.res)
		v = int((y+3.81)* self.res)
		(cu, cv) = self.closest[v][u]
		cx = (cu / self.res) - 3.81
		cy = (cv / self.res) - 3.81
		d = self.dist(x, y, cx, cy)
		return d > 0.05
			
	def isFree(self, x, y):
		u = int((x+3.81)* self.res)
		v = int((y+3.81)* self.res)
		print(self.imdat[v,u])
		return 0 <= self.imdat[v, u] <= 100 and self.NoWallHit(x, y)
		
	def Intermediate(self, x1, y1, x2, y2, frac):
		x = x1 + frac * (x2 - x1)
		y = y1 + frac * (y2 - y1)
		return x, y
		
	def isConnected(self, x1, y1, x2, y2):
		#plt.imshow(self.imdat)
		#plt.scatter((x1+3.81)*self.res,(y1+3.81)*self.res)
		#plt.scatter((x2+3.81)*self.res,(y2+3.81)*self.res)
		#plt.show()
		free = True
		for f in np.array(range(0,101))/100:
			x, y = self.Intermediate(x1, y1, x2, y2, f)
			print(x,y,self.isFree(x,y))
			free = free and self.isFree(x, y)
		return free
			
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
			
		# print(self.imdat[150][150])
		# print(self.isConnected(-1.3984, 2.761, -1.3984, 2.006))
		
	# Run this function when a 2D Nav goal message is received.					
	def cb_pose_stamp(self, msg):
		assert (msg.header.frame_id == 'map'), "Message not in map frame"
		
		# Update goal position.
		self.goal_x = msg.pose.position.x
		self.goal_y = msg.pose.position.y
		self.stage = 0
		self. moveind = 0
		# Update goal theta (conversion from quaternion to Euler needed).
		qz = msg.pose.orientation.z
		qw = msg.pose.orientation.w
		self.goal_th = math.atan2(2*qz*qw, 1 - 2*qz*qz)
		
		#PUT THE ASTAR OR OTHER THING IN HERE, GIVE A LIST OF POINTS AND ANGLES TO GO TO in self.moves
		#OF THE FORM [(X, Y, THETA), (X2, Y2, TH2) ...] LAST ONE SHOULD BE GOAL POINT AND THETA
		print('CONNECTION', self.isConnected(self.act_x, self.act_y, self.goal_x, self.goal_y))
		if (self.isConnected(self.act_x, self.act_y, self.goal_x, self.goal_y)):
			self.moves = [(self.goal_x, self.goal_y, self.goal_th)]
		else:
			start_adj = None
			end_adj = None
			samples = [(2.798, -1.268),(2.798, 0.0025),(2.798, 1.5275),(-0.506,-1.268),(-0.506, 0.0025),(-0.506, 1.5275)]
			for i in samples:
				if self.isConnected(self.act_x, self.act_y, i[0], i[1]):
					start_adj = i
				if self.isConnected(self.goal_x, self.goal_y, i[0], i[1]):
					end_adj = i
			
			if start_adj == None or end_adj == None:
				print("no connection to map!")
				return
					
			adj_dict = {(2.798, -1.268): [(-0.506, -1.268)],
						(2.798, 0.0025): [(-0.506, 0.0025)],
						(2.798, 1.5275): [(-0.506, 1.5275)],
						(-0.506,-1.268): [(2.798, -1.268),(-0.506, 0.0025)],
						(-0.506, 0.0025): [(-0.506,-1.268),(-0.506, 1.5275),(2.798, 0.0025)],
						(-0.506, 1.5275): [(2.798, 1.5275), (-0.506, 0.0025)]}
			
			if self.isConnected(start_adj[0], start_adj[1], end_adj[0], end_adj[1]):
				self.moves = [(start_adj[0], start_adj[1], 0),
							  (end_adj[0], end_adj[1], 0), 
							  (self.goal_x, self.goal_y, self.goal_th)]
				return
			
			shared = None
			for k in adj_dict[start_adj]:
				if k in adj_dict[end_adj]:
					shared = k
			
			if shared is not None:
				self.moves = [(start_adj[0], start_adj[1], 0),
				              (shared[0], shared[1], 0),
							  (end_adj[0], end_adj[1], 0), 
							  (self.goal_x, self.goal_y, self.goal_th)]
				return
			
			c_st2 = adj_dict[start_adj][0]
			c_end2 = adj_dict[end_adj][0]
			self.moves = [(start_adj[0], start_adj[1], 0),
				          (c_st2[0], c_st2[1], 0),
						  (c_end2[0], c_end2[1], 0), 
						  (end_adj[0], end_adj[1], 0), 
						  (self.goal_x, self.goal_y, self.goal_th)]
		
	def cb_pose(self, msg):
		assert (msg.header.frame_id == 'map'), "Message not in map frame"
		
		self.vmsg.linear.x = 0.0
		self.vmsg.linear.y = 0.0
		self.vmsg.linear.z = 0.0
		self.vmsg.angular.x = 0.0
		self.vmsg.angular.y = 0.0
		self.vmsg.angular.z = 0.0
		
		# Update actual position
		self.act_x = msg.pose.position.x
		self.act_y = msg.pose.position.y
		
		# Update actual heading
		qz = msg.pose.orientation.z
		qw = msg.pose.orientation.w
		self.act_th = math.atan2(2*qz*qw, 1 - 2*qz*qz)
		if len(self.moves) == 0: return
		
		(gx, gy, gt) = self.moves[0]
		
		# Compute difference in position and heading to target position
		del_x = gx - self.act_x
		del_y = gy - self.act_y
		theta_target = math.atan2(del_y, del_x)
		del_th_tar = ((theta_target - self.act_th) - math.pi) % (2*math.pi) - math.pi
		
		# Compute difference between goal and current heading, and distance
		del_th = self.AngleDiff(gt, self.act_th)
		del_th = ((gt - self.act_th) - math.pi) % (2*math.pi) - math.pi
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
			self.moves = self.moves[1:]
			if len(self.moves) > 0:
				self.stage = 0
	
		self.pub_vl_cmd.publish(self.vmsg)
			
if __name__ == "__main__":
    # Initialize the ROS node.
    rospy.init_node('localplanner')

    # Instantiate the LocalPlanner object
    odometry = Planner()

    # Report and spin (waiting while callbacks do their work).
    rospy.loginfo("Planner spinning...")
    rospy.spin()
    rospy.loginfo("Planner stopped.")
	
