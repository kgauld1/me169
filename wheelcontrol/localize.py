#!/usr/bin/env python3
import sys
import math
import rospy
import numpy as np
import tf2_ros
import matplotlib.pyplot as plt
from PlanarTransform import *

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped

import pickle

class Localize:
	# Initalize the Localization code
	def __init__(self):
		self.points = None
		self.map_to_odom = PlanarTransform.unity()
		self.odom_to_base = PlanarTransform.unity()
		
		rospy.loginfo("Waiting for a map...")
		self.mapmsg = rospy.wait_for_message('/map', OccupancyGrid, 30.0)
		self.imdat = np.array(self.mapmsg.data).reshape(300,300)
		
		self.res = 300/7.625
		self.closest = pickle.load(open('/home/tropic/brushfire_out.pickle', 'rb'))
		
		rospy.Subscriber('/odom', Odometry, self.cb_odom)
		rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.cb_ipose)
		rospy.Subscriber('/scan', LaserScan, self.cb_laserscan)
		
		self.pub_pose = rospy.Publisher('/pose', PoseStamped, queue_size = 10)
		self.pose_msg = PoseStamped()
		
		# First create a TF2 listener. This implicily fills a local
		# buffer, so we can always retrive the transforms we want.
		self.tfBuffer = tf2_ros.Buffer()
		self.tflisten = tf2_ros.TransformListener(self.tfBuffer)
		# Then create a TF2 transform broadcaster.
		self.tfbroadcast = tf2_ros.TransformBroadcaster()
		# Give the broadcaster time to connect, then send the initial transform.
		rospy.sleep(0.25)
		self.tfmsg = TransformStamped()
		self.tfmsg.header.stamp = rospy.Time.now()
		self.tfmsg.header.frame_id = 'map'
		self.tfmsg.child_frame_id = 'odom'
		self.tfmsg.transform = self.map_to_odom.toTransform()
		self.tfbroadcast.sendTransform(self.tfmsg)
		
	def cb_odom(self, msg):		
		self.odom_to_base = PlanarTransform.fromPose(msg.pose.pose)
		
		map_to_base = self.map_to_odom * self.odom_to_base
		
		self.pose_msg = PoseStamped()
		self.pose_msg.header = msg.header
		self.pose_msg.header.frame_id = "map"
		
		self.pose_msg.pose = map_to_base.toPose()

		self.pub_pose.publish(self.pose_msg)

	
	def cb_ipose(self, msg):
		# plt.imshow(self.imdat)
		# plt.plot([x[0] for x in self.points], [x[1] for x in self.points])
		# cx = [(self.closest[x[1]][x[0]])[1] for x in self.points]
		# cy = [(self.closest[x[1]][x[0]])[0] for x in self.points]
		# plt.scatter(cx, cy)
		# plt.show()
		map_to_base = PlanarTransform.fromPose(msg.pose.pose)
		self.map_to_odom = map_to_base * self.odom_to_base.inv()
	
	def cb_laserscan(self, msg):
		self.tfmsg = self.tfBuffer.lookup_transform('odom',
										msg.header.frame_id,
										msg.header.stamp,
										rospy.Duration(0.1))
		
		odom2laser = PlanarTransform.fromTransform(self.tfmsg.transform)
		
		# Angle min/max/increment
		amin = msg.angle_min
		amax = msg.angle_max
		ainc = msg.angle_increment
		# Range min/max
		rmin = msg.range_min
		rmax = msg.range_max
		ranges = msg.ranges
		
		angles = []
		nranges = []
		ang = amin
		for r in ranges:
			if rmin<r<rmax:
				angles.append(ang)
				nranges.append(r)
			ang = ang + ainc
		
		angles = np.array(angles)
		nranges = np.array(nranges)
		x = nranges * np.cos(angles)
		y = nranges * np.sin(angles)
		
		points = []
		
		for i in range(len(nranges)):
			tr = (self.map_to_odom * odom2laser * PlanarTransform(x[i],y[i],0,1))
			
			u = int((tr.x()+3.81)* self.res)
			v = int((tr.y()+3.81)* self.res)
			points.append((u,v))
		
		self.points = points
		
		delta = self.update_fraction(points)
		map2grid = PlanarTransform(-3.81,-3.81,0,1)
		# map2grid = self.res * PlanarTransform(3.81, 3.81, 0, 1)
		nmap2odom = map2grid * PlanarTransform(delta[0], delta[1], np.sin(delta[2]/2), np.cos(delta[2]/2)) * map2grid.inv() * self.map_to_odom
		# nmap2odom = PlanarTransform(delta[0], delta[1], np.sin(delta[2]/2), np.cos(delta[2]/2)) * self.map_to_odom
		self.map_to_odom = nmap2odom
		
		otfmsg = TransformStamped()
		otfmsg.header.stamp = msg.header.stamp
		otfmsg.header.frame_id = 'map'
		otfmsg.child_frame_id = 'odom'
		otfmsg.transform = self.map_to_odom.toTransform()
		self.tfbroadcast.sendTransform(otfmsg)
		
	
	def update_fraction(self, points):
		N = len(points)
		Rx = 0
		Ry = 0
		Px = 0
		Py = 0
		RR = 0
		RP = 0
		for (rx,ry) in points:
			P = self.closest[ry][rx]
			
			Rx += rx/N
			Ry += ry/N
			RR += ((rx)**2 + (ry)**2)/N
			Px += P[1]/N
			Py += P[0]/N
			RP += (rx*P[0] - ry*P[1])/N
		
		# (RR-(Rx**2+Ry**2))
		# if (RR-(Rx**2+Ry**2)) != 0:
		dt = (RP - (Rx*Py-Ry*Px))/(RR-(Rx**2+Ry**2))
		dx = Px - Rx + Ry*dt
		dy = Py - Ry - Rx*dt
		# else:
			#dx = Px - Rx
			#dy = Py - Ry
			# dx = 0
			# dy = 0
			# dt = 0
		frac = 0.05
		return([frac*dx / self.res, frac*dy / self.res, frac*dt])
		# return([frac * dx, frac * dy, frac * dt])

		
if __name__ == "__main__":
    # Initialize the ROS node.
    rospy.init_node('localize')

    # Instantiate the Localize object
    odometry = Localize()

    # Report and spin (waiting while callbacks do their work).
    rospy.loginfo("Localizer spinning...")
    rospy.spin()
    rospy.loginfo("Localizer stopped.")
		
