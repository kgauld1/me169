#!/usr/bin/env python3
#
#   odometry.py
#
#   Odometry node.  This
#   (a) converts both a body velocity command to wheel velocity commands.
#   (b) estimates the body velocity and pose from the wheel motions
#       and the gyroscope.
#
#   Node:       /odometry
#   Publish:    /odom                   geometry_msgs/TransJointState
#               TF odom -> base         geometry_msgs/TransformStamped
#               /wheel_command          sensor_msgs/JointState
#   Subscribe:  /vel_cmd                geometry_msgs/Twist
#               /wheel_state            sensor_msgs/JointState
#
import math
import rospy
import tf2_ros

from geometry_msgs.msg import Point, Quaternion, Twist
from geometry_msgs.msg import TransformStamped, Vector3
from nav_msgs.msg      import Odometry
from sensor_msgs.msg   import JointState


#
#   Constants
#
R = 0.03175           # Wheel radius
d = 0.1317625/2 
d = 0.136525/2          # Halfwidth between wheels
olpst = 0
orpst = 0

#
#   Odometry Object
#
class OdometryObj:
    # Initialize.
    def __init__(self):
        # Set the initial pose to zero.
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Create a publisher to send wheel commands.
        self.pub_wcmd = rospy.Publisher('/wheel_command', JointState,
                                        queue_size=3)

        # Create a publisher to send odometry information.
        self.pub_odom = rospy.Publisher('/odom', Odometry, queue_size=10)

        # Create a TF2 transform broadcaster.
        self.brd_tf = tf2_ros.TransformBroadcaster()

        # Create a subscriber to listen to twist commands.
        rospy.Subscriber('/vel_cmd', Twist, self.cb_vel_cmd)

        # Create a subscriber to listen to wheel state.
        rospy.Subscriber('/wheel_state', JointState, self.cb_wheel_state)


    # Velocity Command Message Callback
    def cb_vel_cmd(self, msg):
        # Grab the forward and spin (velocity) commands.

        #CONVERT THE BODY VELOCITY COMMANDS TO L/R WHEEL COMMANDS
        lpsi_dot = (msg.linear.x-(msg.angular.z*d))/R
        rpsi_dot = (msg.linear.x+(msg.angular.z*d))/R

        # Create the wheel command msg and publish.  Note the incoming
        # message does not have a time stamp, so generate one here.
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name         = ['leftwheel', 'rightwheel']
        msg.velocity     = [lpsi_dot,    rpsi_dot    ]
        self.pub_wcmd.publish(msg)


    # Wheel State Message Callback
    def cb_wheel_state(self, msg):
        global olpst
        global orpst
        # Grab the timestamp, wheel and gyro position/velocities.
        timestamp = msg.header.stamp
        lpst = msg.position[0]
        lvst = msg.velocity[0]
        rpst = msg.position[1]
        rvst = msg.velocity[1]
        hest = msg.position[2]
        omst = msg.velocity[2]
        whst = msg.position[3]
        wost = msg.velocity[3]
        
        lphi = lpst-olpst
        rphi = rpst-orpst
        vx = (R/2) * (lvst + rvst)
        wz = (R/(2*d))*(rvst-lvst)
        
        dp = (R/2)*(lphi+rphi)
        dth = (R/(2*d))*(rphi-lphi)
        
        olpst = lpst
        orpst = rpst
        # Update the pose.
        self.x    += dp * math.cos(self.theta + (dth/2))
        self.y    += dp * math.sin(self.theta + (dth/2))
        self.theta += dth

        # Convert to a ROS Point, Quaternion, Twist (lin&ang veloocity).
        p = Point(self.x, self.y, 0.0)
        q = Quaternion(0.0, 0.0, math.sin(self.theta/2), math.cos(self.theta/2))
        t = Twist(Vector3(vx, 0.0, 0.0), Vector3(0.0, 0.0, wz))

        # Create the odometry msg and publish (reuse the time stamp).
        msg = Odometry()
        msg.header.stamp            = timestamp
        msg.header.frame_id         = 'odom'
        msg.child_frame_id          = 'base'
        msg.pose.pose.position      = p
        msg.pose.pose.orientation   = q
        msg.twist.twist             = t
        self.pub_odom.publish(msg)

        # Create the transform msg and broadcast (reuse the time stamp).
        msg = TransformStamped()
        msg.header.stamp            = timestamp
        msg.header.frame_id         = 'odom'
        msg.child_frame_id          = 'base'
        msg.transform.translation   = p
        msg.transform.rotation      = q
        self.brd_tf.sendTransform(msg)


#
#   Main Code
#
if __name__ == "__main__":
    # Initialize the ROS node.
    rospy.init_node('odometry')

    # Instantiate the Odometry object
    odometry = OdometryObj()

    # Report and spin (waiting while callbacks do their work).
    rospy.loginfo("Odometry spinning...")
    rospy.spin()
    rospy.loginfo("Odometry stopped.")
