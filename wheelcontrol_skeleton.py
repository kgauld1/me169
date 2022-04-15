#!/usr/bin/env python3
#
#   wheelcontrol_skeleton.py
#
#   This is a skelenton for the implementation of the Wheel-Control node.  Beyond
#   the basic, it should
#     - stops if no commands are received after 0.25sec
#     - filter the commands to avoid spikes (high torque/current loads)
#     - filters the actual velocity
#     - adds feedback to perfectly achieve the velocity commands
#
#   Node:       /wheelcontrol
#   Publish:    /wheel_state            sensor_msgs/JointState
#               /wheel_desired          sensor_msgs/JointState
#   Subscribe:  /wheel_command          sensor_msgs/JointState
#
#   Other Inputs:   Encoder Channels (GPIO)
#   Other Outputs:  Motor Driver Commands (via I2C)
#
import math
import sys
import time
import rospy

import encoder as encode
import driver as drive

from sensor_msgs.msg import JointState

lpos = 0
rpos = 0
vleft = 0
vright= 0
old = 0
lcom = 0
rcom = 0
lint = 0
rint = 0
cmdvel = [0,0]
cmdtime = 0
#
#   Command Callback Function
#
#   Save the command and the time received.
#
def callback_command(msg):
    global cmdvel
    global cmdtime
    # Check the message?
    
    # Note the current time (to timeout the command).
    now = rospy.Time.now()

    # Save...
    cmdvel  = msg.velocity
    cmdtime = now.to_sec()


#
#   Timer Callback Function
#
def callback_timer(event):
    # Note the current time to compute dt and populate the ROS messages.
    global old
    now1 = rospy.Time.now()
    now = now1.to_sec()
    dt = now-old
    global lpos
    global rpos
    global vleft
    global vright
    global lcom
    global rcom
    global lint
    global rint
    global cmdvel
    global cmdtime
    # Process the commands.
    
    ctime = cmdtime
    cvel =cmdvel
    lc = lcom
    rc = rcom
    if now-ctime < .25:
        lc = cvel[0]
        rc = cvel[1]
    lam = .05
    lwcomm = (1-lam)*lcom + lam*lc
    rwcomm = (1-lam)*rcom + lam*rc
    lint = lint + lwcomm*dt
    rint = rint + rwcomm*dt
    lcom = lwcomm
    rcom = rwcomm
    # Process the encoders, convert to wheel angles
    pleft = (encoder.leftencoder()*2*math.pi / (45*16))
    pright = (encoder.rightencoder()*2*math.pi / (45*16))
    const = .2
    vleft = ((1-const)*vleft) + ((const/.01)*(pleft-lpos))
    vright = ((1-const)*vright) + ((const/.01)*(pright-rpos))
    print([vleft,vright])
    # Add feedback?
    

    # Generate motor commands (convert wheel speed to PWM)
    lam2 = .15
    ldesv = lwcomm + lam2*((lint-pleft))
    rdesv = rwcomm + lam2*((rint-pright))
    lpwm = ((abs(ldesv)*10) + 30) * math.copysign(1, ldesv)
    rpwm = ((abs(rdesv)*10) + 30) * math.copysign(1, rdesv)
    # Send wheel commands.
    driver.left(lpwm)
    driver.right(rpwm)

    # Publish the actual wheel state
    lpos = pleft
    rpos = pright
    old = now
    

    msg = JointState()
    msg.header.stamp = now1
    msg.name         = ['leftwheel', 'rightwheel']
    msg.position     = [pleft, pright]
    msg.velocity     = [vleft, vright]
    msg.effort       = [lpwm, rpwm]
    pubact.publish(msg)
    

    # Publish the desired wheel state
    msg = JointState()
    msg.header.stamp = now1
    msg.name         = ['leftwheel', 'rightwheel']
    msg.position     = [lint, rint]
    msg.velocity     = [lwcomm, rwcomm]
    msg.effort       = [lpwm, rpwm]
    pubdes.publish(msg)


#
#   Main Code
#
if __name__ == "__main__":
    # Initialize the ROS node.
    rospy.init_node('wheelcontrol')

    # Inititlize the low level.
    encoder = encode.Encoder()
    driver  = drive.Driver()

    # Create a publisher to send the wheel desired and actual (state).
    pubdes = rospy.Publisher('/wheel_desired', JointState, queue_size=10)
    pubact = rospy.Publisher('/wheel_state',   JointState, queue_size=10)

    # Create a subscriber to listen to wheel commands.
    sub = rospy.Subscriber("/wheel_command", JointState, callback_command)

    # Create the timer.
    DT = .01
    duration = rospy.Duration(DT);
    dt       = duration.to_sec()
    timer    = rospy.Timer(duration, callback_timer)


    # Spin while the callbacks are doing all the work.
    rospy.loginfo("Running with dt = %.3f sec..." % dt)
    rospy.spin()
    rospy.loginfo("Stopping...")

    # Stop the timer (if not already done).
    timer.shutdown()

    # Clean up the low level.
    driver.shutdown()
    encoder.shutdown()
