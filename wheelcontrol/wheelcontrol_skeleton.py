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
import driver_replacement as drive
import gyro as gyr
import smbus

from sensor_msgs.msg import JointState

lpos = 0
rpos = 0
vleft = 0
vright= 0
old = 0
lwcomm = 0
rwcomm = 0
lint = 0
rint = 0
cmdvel = [0,0]
cmdtime = 0
head = 0
whead = 0
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
    global lwcomm
    global rwcomm
    global lint
    global rint
    global cmdvel
    global cmdtime
    global head
    global whead
    # Process the commands.
    
    ctime = cmdtime
    cvel =cmdvel
    lc = lwcomm
    rc = rwcomm
    if now-ctime < .25:
        lc = cvel[0]
        rc = cvel[1]
    lam = .02
    lwcomm = (1-lam)*lwcomm + lam*lc
    rwcomm = (1-lam)*rwcomm + lam*rc
    lint = lint + lwcomm*(dt)
    rint = rint + rwcomm*(dt)
    # Process the encoders, convert to wheel angles
    pleft = (encoder.leftencoder()*2*math.pi / (45*16))
    pright = (encoder.rightencoder()*2*math.pi / (45*16))
    const = .2
    vleft = ((1-const)*vleft) + ((const/dt)*(pleft-lpos))
    vright = ((1-const)*vright) + ((const/dt)*(pright-rpos))
    # Add feedback?
    

    # Generate motor commands (convert wheel speed to PWM)
    lam2 = .2/.01
    ldesv = lwcomm + lam2*((lint-pleft))
    rdesv = rwcomm + lam2*((rint-pright))

    
    lpwm = ((abs(ldesv)*9) + 40) * math.copysign(1, ldesv)
    rpwm = ((abs(rdesv)*9) + 40) * math.copysign(1, rdesv)
    if lpwm > 255:
        lpwm = 254
    if rpwm > 254:
        rpwm = 254
    # Send wheel commands.
    driver.left(lpwm)
    driver.right(rpwm)

    # Read and integrate Gyro
    (omega, sat) = gyro.read()
    head = head + (dt*(omega))
    
    # Calculate spin from wheels
    womega = -0.03175*(vright-vleft) / (0.1317625)
    whead = whead + dt*womega

    # Publish the actual wheel state
    lpos = pleft
    rpos = pright
    old = now
    

    msg = JointState()
    msg.header.stamp = now1
    msg.name         = ['leftwheel', 'rightwheel', 'rot', 'wrot']
    msg.position     = [pleft, pright, head, whead]
    msg.velocity     = [vleft, vright, omega, womega]
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
    i2cbus = smbus.SMBus(1)
    # Inititlize the low level.
    encoder = encode.Encoder()
    driver  = drive.Driver(i2cbus)
    gyro = gyr.Gyro(i2cbus)

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
    time.sleep(2)
    head = 0
    rospy.spin()
    rospy.loginfo("Stopping...")

    # Stop the timer (if not already done).
    timer.shutdown()

    # Clean up the low level.
    driver.shutdown()
    encoder.shutdown()
