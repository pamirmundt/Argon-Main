#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy
import tf

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

from argon_msgs.msg import OdometryWithoutCovariance_32
from argon_msgs.msg import TransformStamped_32
from argon_msgs.msg import JointState_32
from argon_msgs.msg import Twist_32

msg = """
Argon Msgs Transformer!
---------------------------
argon_msgs -> ros_msgs
"""

tfbroadcaster = tf.TransformBroadcaster()

odom = Odometry()
odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)

joint_states = JointState()
joint_states_pub = rospy.Publisher("/joint_states", JointState, queue_size=50)

cmd_vel_32 = Twist_32()
cmd_vel_32_pub = rospy.Publisher("/cmd_vel_32", Twist_32, queue_size=50)

def odom_32_Cb(msg):
    odom.header = msg.header
    odom.child_frame_id = msg.child_frame_id
    odom.pose.pose.position = msg.pose.pose.position
    odom.pose.pose.orientation = msg.pose.pose.orientation
    odom.twist.twist.linear = msg.twist.twist.linear
    odom.twist.twist.angular = msg.twist.twist.angular

    #->covariance for pose and twist

    odom_pub.publish(odom)
    
    #print odom

def odom_tf_32_Cb(msg):
    tfbroadcaster.sendTransform((msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z),
        (msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w),
        msg.header.stamp,
        msg.child_frame_id,
        msg.header.frame_id)

    #print msg

def JointState_32_Cb(msg):
    joint_states.header = msg.header
    joint_states.name = msg.name
    joint_states.position = msg.position
    joint_states.velocity = msg.velocity
    joint_states.effort = msg.effort

    joint_states_pub.publish(joint_states)

    #print msg

def cmd_vel_Cb(msg):
    cmd_vel_32.linear = msg.linear
    cmd_vel_32.angular = msg.angular

    cmd_vel_32_pub.publish(cmd_vel_32)
    #print msg

def listener():
    rospy.init_node('listener', anonymous=True) #make node

    rospy.Subscriber('odom_32', OdometryWithoutCovariance_32, odom_32_Cb) #odom_32 Subscriber
    
    rospy.Subscriber('tf_32', TransformStamped_32, odom_tf_32_Cb) #tf_32 Subscriber
    
    rospy.Subscriber('joint_states_32', JointState_32, JointState_32_Cb) #JointState_32 Subscriber

    rospy.Subscriber('cmd_vel', Twist, cmd_vel_Cb)

    rospy.spin()

if __name__ == '__main__':
    print msg

    try:
        listener()
    except:
        print e
