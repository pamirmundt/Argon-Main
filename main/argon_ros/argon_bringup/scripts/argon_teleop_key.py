#!/usr/bin/env python

import rospy

#from argon_msgs.msg import Twist_32
from geometry_msgs.msg import Twist

import sys, select, termios, tty

msg = """
Control Your Turtlebot3!
---------------------------
Moving around:
   q    w    e
   a    s    d
        x

w/x : increase/decrease longitudinal velocity
a/d : increase/decrease transversal velocity
q/e : increase/decrease angular velocity
space key, s : force stop

CTRL-C to quit
"""

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(target_long_vel, target_trans_vel, target_ang_vel):
    return "currently:\tlongitudinal vel %s\t transversal vel %s\t angular vel %s " % (target_long_vel,target_trans_vel,target_ang_vel)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('turtlebot3_teleop')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

    status = 0
    target_long_vel = 0
    target_trans_vel = 0
    target_ang_vel = 0
    control_long_vel = 0
    control_trans_vel = 0
    control_ang_vel = 0

    try:
        print msg
        while(1):
            key = getKey()
            if key == 'w' :
                target_long_vel = target_long_vel + 0.01
                status = status + 1
                print vels(target_long_vel, target_trans_vel, target_ang_vel)
            elif key == 'x' :
                target_long_vel = target_long_vel - 0.01
                status = status + 1
                print vels(target_long_vel, target_trans_vel, target_ang_vel)
            elif key == 'a' :
                target_trans_vel = target_trans_vel - 0.01
                status = status + 1
                print vels(target_long_vel, target_trans_vel, target_ang_vel)
            elif key == 'd' :
                target_trans_vel = target_trans_vel + 0.01
                status = status + 1
                print vels(target_long_vel, target_trans_vel, target_ang_vel)
            elif key == 'q' :
                target_ang_vel = target_ang_vel - 0.01
                status = status + 1
                print vels(target_long_vel, target_trans_vel, target_ang_vel)
            elif key == 'e' :
                target_ang_vel = target_ang_vel + 0.01
                status = status + 1
                print vels(target_long_vel, target_trans_vel, target_ang_vel)
            elif key == ' ' or key == 's' :
                target_long_vel = 0
                control_long_vel = 0
                target_trans_vel = 0
                control_trans_vel = 0
                target_ang_vel = 0
                control_ang_vel = 0
                print vels(0, 0, 0)
            elif status == 14 :
                print msg
                status = 0
            else:
                if (key == '\x03'):
                    break

            control_long_vel = target_long_vel
            control_trans_vel = target_trans_vel
            control_ang_vel = target_ang_vel

            twist = Twist()
            twist.linear.x = control_long_vel; twist.linear.y = control_trans_vel; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = control_ang_vel
            pub.publish(twist)

    except:
        print e

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
