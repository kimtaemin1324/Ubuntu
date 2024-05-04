#!/usr/bin/env python
import rospy
import sys, select, os
import sys
import tty
import termios
from geometry_msgs.msg import Twist
from std_msgs.msg import String
#import sys, select, os
velocity = 0
steering = 0
breakcontrol = 1
gear = 0
MAX_Velocity = 255

publisher = rospy.Publisher('/cmd_vel', Twist,queue_size=1)

def getkey():
        fd = sys.stdin.fileno()
        original_attributes = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, original_attributes)
        return ch

def teleop():
    global velocity,steering,breakcontrol,gear
    rospy.init_node('teleop', anonymous=True)
#    rospy.Subscriber("/move_base_simple/goal", PoseStamped, callback)
    rate = rospy.Rate(10) # 10hz
#    try:
    status = 0
    while not rospy.is_shutdown():
        key = getkey()
        if key == 'w':
            velocity = velocity + 15
            #steering = 0 
            status = status + 1
        elif key == 'x':
            velocity = 0
            steering = 0
            status = status + 1
        elif key == 'd':
            steering = steering - 3
            status = status + 1
        elif key == 'a':
            steering = steering + 3
            status = status + 1
        elif key == 's':
            velocity = velocity - 15
            steering = 0
            status = status + 1
        else:
            if (key == '\x03'):
                break
        pubmsg = Twist()
        if velocity >= MAX_Velocity:
            velocity = MAX_Velocity

        if velocity <= -MAX_Velocity:
            velocity = -MAX_Velocity
  
        pubmsg.linear.x = velocity
        pubmsg.angular.z = steering
        publisher.publish(pubmsg)
        print('robot_speed : ' + str(velocity))
	print('robot_angle : ' + str(steering))
        #rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    try:
        teleop()
    except rospy.ROSInterruptException: pass
