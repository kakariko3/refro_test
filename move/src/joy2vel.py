#!/usr/bin/env python3
import rospy
import smbus
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

class main():
    def __init__(self):      
        rospy.Subscriber('joy', Joy, self.call_joy_F710, queue_size=10)
        self.pub_vel = rospy.Publisher('joy_nvel', Twist, queue_size=10)
        self.pub_mode = rospy.Publisher('joymode', Bool, queue_size=10)
        self.mode = True

    def call_joy_F710(self,joy_msg): #F710_X
        nvel = Twist()
        mode = Bool()
        if joy_msg.buttons[0] == 1:
            rospy.loginfo ('Push A/Green')
            self.mode = False
        elif joy_msg.buttons[1] == 1:
            rospy.loginfo ('Push B/Red')
            self.mode = True
        elif joy_msg.buttons[2] == 1:
            rospy.loginfo ('Push X/Blue')
        elif joy_msg.buttons[3] == 1:
            rospy.loginfo ('Push Y/Yellow')
        elif joy_msg.buttons[4] == 1:
            rospy.loginfo ('Push LB')
        elif joy_msg.buttons[5] == 1:
            rospy.loginfo ('Push RB')
        elif joy_msg.buttons[6] == 1:
            rospy.loginfo ('Push Back')
        elif joy_msg.buttons[7] == 1:
            rospy.loginfo ('Push Start')

        if joy_msg.axes[7] > 0.5:
            #rospy.loginfo ('Push Cross-Up')
            nvel.linear.x = 1.0
        elif joy_msg.axes[7] < -0.5:
            #rospy.loginfo ('Push Cross-Down')
            nvel.linear.x = -1.0
        else:
            nvel.linear.x = (joy_msg.axes[1] + joy_msg.axes[4]) / 2.0
        if joy_msg.axes[6] > 0.5:
            #rospy.loginfo ('Push Cross-Left')
            nvel.angular.z = 1.0
        elif joy_msg.axes[6] < -0.5:
            #rospy.loginfo ('Push Cross-Right')
            nvel.angular.z = -1.0
        else:
            nvel.angular.z = (joy_msg.axes[4] - joy_msg.axes[1]) / 2.0

        mode.data = self.mode
        self.pub_vel.publish(nvel)
        self.pub_mode.publish(mode)

if __name__ == '__main__':
    rospy.init_node('joy2vel_node',anonymous=True)
    rospy.loginfo('Start joy2vel_node')
    r = rospy.Rate(20) #20Hz
    m = main()
    while not rospy.is_shutdown():
        r.sleep()
