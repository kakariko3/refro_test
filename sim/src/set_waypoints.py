#!/usr/bin/env python

import rospy
import csv
from move_base_msgs.msg import MoveBaseActionGoal

filename = '/home/hanaoka/workspace/src/refro_dandy/sim/src/waypoint.csv'
counter = 0

with open(filename, 'w') as f:
    writer = csv.writer(f)
    writer.writerow(['num', 'x', 'y', 'z', 'q1', 'q2', 'q3', 'q4'])

def callback(data):
    global counter
    pos = data.goal.target_pose.pose
    print "[({0},{1},0.0),(0.0,0.0,{2},{3})],".format(pos.position.x,pos.position.y,pos.orientation.z,pos.orientation.w)
    with open(filename, 'a') as f:
        writer = csv.writer(f)
        writer.writerow([counter,pos.position.x,pos.position.y,0.0,0.0,0.0,pos.orientation.z,pos.orientation.w])
    counter += 1

def listener():

    rospy.init_node('goal_sub', anonymous=True)

    rospy.Subscriber("/move_base/goal", MoveBaseActionGoal, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()

