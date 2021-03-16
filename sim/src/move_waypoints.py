#!/usr/bin/env python

import rospy
import actionlib
import tf
import csv
import time
from nav_msgs.msg import Odometry
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

filename = '/home/hanaoka/workspace/src/refro_dandy/sim/src/waypoint.csv'

#
# waypoints = [
# [(2.43980264664,0.719987869263,0.0),(0.0,0.0,0.703186509598,0.711005437897)],
# [(0.651513695717,1.81621360779,0.0),(0.0,0.0,0.999996771909,-0.0025408995613)],
# [(0.44998344779,4.06985092163,0.0),(0.0,0.0,0.708861209279,0.705347989279)],
# ]


def goal_pose(row):
    id = int(row[0])
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = 'map'
    goal_pose.target_pose.pose.position.x = map(float,row)[1]
    goal_pose.target_pose.pose.position.y = map(float,row)[2]
    goal_pose.target_pose.pose.position.z = map(float,row)[3]
    goal_pose.target_pose.pose.orientation.x = map(float,row)[4]
    goal_pose.target_pose.pose.orientation.y = map(float,row)[5]
    goal_pose.target_pose.pose.orientation.z = map(float,row)[6]
    goal_pose.target_pose.pose.orientation.w = map(float,row)[7]

    return id, goal_pose


if __name__ == '__main__':
    rospy.init_node('patrol')
    listener = tf.TransformListener()

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction) 
    client.wait_for_server()
    listener.waitForTransform("map", "base_link", rospy.Time(), rospy.Duration(4.0))

    with open(filename, 'r') as f:
        counter = 0
        waypoints = csv.reader(f)
        header = next(waypoints)
        way_no = 0
        time.sleep(1)
        for pose in waypoints:
            way_no,goal = goal_pose(pose)
            client.send_goal(goal)
            while True:
                now = rospy.Time.now()
                listener.waitForTransform("map", "base_link", now, rospy.Duration(4.0))
                #
                position, quaternion = listener.lookupTransform("map", "base_link", now)

                #
                if(math.sqrt((position[0]-goal.target_pose.pose.position.x)**2 + (position[1]-goal.target_pose.pose.position.y)**2 ) <= 0.3):
                    print ("Reach Waypoint No: ", way_no)
                    break
                else:
                    rospy.sleep(0.5)
        print('Finish!!')



