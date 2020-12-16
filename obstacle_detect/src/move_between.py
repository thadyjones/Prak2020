#! /usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def move_between_client():
 	client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
 	client.wait_for_server()

 	point1 = MoveBaseGoal()
 	point1.target_pose.header.frame_id = "map"
 	point1.target_pose.header.stamp = rospy.Time.now()
 	point1.target_pose.pose.position.x = -10.0
 	point1.target_pose.pose.position.y = 2.2
 	point1.target_pose.pose.orientation.w = 1.0

 	point2 = MoveBaseGoal()
 	point2.target_pose.header.frame_id = "map"
 	point2.target_pose.header.stamp = rospy.Time.now()
 	point2.target_pose.pose.position.x = -4.5
 	point2.target_pose.pose.position.y = 3.1
 	point2.target_pose.pose.orientation.w = 1.0

 	while not rospy.is_shutdown():
 		client.send_goal(point1)
 		client.wait_for_result()
 		client.send_goal(point2)
 		client.wait_for_result()
 		

if __name__ == '__main__':
 	rospy.init_node('move_between_client')
 	move_between_client()
