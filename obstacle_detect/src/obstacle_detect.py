#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

move_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
noObstacle = True

def react(msg):
	for i in msg.ranges:
		if i<1:
			global noObstacle
			noObstacle = False
			break


def sensorNode():

	rospy.init_node('detect', anonymous=True)

	rospy.Subscriber('/kinect_scan', LaserScan, react)

	while not rospy.is_shutdown():

		if noObstacle:
			move = Twist()
			move.linear.x = 0.1
		else :
			move = Twist()
			move.linear.x = 0
		move_pub.publish(move)


if __name__ == '__main__':
	sensorNode()