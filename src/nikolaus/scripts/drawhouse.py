#!/usr/bin/env python
import rospy
#import turtlesim
from geometry_msgs.msg import Twist
PI = 3.1415926535897

#draws a Nicolaus house
def drawhouse():
	#create publisher with topic name and topic type
	draw_house_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
	
	#initiate node with name
	rospy.init_node('drawhouse', anonymous=True)

	rate = rospy.Rate(1)

	while not rospy.is_shutdown():
		turn45 = Twist()
		turn45.angular.z = -45*2*PI/360

		turn90r = Twist()
		turn90r.angular.z = -90*2*PI/360

		turn90l = Twist()
		turn90l.angular.z = 90*2*PI/360

		turn135r = Twist()
		turn135r.angular.z = -135*2*PI/360

		turn135l = Twist()
		turn135l.angular.z = 135*2*PI/360

		turn0 = Twist()
		turn0.angular.z = 0

		edge1= Twist()
		edge1.linear.x = 2

		edge0= Twist()
		edge0.linear.x = 0

		edge2= Twist()
		edge2.linear.x = 1.4142

		edge3= Twist()
		edge3.linear.x = 2.8284

		rate.sleep()
		draw_house_pub.publish(turn90l)
		rate.sleep()
		draw_house_pub.publish(edge1)
		rate.sleep()
		draw_house_pub.publish(turn45)
		rate.sleep()
		draw_house_pub.publish(edge2)
		rate.sleep()
		draw_house_pub.publish(turn90r)
		rate.sleep()
		draw_house_pub.publish(edge2)
		rate.sleep()
		draw_house_pub.publish(turn135r)
		rate.sleep()
		draw_house_pub.publish(edge1)
		rate.sleep()
		draw_house_pub.publish(turn135l)
		rate.sleep()
		draw_house_pub.publish(edge3)
		rate.sleep()
		draw_house_pub.publish(turn135r)
		rate.sleep()
		draw_house_pub.publish(edge1)
		rate.sleep()
		draw_house_pub.publish(turn135r)
		rate.sleep()
		draw_house_pub.publish(edge3)
		rate.sleep()
		draw_house_pub.publish(turn135r)
		rate.sleep()
		draw_house_pub.publish(edge1)
		rate.sleep()

if __name__ == '__main__':
	try:
        # Testing our function
		drawhouse()
	except rospy.ROSInterruptException:
		pass